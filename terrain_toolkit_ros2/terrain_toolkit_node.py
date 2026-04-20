#!/usr/bin/env python3
"""
ROS 2 Kilted interface node for a ROS-less terrain toolkit library.

Subscribes to a LiDAR PointCloud2, transforms it into a map frame, calls the
terrain-toolkit library, and republishes the result as a PointCloud2 with
one field per elevation layer.
"""

import numpy as np
import rclpy
from rclpy.node import Node
from rcl_interfaces.msg import ParameterDescriptor, FloatingPointRange, IntegerRange, SetParametersResult

import tf2_ros
from tf2_ros import TransformException
from geometry_msgs.msg import TransformStamped
import tf_transformations

from sensor_msgs.msg import PointCloud2, PointField
from sensor_msgs_py import point_cloud2 as pc2
from std_msgs.msg import Header

from terrain_toolkit_ros2.terrain_toolkit import FilterConfig, TerrainPipeline, TraversabilityConfig, TerrainMap

# Parameters that require rebuilding the TerrainPipeline when changed.
_PIPELINE_PARAMS = frozenset({
    "resolution", "x_range", "y_range",
    "z_max", "primary", "inpaint", "inpaint_coarse_iters", "inpaint_iters_per_level", "smooth_sigma",
    "trav_max_slope_deg", "trav_max_step_height_m", "trav_max_roughness_m",
    "trav_step_window_radius_m", "trav_roughness_window_radius_m",
    "trav_slope_weight", "trav_step_weight", "trav_roughness_weight",
    "filter_support_radius_m", "filter_support_ratio", "filter_inflation_sigma_m",
    "filter_obstacle_threshold", "filter_obstacle_growth_threshold",
    "filter_rejection_limit_frames", "filter_min_obstacle_baseline",
})


class TerrainToolkitNode(Node):
    """Interface between a LiDAR PointCloud2 topic and the terrain-toolkit library."""

    def __init__(self) -> None:
        super().__init__("elevation_map_interface")

        self._declare_parameters()
        p = self._read_parameters()

        self.lidar_topic: str        = p["lidar_topic"]
        self.map_frame: str          = p["map_frame"]
        self.robot_frame: str        = p["robot_frame"]
        self.resolution: float       = p["resolution"]
        self.x_range: float          = p["x_range"]
        self.y_range: float          = p["y_range"]
        self.square_half_size: float = p["square_half_size"]

        self.tf_buffer   = tf2_ros.Buffer()
        self.tf_listener = tf2_ros.TransformListener(self.tf_buffer, self)

        self.sub = self.create_subscription(
            PointCloud2, self.lidar_topic, self._cloud_callback, 10,
        )
        self.pub = self.create_publisher(PointCloud2, "terrain_map", 10)

        self._build_pipeline(p)

        # Register dynamic-reconfigure callback — called on every ros2 param set.
        self.add_on_set_parameters_callback(self._on_parameters_changed)

        self.get_logger().info(
            f"TerrainToolkitNode ready — "
            f"lidar={self.lidar_topic}, frame={self.robot_frame}, "
            f"res={p['resolution']}m, x_range=±{p['x_range']}m, y_range=±{p['y_range']}m"
        )

    # ------------------------------------------------------------------
    # Parameter helpers
    # ------------------------------------------------------------------

    def _declare_parameters(self) -> None:

        def fp(desc, lo, hi):
            return ParameterDescriptor(
                description=desc,
                floating_point_range=[FloatingPointRange(from_value=lo, to_value=hi, step=0.0)],
            )

        def ip(desc, lo, hi):
            return ParameterDescriptor(
                description=desc,
                integer_range=[IntegerRange(from_value=lo, to_value=hi, step=1)],
            )

        def sp(desc):
            return ParameterDescriptor(description=desc)

        # ROS / sensor
        self.declare_parameter("lidar_topic",      "/lidar/points", sp("PointCloud2 input topic"))
        self.declare_parameter("map_frame",         "map",           sp("Map TF frame (unused)"))
        self.declare_parameter("robot_frame",       "base_link",     sp("Robot TF frame"))
        self.declare_parameter("square_half_size",  10.0,            fp("Half-side of square ROI (m)", 0.5, 200.0))

        # Grid
        self.declare_parameter("resolution", 0.15, fp("Grid cell size (m)", 0.01, 5.0))
        self.declare_parameter("x_range",    12.0, fp("Grid half-extent in x (m)", 0.0, 50.0))
        self.declare_parameter("y_range",    12.0, fp("Grid half-extent in y (m)", 0.0, 50.0))

        # Pipeline
        self.declare_parameter("z_max",                   1.0,   fp("Discard points above this height (m)", -10.0, 50.0))
        self.declare_parameter("primary",                 "max", sp("Height reduction: 'max' | 'mean' | 'min'"))
        self.declare_parameter("inpaint",                 True,  sp("Enable multigrid inpainting"))
        self.declare_parameter("inpaint_coarse_iters",    200,   ip("Inpaint coarse iterations", 1, 10_000))
        self.declare_parameter("inpaint_iters_per_level", 50,    ip("Inpaint iterations per pyramid level", 1, 5_000))
        self.declare_parameter("smooth_sigma",            0.8,   fp("Gaussian smoothing sigma (m)", 0.0, 10.0))

        # Traversability
        self.declare_parameter("trav_max_slope_deg",             60.0, fp("Slope angle that saturates cost to 1 (deg)", 0.0, 90.0))
        self.declare_parameter("trav_max_step_height_m",         0.55, fp("Step height that saturates cost to 1 (m)", 0.0, 5.0))
        self.declare_parameter("trav_max_roughness_m",           0.2,  fp("Roughness that saturates cost to 1 (m)", 0.0, 5.0))
        self.declare_parameter("trav_step_window_radius_m",      0.15, fp("Morphological window radius for step detection (m)", 0.01, 5.0))
        self.declare_parameter("trav_roughness_window_radius_m", 0.3,  fp("Window radius for roughness std-dev (m)", 0.01, 5.0))
        self.declare_parameter("trav_slope_weight",              0.2,  fp("Slope component weight in combined cost", 0.0, 1.0))
        self.declare_parameter("trav_step_weight",               0.2,  fp("Step component weight in combined cost", 0.0, 1.0))
        self.declare_parameter("trav_roughness_weight",          0.6,  fp("Roughness component weight in combined cost", 0.0, 1.0))

        # Filter
        self.declare_parameter("filter_support_radius_m",         0.5,  fp("Neighborhood radius for support check (m)", 0.0, 10.0))
        self.declare_parameter("filter_support_ratio",             0.5,  fp("Min fraction of measured cells to keep", 0.0, 1.0))
        self.declare_parameter("filter_inflation_sigma_m",         0.3,  fp("Gaussian sigma for obstacle dilation (m)", 0.0, 10.0))
        self.declare_parameter("filter_obstacle_threshold",        0.8,  fp("Cost above which a cell is an obstacle source", 0.0, 1.0))
        self.declare_parameter("filter_obstacle_growth_threshold", 2.0,  fp("Reject frame if obstacle count grows by this factor", 1.0, 100.0))
        self.declare_parameter("filter_rejection_limit_frames",    5,    ip("Force-accept after this many consecutive rejections", 1, 1000))
        self.declare_parameter("filter_min_obstacle_baseline",     10,   ip("Skip hysteresis until this many obstacles seen", 0, 100_000))

    def _read_parameters(self) -> dict:
        keys = [
            "lidar_topic", "map_frame", "robot_frame", "square_half_size",
            "resolution", "x_range", "y_range",
            "z_max", "primary", "inpaint", "inpaint_coarse_iters", "inpaint_iters_per_level", "smooth_sigma",
            "trav_max_slope_deg", "trav_max_step_height_m", "trav_max_roughness_m",
            "trav_step_window_radius_m", "trav_roughness_window_radius_m",
            "trav_slope_weight", "trav_step_weight", "trav_roughness_weight",
            "filter_support_radius_m", "filter_support_ratio", "filter_inflation_sigma_m",
            "filter_obstacle_threshold", "filter_obstacle_growth_threshold",
            "filter_rejection_limit_frames", "filter_min_obstacle_baseline",
        ]
        return {k: self.get_parameter(k).value for k in keys}

    def _build_pipeline(self, p: dict) -> None:
        """Construct (or reconstruct) the TerrainPipeline from a parameter dict."""
        traversability = TraversabilityConfig(
            max_slope_deg=p["trav_max_slope_deg"],
            max_step_height_m=p["trav_max_step_height_m"],
            max_roughness_m=p["trav_max_roughness_m"],
            step_window_radius_m=p["trav_step_window_radius_m"],
            roughness_window_radius_m=p["trav_roughness_window_radius_m"],
            slope_weight=p["trav_slope_weight"],
            step_weight=p["trav_step_weight"],
            roughness_weight=p["trav_roughness_weight"],
        )

        filter_cfg = FilterConfig(
            support_radius_m=p["filter_support_radius_m"],
            support_ratio=p["filter_support_ratio"],
            inflation_sigma_m=p["filter_inflation_sigma_m"],
            obstacle_threshold=p["filter_obstacle_threshold"],
            obstacle_growth_threshold=p["filter_obstacle_growth_threshold"],
            rejection_limit_frames=p["filter_rejection_limit_frames"],
            min_obstacle_baseline=p["filter_min_obstacle_baseline"],
        )

        self.pipe = TerrainPipeline(
            resolution=p["resolution"],
            bounds=(-p["x_range"], p["x_range"], -p["y_range"], p["y_range"]),
            z_max=p["z_max"],
            primary=p["primary"],
            inpaint=p["inpaint"],
            inpaint_iters_per_level=p["inpaint_iters_per_level"],
            inpaint_coarse_iters=p["inpaint_coarse_iters"],
            smooth_sigma=p["smooth_sigma"],
            traversability=traversability,
            filter=filter_cfg,
        )

    # ------------------------------------------------------------------
    # Dynamic-reconfigure callback
    # ------------------------------------------------------------------

    def _on_parameters_changed(self, params) -> SetParametersResult:
        """
        Called by ROS 2 before each ros2 param set / rqt_reconfigure update.
        The new values arrive in `params`; current stored values are still the old ones.
        We merge new values over the current parameter snapshot and react accordingly.
        """
        new_values = {param.name: param.value for param in params}

        # Merge new values over current snapshot so _build_pipeline sees a complete dict.
        merged = self._read_parameters()
        merged.update(new_values)

        # Update cheap stored fields immediately.
        for attr in ("map_frame", "robot_frame", "resolution", "x_range", "y_range", "square_half_size"):
            if attr in new_values:
                setattr(self, attr, new_values[attr])

        # lidar_topic change requires recreating the subscription.
        if "lidar_topic" in new_values and new_values["lidar_topic"] != self.lidar_topic:
            self.destroy_subscription(self.sub)
            self.lidar_topic = new_values["lidar_topic"]
            self.sub = self.create_subscription(
                PointCloud2, self.lidar_topic, self._cloud_callback, 10,
            )
            self.get_logger().info(f"Resubscribed to {self.lidar_topic}")

        # Rebuild pipeline if any pipeline parameter changed.
        if _PIPELINE_PARAMS & new_values.keys():
            try:
                self._build_pipeline(merged)
                self.get_logger().info("TerrainPipeline rebuilt with new parameters.")
            except Exception as exc:
                return SetParametersResult(successful=False, reason=str(exc))

        return SetParametersResult(successful=True)

    # ------------------------------------------------------------------
    # Main callback
    # ------------------------------------------------------------------

    def _cloud_callback(self, msg: PointCloud2) -> None:
        source_frame = msg.header.frame_id
        stamp        = msg.header.stamp

        try:
            self.tf_buffer.lookup_transform(
                self.robot_frame, source_frame, stamp,
                timeout=rclpy.duration.Duration(seconds=0.1),
            )
        except TransformException as exc:
            self.get_logger().warn(f"TF lookup failed: {exc}")
            return

        points_xyz = self._transform_pointcloud_xyz(msg, self.robot_frame, self.tf_buffer)

        if points_xyz is None or points_xyz.shape[0] == 0:
            self.get_logger().warn("Received empty / invalid point cloud – skipping.")
            return

        try:
            terrain_map: TerrainMap = self.pipe.process(points_xyz)
        except Exception as exc:
            self.get_logger().error(f"terrain toolkit error: {exc}")
            return

        out_cloud = self._grid_to_cloud(
            terrain_map=terrain_map,
            x_min=-self.x_range,
            y_min=-self.y_range,
            resolution=self.resolution,
            stamp=stamp,
        )
        self.pub.publish(out_cloud)

    # ------------------------------------------------------------------
    # Utility: PointCloud2 → numpy
    # ------------------------------------------------------------------

    def pointcloud2_to_xyz_array(self, msg: PointCloud2) -> np.ndarray:
        pc = pc2.read_points(msg, field_names=("x", "y", "z"), skip_nans=True,
                             reshape_organized_cloud=False)
        if isinstance(pc, np.ndarray) and pc.dtype.names is not None:
            xyz = np.stack([pc['x'], pc['y'], pc['z']], axis=-1)
        else:
            xyz = np.array(list(pc), dtype=np.float32)
        return xyz.astype(np.float32)

    # ------------------------------------------------------------------
    # Utility: grid → PointCloud2
    # ------------------------------------------------------------------

    def _grid_to_cloud(
        self,
        terrain_map: TerrainMap,
        x_min: float,
        y_min: float,
        resolution: float,
        stamp,
    ) -> PointCloud2:
        rows, cols = terrain_map.elevation.shape  # (ny, nx): dim0=y rows, dim1=x cols

        # Build coordinate arrays: dim0 index → y, dim1 index → x
        row_idx = np.arange(rows, dtype=np.float32)
        col_idx = np.arange(cols, dtype=np.float32)
        row_grid, col_grid = np.meshgrid(row_idx, col_idx, indexing="ij")  # (rows, cols)

        x_coords = (x_min + (col_grid + 0.5) * resolution).astype(np.float32)
        y_coords = (y_min + (row_grid + 0.5) * resolution).astype(np.float32)

        valid = np.isfinite(terrain_map.elevation)

        x_valid = x_coords[valid]
        y_valid = y_coords[valid]
        z_valid = terrain_map.elevation[valid].astype(np.float32)

        terrain_map_dict = terrain_map.as_dict()
        layers_valid = [
            terrain_map_dict[k][valid].astype(np.float32) for k in sorted(terrain_map_dict.keys())
        ]

        n_pts = x_valid.shape[0]
        point_data = np.column_stack([x_valid, y_valid, z_valid] + layers_valid)

        fields: list[PointField] = []
        offset = 0
        for name in ["x", "y", "z"]:
            fields.append(PointField(name=name, offset=offset, datatype=PointField.FLOAT32, count=1))
            offset += 4
        for k in sorted(terrain_map_dict.keys()):
            fields.append(PointField(name=k, offset=offset, datatype=PointField.FLOAT32, count=1))
            offset += 4

        header = Header()
        header.stamp    = stamp
        header.frame_id = self.robot_frame

        cloud_msg = PointCloud2()
        cloud_msg.header      = header
        cloud_msg.height      = 1
        cloud_msg.width       = n_pts
        cloud_msg.fields      = fields
        cloud_msg.is_bigendian = False
        cloud_msg.point_step  = offset
        cloud_msg.row_step    = offset * n_pts
        cloud_msg.is_dense    = True
        cloud_msg.data        = point_data.astype(np.float32).tobytes()
        return cloud_msg

    # ------------------------------------------------------------------
    # Utility: transform PointCloud2 XYZ to target frame
    # ------------------------------------------------------------------

    def _transform_pointcloud_xyz(
        self,
        cloud_msg: PointCloud2,
        target_frame: str,
        tf_buffer: tf2_ros.Buffer,
    ) -> np.ndarray | None:
        try:
            transform: TransformStamped = tf_buffer.lookup_transform(
                target_frame, cloud_msg.header.frame_id, cloud_msg.header.stamp
            )
        except TransformException as exc:
            self.get_logger().error(f"TF lookup failed in transform: {exc}")
            return None

        points = self.pointcloud2_to_xyz_array(cloud_msg)
        if points.size == 0:
            return np.empty((0, 3), dtype=np.float32)

        t = transform.transform.translation
        r = transform.transform.rotation
        T = tf_transformations.quaternion_matrix([r.x, r.y, r.z, r.w])
        T[0, 3] = t.x
        T[1, 3] = t.y
        T[2, 3] = t.z

        ones = np.ones((points.shape[0], 1), dtype=np.float32)
        return (T @ np.hstack((points, ones)).T).T[:, :3]


# ---------------------------------------------------------------------------
# Entry point
# ---------------------------------------------------------------------------

def main(args=None) -> None:
    rclpy.init(args=args)
    node = TerrainToolkitNode()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.try_shutdown()


if __name__ == "__main__":
    main()
