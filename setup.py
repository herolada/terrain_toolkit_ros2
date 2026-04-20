import os
from glob import glob
from setuptools import find_packages, setup

package_name = 'terrain_toolkit_ros2'

setup(
 name=package_name,
 version='0.1.0',
 packages=find_packages(exclude=['test']),
 data_files=[
     ('share/ament_index/resource_index/packages', ['resource/' + package_name]),
     (os.path.join("share", package_name), ["package.xml"]),
        (os.path.join("share", package_name, "launch"), glob("launch/*")),
   ],
 install_requires=['setuptools'],
 zip_safe=True, 
 maintainer=['Kucera Ales','Herold Adam'],
 maintainer_email=['kuceral4@fel.cvut.cz','herolada@fel.cvut.cz'],
 description='Terrain Toolkit ROS2 Wrapper',
 license='BSD-3-Clause',
 tests_require=['pytest'],
 entry_points={
     'console_scripts': [
             'terrain_toolkit_node = terrain_toolkit_ros2.terrain_toolkit_node:main',
     ],
   },
)
