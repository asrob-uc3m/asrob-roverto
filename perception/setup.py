import os
from glob import glob
from setuptools import setup

package_name = 'perception'

setup(
 name=package_name,
 version='0.0.0',
 packages=[package_name],
 data_files=[
     ('share/ament_index/resource_index/packages',
     ['resource/' + package_name]),
     ('share/' + package_name, ['package.xml']),
     (os.path.join('share', package_name, 'launch'), glob(os.path.join('launch', '*launch.[pxy][yma]*')))
 ],
 install_requires=['setuptools'],
 zip_safe=True,
 maintainer='anacg',
 maintainer_email='anacg1620@gmail.com',
 description='ROVERTO perception package',
 license='TODO: License declaration',
 tests_require=['pytest'],
 entry_points={
     'console_scripts': [
             'box_color = perception.box_color_detection:main',
             'aruco = perception.aruco_detection:main',
             'number = perception.number_detection:main',
             'lidar = perception.lidar_detection:main',
             'prueba1 = perception.prueba1:main',
             'prueba2 = perception.prueba2:main',
             'prueba3 = perception.prueba3:main',
             'prueba4 = perception.prueba4:main'
     ],
   },
)
