import os
from glob import glob
from setuptools import setup

package_name = 'asrob-roverto'

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
 description='ROVERTO package',
 license='TODO: License declaration',
 tests_require=['pytest'],
 entry_points={
     'console_scripts': [
             'box_color = asrob-roverto.box_color_detection:main',
             'aruco = asrob-roverto.aruco_detection:main',
             'number = asrob-roverto.number_detection:main',
             'lidar = asrob-roverto.lidar_detection:main',
             'prueba1 = asrob-roverto.prueba1:main',
             'prueba2 = asrob-roverto.prueba2:main',
             'og_prueba2 = asrob-roverto.og_prueba2:main',
             'prueba3 = asrob-roverto.prueba3:main',
             'prueba4 = asrob-roverto.prueba4:main'
     ],
   },
)
