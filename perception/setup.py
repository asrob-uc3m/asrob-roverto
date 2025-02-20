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
             'number = perception.number_detection:main'
     ],
   },
)
