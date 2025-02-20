# asrob-roverto

Apart from the perception module, the following packages are needed:

## [Adafruit_CircuitPython_BNO055](https://github.com/adafruit/Adafruit_CircuitPython_BNO055.git)
For the IMU

## [OrbbecSDK_ROS2](https://github.com/orbbec/OrbbecSDK_ROS2.git)
For the camera. Follow instructions in repo for building.

To launch, 
```bash
cd OrbbecSDK_ROS2/
source install/setup.bash
ros2 launch orbbec_camera astra_pro_plus.launch.py
```

## [osr-rover-code](https://github.com/nasa-jpl/osr-rover-code/tree/master)
For the rover

## [YDLidar-SDK](https://github.com/YDLIDAR/YDLidar-SDK.git)
For the LiDAR

## [ydlidar_ros2_driver](https://github.com/YDLIDAR/ydlidar_ros2_driver)
For the LiDAR. Needs YDLidar-SDK built and installed. Follow instructions in repo for building.

To launch,
```bash
cd ydlidar_ros2_driver/
source install/setup.bash
ros2 launch ydlidar_ros2_driver ydlidar_launch.py 
```
