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
For the rover.

### Calibration
Calibrate servos by running `python3 calibrate_servos.py [id] [angle]` from `~/osr_ws/src/osr-rover-code/scripts`. Since the PCA9685 is inverted, `id` can take value 15 (back right), 14 (front right), 13 (front left) or 12 (back left). After calibration, the best angles have been found to be the following:
```bash
python3 calibrate_servos.py 15 90
python3 calibrate_servos.py 14 150
python3 calibrate_servos.py 13 45
python3 calibrate_servos.py 12 70
```

To update this in the rover code, the next lines should be changed:
- Line 47 in `osr_ws/src/osr-rover-code/ROS/osr_bringup/launch/osr_launch.py`. Change to `parameters=[{'centered_pulse_widths': [90, 150, 45, 70]}]`
- Line 55 in `osr_ws/src/osr-rover-code/ROS/osr_control/osr_control/servo_control.py`. Change to `for servo_id in reversed(range(16, 12)):`
- Line 65 in the same file. Change to `for ind, corner_name in zip(reversed(range(16, 12)), self.corner_motors):`

### INA260
INA260 is at I2C address 0x40. To check I2C addresses, run `i2cdetect -y 1`. Check INA is working with 
```bash
python3 test_ina260.py 40
```

### Bringup
Start the robot by running
```bash
cd osr_ws/src/osr-rover-code/
colcon build
source install/setup.bash
ros2 launch osr_bringup osr_launch.py
```

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
