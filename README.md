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
python3 calibrate_servos.py 14 90
python3 calibrate_servos.py 13 95
python3 calibrate_servos.py 12 35
```
To turn:
```bash
python3 calibrate_servos.py 15 45
python3 calibrate_servos.py 14 120
python3 calibrate_servos.py 13 65
python3 calibrate_servos.py 12 60
```

To update this in the rover code, the next lines should be changed:
- Line 47 in `osr_ws/src/osr-rover-code/ROS/osr_bringup/launch/osr_launch.py`. Change to `parameters=[{'centered_pulse_widths': [90, 90, 95, 35]}]`
- Line 55 in `osr_ws/src/osr-rover-code/ROS/osr_control/osr_control/servo_control.py`. Change to `for servo_id in [reversed(range(12, 16))]:`
- Line 77 in the same file. Change to `self.kit.servo[15 - ind].angle = angle`

Additionally, because the middle-right wheel has +/- switched, line 284 in `/home/rover/osr_ws_uc3m/src/osr-rover-code/ROS/osr_control/osr_control/roboclaw_wrapper.py` has to be changed to `self.send_velocity_cmd(props["address"], props["channel"], -vel_cmd)`

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

**IMPORTANT**

The parameter file `ydlidar_ros2_driver/params/ydlidar.yaml` has to be the one for the LiDAR we are using. Check [X4-Pro.yaml](https://github.com/YDLIDAR/ydlidar_ros2_driver/blob/master/params/X4-Pro.yaml):
```yaml
ydlidar_ros2_driver_node:
  ros__parameters:
    port: /dev/ttyUSB0
    frame_id: laser_frame
    ignore_array: ""
    baudrate: 128000
    lidar_type: 1
    device_type: 0
    sample_rate: 5
    abnormal_check_count: 4
    fixed_resolution: true
    reversion: true
    inverted: true
    auto_reconnect: true
    isSingleChannel: true
    intensity: false
    support_motor_dtr: false
    angle_max: 180.0
    angle_min: -180.0
    range_max: 12.0
    range_min: 0.1
    frequency: 10.0
    invalid_range_is_inf: false
```
