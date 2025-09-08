# 🤖 ROVER.T.O

Welcome to the **ASROB Roverto** repository!  
This project contains the ROS 2 packages developed by students from **ASROB**, the robotics association at **Universidad Carlos III de Madrid**, for the **Bot Talent** competition by SENER-CEA.

The competition involved **perception**, **control**, and **navigation** tasks.  
This repo contains all the implemented solutions in the **asrob-roverto** module, but also includes setup instructions and dependencies for the entire rover system.

---

## 📦 Required Dependencies

To run the complete system, make sure the following packages are installed and properly set up:

### 1. [Adafruit_CircuitPython_BNO055](https://github.com/adafruit/Adafruit_CircuitPython_BNO055.git)  
Used for the IMU (BNO055).

---

### 2. [OrbbecSDK_ROS2](https://github.com/orbbec/OrbbecSDK_ROS2.git)  
Used to interface with the **Orbbec Astra Pro Plus** camera.

#### Launch Instructions:
```bash
cd OrbbecSDK_ROS2/
source install/setup.bash
ros2 launch orbbec_camera astra_pro_plus.launch.py
````

---

### 3. [osr-rover-code](https://github.com/nasa-jpl/osr-rover-code/tree/master)

Base code for the rover (from NASA JPL).

#### 🛠️ Servo Calibration

To calibrate servos:

```bash
cd ~/osr_ws/src/osr-rover-code/scripts
python3 calibrate_servos.py [id] [angle]
```

Servo IDs (due to PCA9685 inversion):

* `15`: Back Right
* `14`: Front Right
* `13`: Front Left
* `12`: Back Left

##### ✅ Recommended Center Angles:

```bash
python3 calibrate_servos.py 15 90
python3 calibrate_servos.py 14 90
python3 calibrate_servos.py 13 95
python3 calibrate_servos.py 12 35
```

##### ↪️ To Turn:

```bash
python3 calibrate_servos.py 15 45
python3 calibrate_servos.py 14 120
python3 calibrate_servos.py 13 65
python3 calibrate_servos.py 12 60
```

#### 🔧 Code Changes for Calibration

Make the following updates to apply calibration:

* **`osr_launch.py` (line 47)**

  ```python
  parameters=[{'centered_pulse_widths': [90, 90, 95, 35]}]
  ```

* **`servo_control.py` (line 55)**

  ```python
  for servo_id in [reversed(range(12, 16))]:
  ```

* **`servo_control.py` (line 77)**

  ```python
  self.kit.servo[15 - ind].angle = angle
  ```

* **Motor fix**: In `roboclaw_wrapper.py` (line 284), fix reversed motor wiring:

  ```python
  self.send_velocity_cmd(props["address"], props["channel"], -vel_cmd)
  ```

---

### 4. INA260 Current Sensor

* Address: `0x40` on I2C

* To check I2C devices:

  ```bash
  i2cdetect -y 1
  ```

* Test INA260:

  ```bash
  python3 test_ina260.py 40
  ```

---

### 5. 🚀 Bringup Instructions

To launch the full robot stack:

```bash
cd osr_ws/src/osr-rover-code/
colcon build
source install/setup.bash
ros2 launch osr_bringup osr_launch.py
```

---

### 6. [YDLidar-SDK](https://github.com/YDLIDAR/YDLidar-SDK.git)

Required backend SDK for the YD LiDAR.

---

### 7. [ydlidar\_ros2\_driver](https://github.com/YDLIDAR/ydlidar_ros2_driver)

ROS 2 driver for YD LiDAR. Requires the SDK above to be built first.

#### Launch Instructions:

```bash
cd ydlidar_ros2_driver/
source install/setup.bash
ros2 launch ydlidar_ros2_driver ydlidar_launch.py 
```

> ⚠️ **IMPORTANT:**
> Update the parameter file to match your device.
> Use the [X4-Pro.yaml](https://github.com/YDLIDAR/ydlidar_ros2_driver/blob/master/params/X4-Pro.yaml) as a reference:

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

---

## 📬 Questions? Contributions?

Feel free to open an [issue](https://github.com/your-repo/issues) or create a pull request if you'd like to contribute or report a bug.

---

## 🧑‍💻 Credits

Developed by members of **ASROB** (UC3M) for the **Bot Talent** competition. Check commit history for updated list of contributors. Thanks to ChatGPT for re-writing this README.
