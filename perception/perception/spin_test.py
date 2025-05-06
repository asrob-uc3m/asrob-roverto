# A short and sweet script to test movement of all roboclaw motor channels

from time import sleep
import sys
from os import path
# need to add the roboclaw.py file in the path
sys.path.append(path.join(path.expanduser('~'), 'osr_ws_uc3m/src/osr-rover-code/ROS/osr_control/osr_control'))
from roboclaw import Roboclaw
from adafruit_servokit import ServoKit

BAUD_RATE = 115200

def test_connection(address):
    roboclaw0 = Roboclaw("/dev/serial0", BAUD_RATE)
    roboclaw1 = Roboclaw("/dev/serial1", BAUD_RATE)
    connected0 = roboclaw0.Open() == 1
    connected1 = roboclaw1.Open() == 1
    if connected0:
        print("Connected to /dev/serial0.")
        print(f"version: {roboclaw0.ReadVersion(address)}")
        print(f"encoders: {roboclaw0.ReadEncM1(address)}")
        return roboclaw0
    elif connected1:
        print("Connected to /dev/serial1.")
        print(f"version: {roboclaw1.ReadVersion(address)}")
        print(f"encoders: {roboclaw1.ReadEncM1(address)}")
        return roboclaw1
    else:
        print("Could not open comport /dev/serial0 or /dev/serial1, make sure it has the correct permissions and is available")
        return None
    

if __name__ == "__main__":
    
    address1 = 128
    address2 = 129
    address3 = 130
    
    rc1 = test_connection(address1)
    rc2 = test_connection(address2)
    rc3 = test_connection(address3)

    ## Set accel
    drive_accel = 26213
    ## Set speed
    drive_vel = 14652

    motor_index = [15, 14, 13, 12]
    target_angle = [35, 125, 70, 70]

    kit = ServoKit(channels=16)
    sleep(0.1)
    for index, angle in zip(motor_index, target_angle):
        kit.servo[index].actuation_range = 300
        kit.servo[index].set_pulse_width_range(500, 2500)
        kit.servo[index].angle = angle
        self.get_logger().info(f'Servo motor at channel {index} was set to {angle}')
    
    rc1.DutyAccelM1(address1, drive_accel, drive_vel)
    rc1.DutyAccelM2(address1, drive_accel, drive_vel)
    rc2.DutyAccelM1(address2, drive_accel, -drive_vel * -1)
    rc2.DutyAccelM2(address2, drive_accel, drive_vel)
    rc3.DutyAccelM1(address3, drive_accel, -drive_vel * -1)
    rc3.DutyAccelM2(address3, drive_accel, drive_vel * -1)
    sleep(5)
    rc1.ForwardM1(address1, 0)
    rc1.ForwardM2(address1, 0)
    rc2.ForwardM1(address2, 0)
    rc2.ForwardM2(address2, 0)
    rc3.ForwardM1(address3, 0)
    rc3.ForwardM2(address3, 0)
    sleep(1)
    rc1.DutyAccelM1(address1, drive_accel, -drive_vel)
    rc1.DutyAccelM2(address1, drive_accel, -drive_vel)
    rc2.DutyAccelM1(address2, drive_accel, drive_vel)
    rc2.DutyAccelM2(address2, drive_accel, -drive_vel)
    rc3.DutyAccelM1(address3, drive_accel, drive_vel)
    rc3.DutyAccelM2(address3, drive_accel, -drive_vel)
    sleep(1)
    rc1.ForwardM1(address1, 0)
    rc1.ForwardM2(address1, 0)
    rc2.ForwardM1(address2, 0)
    rc2.ForwardM2(address2, 0)
    rc3.ForwardM1(address3, 0)
    rc3.ForwardM2(address3, 0)
