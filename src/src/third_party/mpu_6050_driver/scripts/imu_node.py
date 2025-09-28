#!/usr/bin/env python
import rospy
import smbus
import numpy as np
from sensor_msgs.msg import Temperature, Imu
from tf.transformations import quaternion_about_axis

bus = None
ADDR = None
IMU_FRAME = None

PWR_MGMT_1 = 0x6b

ACCEL_CONFIG = 0x1C
ACCEL_XOUT_H = 0x3B
ACCEL_XOUT_L = 0x3C
ACCEL_YOUT_H = 0x3D
ACCEL_YOUT_L = 0x3E
ACCEL_ZOUT_H = 0x3F
ACCEL_ZOUT_L = 0x40

GYRO_CONFIG = 0x1B
GYRO_XOUT_H = 0x43
GYRO_XOUT_L = 0x44
GYRO_YOUT_H = 0x45
GYRO_YOUT_L = 0x46
GYRO_ZOUT_H = 0x47
GYRO_ZOUT_L = 0x48

TEMP_H = 0x41
TEMP_L = 0x42

def read_word(adr):
    high = bus.read_byte_data(ADDR, adr)
    low = bus.read_byte_data(ADDR, adr+1)
    val = (high << 8) + low
    return val

def read_word_2c(adr):
    val = read_word(adr)
    if (val >= 0x8000):
        return -((65535 - val) + 1)
    else:
        return val

def publish_temp(timer_event):
    try:
        temp_msg = Temperature()
        temp_msg.header.frame_id = IMU_FRAME
        temp_msg.temperature = read_word_2c(TEMP_H)/340.0 + 36.53
        temp_msg.header.stamp = rospy.Time.now()
        temp_pub.publish(temp_msg)
    except BaseException as e:
        print(e)

imu_msg = Imu()
imu_msg.orientation_covariance = [1e6, 0, 0, 0, 1e6, 0, 0, 0, 1e-6]
imu_msg.angular_velocity_covariance = [1e6, 0, 0, 0, 1e6, 0, 0, 0, 1e-6]
imu_msg.linear_acceleration_covariance = [-1,0,0,0,0,0,0,0,0]
def publish_imu(timer_event):
    try:
        # Read the acceleration vals
        accel_x = read_word_2c(ACCEL_XOUT_H) / 16384.0
        accel_y = read_word_2c(ACCEL_YOUT_H) / 16384.0
        accel_z = read_word_2c(ACCEL_ZOUT_H) / 16384.0
        
        # Read the gyro vals
        gyro_x = read_word_2c(GYRO_XOUT_H) / 131.0
        gyro_y = read_word_2c(GYRO_YOUT_H) / 131.0
        gyro_z = read_word_2c(GYRO_ZOUT_H) / 131.0
        
        imu_msg.linear_acceleration.x = accel_x*9.8
        imu_msg.linear_acceleration.y = accel_y*9.8
        imu_msg.linear_acceleration.z = accel_z*9.8

        imu_msg.angular_velocity.x = gyro_x*0.0174
        imu_msg.angular_velocity.y = gyro_y*0.0174
        imu_msg.angular_velocity.z = gyro_z*0.0174
        
        imu_msg.header.stamp = rospy.Time.now()

        imu_pub.publish(imu_msg)
    except BaseException as e:
        print(e)

if __name__ == '__main__':
    rospy.init_node('imu_node')

    bus = smbus.SMBus(rospy.get_param('~bus', 1))
    ADDR = rospy.get_param('~device_address', 0x68)
    if type(ADDR) == str:
        ADDR = int(ADDR, 16)

    IMU_FRAME = rospy.get_param('~imu_frame', 'imu_link')
    imu_raw = rospy.get_param('~imu_raw', '/imu/imu_raw')
    freq = rospy.get_param('~freq', 50)
    while True:
        try:
            bus.write_byte_data(ADDR, PWR_MGMT_1, 0)
            break
        except Exception as e:
            rospy.logerr(str(e))
    
    imu_msg.header.frame_id = IMU_FRAME

    temp_pub = rospy.Publisher('temperature', Temperature, queue_size=2)
    imu_pub = rospy.Publisher(imu_raw, Imu, queue_size=2)
    imu_timer = rospy.Timer(rospy.Duration(1.0/freq), publish_imu)
    temp_timer = rospy.Timer(rospy.Duration(10), publish_temp)
    rospy.spin()
