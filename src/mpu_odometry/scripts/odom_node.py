#!/usr/bin/env python

import rospy
from nav_msgs.msg import Odometry
from geometry_msgs.msg import Point, Quaternion, Twist
from sensor_msgs.msg import Imu
from tf.broadcaster import TransformBroadcaster

import math
import smbus

# MPU6050 Register Addresses
MPU6050_CONFIG = 0x1A
MPU6050_PWR_MGMT_1 = 0x6B
MPU6050_ACCEL_XOUT_H = 0x3B
MPU6050_ACCEL_YOUT_H = 0x3D
MPU6050_ACCEL_ZOUT_H = 0x3F
MPU6050_GYRO_XOUT_H = 0x43
MPU6050_GYRO_YOUT_H = 0x45
MPU6050_GYRO_ZOUT_H = 0x47

# MPU6050 Constants
RAD_TO_DEG = 57.2957795
ACCEL_SCALE = 16384.0
GYRO_SCALE = 131.0

# ROS Constants
ODOM_TOPIC = "/odom"

class MPU6050:
    def __init__(self, bus, address):
        self.bus = bus
        self.address = address
        self.bus.write_byte_data(self.address, MPU6050_PWR_MGMT_1, 0)

    def read_raw_data(self, addr):
        high = self.bus.read_byte_data(self.address, addr)
        low = self.bus.read_byte_data(self.address, addr + 1)
        value = (high << 8) + low
        if value >= 0x8000:
            return -((65535 - value) + 1)
        else:
            return value

    def get_acceleration(self):
        x = self.read_raw_data(MPU6050_ACCEL_XOUT_H)
        y = self.read_raw_data(MPU6050_ACCEL_YOUT_H)
        z = self.read_raw_data(MPU6050_ACCEL_ZOUT_H)
        ax = x / ACCEL_SCALE
        ay = y / ACCEL_SCALE
        az = z / ACCEL_SCALE
        return ax, ay, az

    def get_rotation(self):
        x = self.read_raw_data(MPU6050_GYRO_XOUT_H)
        y = self.read_raw_data(MPU6050_GYRO_YOUT_H)
        z = self.read_raw_data(MPU6050_GYRO_ZOUT_H)
        gx = x / GYRO_SCALE
        gy = y / GYRO_SCALE
        gz = z / GYRO_SCALE
        return gx, gy, gz

def talker():
    rospy.init_node('mpu6050_odom_publisher', anonymous=True)
    pub = rospy.Publisher(ODOM_TOPIC, Odometry, queue_size=10)
    imu_pub = rospy.Publisher("/imu", Imu, queue_size=10)
    br = TransformBroadcaster()
    rate = rospy.Rate(10) # 10hz

    # Initialize the MPU6050 sensor
    bus = smbus.SMBus(1)
    sensor = MPU6050(bus, MPU6050_CONFIG)

    # Initialize the odometry values
    x = 0.0
    y = 0.0
    theta = 0.0
    vx = 0.0
    vy = 0.0
    vtheta = 0.0

    # Initialize the ROS odometry message
    odom = Odometry()
    odom.header.frame_id = "odom"
    odom.child_frame_id = "base_link"

    # Loop to read data from the sensor and publish odometry values
    while not rospy.is_shutdown():
        # Read acceleration and rotation values from the MPU6050 sensor
        ax, ay, az = sensor.get_acceleration()
        gx, gy, gz = sensor.get_rotation()

        # Convert rotation values from degrees per second to radians per second
        gx = gx / RAD_TO_DEG
        gy = gy / RAD_TO_DEG
        gz = gz / RAD_TO_DEG

        # Update the odometry values
        dt = 0.1
        delta_x = (vx * math.cos(theta) - vy * math.sin(theta)) * dt
        delta_y = (vx * math.sin(theta) + vy * math.cos(theta)) * dt
        delta_theta = vtheta * dt

        x += delta_x
        y += delta_y
        theta += delta_theta

        vx += (ax * math.cos(theta) - ay * math.sin(theta)) * dt
        vy += (ax * math.sin(theta) + ay * math.cos(theta)) * dt
        vtheta += gz * dt

        # Publish the odometry message
        odom.header.stamp = rospy.Time.now()
        odom.pose.pose.position = Point(x, y, 0.0)
        q = Quaternion()
        q.x = 0.0
        q.y = 0.0
        q.z = math.sin(theta / 2.0)
        q.w = math.cos(theta / 2.0)
        odom.pose.pose.orientation = q
        odom.twist.twist = Twist(Point(vx, vy, 0), Point(0, 0, vtheta))
        pub.publish(odom)

        # Publish the IMU message
        imu = Imu()
        imu.header.stamp = rospy.Time.now()
        imu.header.frame_id = "imu_link"
        imu.linear_acceleration.x = ax
        imu.linear_acceleration.y = ay
        imu.linear_acceleration.z = az
        imu.angular_velocity.x = gx
        imu.angular_velocity.y = gy
        imu.angular_velocity.z = gz
        imu_pub.publish(imu)

        # Broadcast the transform from base_link to odom
        br.sendTransform((x, y, 0),
                         (q.x, q.y, q.z, q.w),
                         rospy.Time.now(),
                         "base_link",
                         "odom")

        rate.sleep()

if __name__ == '__main__':
    try:
        talker()
    except rospy.ROSInterruptException:
        pass
