#! /usr/bin/env python

import navio.adc
import navio.mpu9250
import navio.util

import rospy
from std_msgs.msg import Float32MultiArray
from geometry_msgs.msg import Twist


class AccelGyroMag(object):
    def __init__(self):
        self.pub = rospy.Publisher('/imu/data', Float32MultiArray, queue_size=1)
        self.imu_sensor = rospy.get_param("/accel_gyro_mag/imu_sensor")
        self.imu = None
       

    def get_data(self):

        # imu.read_all()
        # imu.read_gyro()
        # imu.read_acc()
        # imu.read_temp()
        # imu.read_mag()

        # print "Accelerometer: ", imu.accelerometer_data
        # print "Gyroscope:     ", imu.gyroscope_data
        # print "Temperature:   ", imu.temperature
        # print "Magnetometer:  ", imu.magnetometer_data

        m9a, m9g, m9m = self.imu.getMotion9()

        print "Acc:", "{:+7.3f}".format(m9a[0]), "{:+7.3f}".format(m9a[1]), "{:+7.3f}".format(m9a[2]),
        print " Gyr:", "{:+8.3f}".format(m9g[0]), "{:+8.3f}".format(m9g[1]), "{:+8.3f}".format(m9g[2]),
        print " Mag:", "{:+7.3f}".format(m9m[0]), "{:+7.3f}".format(m9m[1]), "{:+7.3f}".format(m9m[2])

    def initialize(self):
        if self.imu_sensor == 'mpu':
            self.imu = navio.mpu9250.MPU9250()
        elif self.imu_sensor == 'lsm':
            self.imu = navio.lsm9ds1.LSM9DS1()
        else:
            #default
            self.imu = navio.mpu9250.MPU9250()

        if self.imu.testConnection():
            rospy.loginfo("Imu Sensor Connection Established")
        else:
            rospy.logwarn("Imu Sensor Connection Error")

        self.imu.initialize()


if __name__ == "__main__":
    rospy.init_node('accel_gyro_mag')
    accel_gyro_mag_object = AccelGyroMag()

    rate = rospy.Rate(10)

    while not rospy.is_shutdown():
        accel_gyro_mag_object.get_data()
        rate.sleep()
