#! /usr/bin/env python

import navio.adc
import navio.mpu9250
import navio.util

import rospy
from std_msgs.msg import Float32MultiArray
from geometry_msgs.msg import Twist
from yqb_car.msg import AccelGyroMag


class AccelGyroMagReader(object):
    def __init__(self):
        # Publishing
        self.pub = rospy.Publisher('/sensor/accel_gyro_mag', AccelGyroMag, queue_size=1)
        
        self.imu_sensor = rospy.get_param("/accel_gyro_mag/imu_sensor")
        self.imu = None
        self.initialize()       
        self.imu_data = AccelGyroMag()

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

        # print "Acc:", "{:+7.3f}".format(m9a[0]), "{:+7.3f}".format(m9a[1]), "{:+7.3f}".format(m9a[2]),
        # print " Gyr:", "{:+8.3f}".format(m9g[0]), "{:+8.3f}".format(m9g[1]), "{:+8.3f}".format(m9g[2]),
        # print " Mag:", "{:+7.3f}".format(m9m[0]), "{:+7.3f}".format(m9m[1]), "{:+7.3f}".format(m9m[2])
        
        self.imu_data.accel[0] = m9a[0]
        self.imu_data.accel[1] = m9a[1]
        self.imu_data.accel[2] = m9a[2]

        self.imu_data.gyro[0] = m9g[0]
        self.imu_data.gyro[1] = m9g[1]
        self.imu_data.gyro[2] = m9g[2]
        
        self.imu_data.mag[0] = m9m[0]
        self.imu_data.mag[1] = m9m[1]
        self.imu_data.mag[2] = m9m[2]

        self.pub.publish(self.imu_data)

    def initialize(self):
        if self.imu_sensor == 'mpu':
            self.imu = navio.mpu9250.MPU9250()
        elif self.imu_sensor == 'lsm':
            self.imu = navio.lsm9ds1.LSM9DS1()
        else:
            #default
            self.imu = navio.mpu9250.MPU9250()

        if self.imu.testConnection():
            rospy.loginfo("Imu Sensor Connection Established: " + self.imu_sensor)
        else:
            rospy.logwarn("Imu Sensor Connection Error")

        self.imu.initialize()


if __name__ == "__main__":
    rospy.init_node('accel_gyro_mag')
    accel_gyro_mag_object = AccelGyroMagReader()

    rate = rospy.Rate(10)

    while not rospy.is_shutdown():
        accel_gyro_mag_object.get_data()
        rate.sleep()
