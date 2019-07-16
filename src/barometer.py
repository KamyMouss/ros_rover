#! /usr/bin/env python

import navio.adc
import navio.ms5611
import navio.util

import rospy
from yqb_car.msg import Barometer


class BarometerReader(object):
    def __init__(self):
        self.pub = rospy.Publisher('/sensor/barometer', Barometer, queue_size=1)
        self.baro = navio.ms5611.MS5611()
        self.baro.initialize()   
        rospy.loginfo("Barometer initialized.")
        self.baro_data = Barometer()

    def get_data(self):
        self.baro.refreshPressure()
        self.baro.readPressure()

        self.baro.refreshTemperature()
        self.baro.readTemperature()

        self.baro.calculatePressureAndTemperature()

        print "Temperature(C): %.6f" % (self.baro.TEMP), "Pressure(millibar): %.6f" % (self.baro.PRES)
        
        self.baro_data.temperature = self.baro.TEMP
        self.baro_data.pressure = self.baro.PRES
        self.pub.publish(self.baro_data)

if __name__ == "__main__":
    rospy.init_node('barometer')
    barometer_reader = BarometerReader()

    rate = rospy.Rate(1)

    while not rospy.is_shutdown():
        barometer_reader.get_data()
        rate.sleep()