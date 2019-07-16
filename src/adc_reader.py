#! /usr/bin/env python

import navio.adc
import navio.util
import rospy
from std_msgs.msg import Float32MultiArray
from yqb_car.msg import ADC


class AdcReader(object):
    def __init__(self):
        self.pub = rospy.Publisher('/health/batteries', ADC, queue_size=1)
        self.adc = navio.adc.ADC()
        self.adc_voltages = ADC()
        self.adc_voltages.A0 = 0
        self.adc_voltages.A1 = 0

    def get_voltage(self):
        self.adc_voltages.A0 = self.adc.read(0)
        self.adc_voltages.A1 = self.adc.read(1)    

        self.pub.publish(self.adc_voltages)

if __name__ == "__main__":
    rospy.init_node('adc_reader')
    adc_reader = AdcReader()

    rate = rospy.Rate(1)

    while not rospy.is_shutdown():
        adc_reader.get_voltage()
        rate.sleep()
