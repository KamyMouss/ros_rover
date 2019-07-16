#! /usr/bin/env python

import navio.adc
import navio.util
import rospy
from std_msgs.msg import Float32MultiArray
from yqb_car.msg import ADC

# ADC Thresholds (V)
A0_MIN = rospy.get_param("/adc_reader/A0_min")
A0_MAX = rospy.get_param("/adc_reader/A0_max")
A1_MIN = rospy.get_param("/adc_reader/A1_min")
A1_MAX = rospy.get_param("/adc_reader/A1_max")


class AdcReader(object):
    def __init__(self):
        self.pub = rospy.Publisher('/health/batteries', ADC, queue_size=1)
        self.adc = navio.adc.ADC()
        self.adc_voltages = ADC()
        self.adc_voltages.A0 = 0
        self.adc_voltages.A0_status = "OK"
        self.adc_voltages.A1 = 0
        self.adc_voltages.A1_status = "OK"

    def get_voltage(self):
        self.adc_voltages.A0 = self.adc.read(0)
        if self.adc_voltages.A0 < A0_MIN:
            self.adc_voltages.A0_status = "UNDER"
            rospy.logwarn("A0 is undervolted!")
        elif self.adc_voltages.A0 > A0_MAX:
            self.adc_voltages.A0_status = "OVER"
            rospy.logwarn("A0 is overvolted!")

        self.adc_voltages.A1 = self.adc.read(1)
        if self.adc_voltages.A1 < A1_MIN:
            self.adc_voltages.A1_status = "UNDER"
            rospy.logwarn("A1 is undervolted!")    
        elif self.adc_voltages.A1 > A1_MAX:
            self.adc_voltages.A1_status = "OVER"
            rospy.logwarn("A1 is overvolted!")

        self.pub.publish(self.adc_voltages)

if __name__ == "__main__":
    rospy.init_node('adc_reader')
    adc_reader = AdcReader()

    rate = rospy.Rate(1)

    while not rospy.is_shutdown():
        adc_reader.get_voltage()
        rate.sleep()
