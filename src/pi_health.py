#! /usr/bin/env python

import rospy

class PiHealthReader(object):
    def __init__(self):
        self.pub = rospy.Publisher('/health/pi', ADC, queue_size=1)

    def get_health(self):
        pass 

if __name__ == "__main__":
    rospy.init_node('pi_health')
    pi_health_reader = PiHealthReader()
    rate = rospy.Rate(1)

    while not rospy.is_shutdown():
        pi_health_reader.get_health()
        rate.sleep()
