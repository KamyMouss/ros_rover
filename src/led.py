#! /usr/bin/env python

import rospy
from std_msgs.msg import Empty

class LedControl(object):
    def __init__(self):
        self.pub = rospy.Publisher('/status/led', Empty, queue_size=1)
        self.sub = rospy.Subscriber('/control/led', Empty, self.callback)

    def callback(self):
        pass

    def get_status(self):
        pass 

if __name__ == "__main__":
    rospy.init_node('led')
    led_control = LedControl()
    rate = rospy.Rate(1)

    while not rospy.is_shutdown():
        led_control.get_status()
        rate.sleep()

