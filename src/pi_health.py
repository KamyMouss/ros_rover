#! /usr/bin/env python

import rospy
import psutil
from std_msgs.msg import Empty

class PiHealthReader(object):
    def __init__(self):
        self.pub = rospy.Publisher('/health/pi', Empty, queue_size=1)

    def get_health(self):
        print psutil.cpu_percent(interval=1)
        print psutil.virtual_memory()
        print psutil.disk_usage('/')
        print psutil.sensors_temperatures()

if __name__ == "__main__":
    rospy.init_node('pi_health')
    pi_health_reader = PiHealthReader()
    rate = rospy.Rate(1)

    while not rospy.is_shutdown():
        pi_health_reader.get_health()
        rate.sleep()
