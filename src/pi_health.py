#! /usr/bin/env python

import rospy
import psutil
from subprocess import PIPE, Popen
from std_msgs.msg import Empty

class PiHealthReader(object):
    def __init__(self):
        self.pub = rospy.Publisher('/health/pi', Empty, queue_size=1)

    def get_cpu_temperature(self):
        process = Popen(['vcgencmd', 'measure_temp'], stdout=PIPE)
        output, _error = process.communicate()
        return float(output[output.index('=') + 1:output.rindex("'")])

    def get_health(self):
        print psutil.cpu_percent(interval=1)
        print psutil.virtual_memory()[2]
        print psutil.disk_usage('/')[3]
	print self.get_cpu_temperature()

if __name__ == "__main__":
    rospy.init_node('pi_health')
    pi_health_reader = PiHealthReader()
    rate = rospy.Rate(1)

    while not rospy.is_shutdown():
        pi_health_reader.get_health()
        rate.sleep()
