#! /usr/bin/env python

import rospy
import psutil
from subprocess import PIPE, Popen
from yqb_car.msg import PiHealth

class PiHealthReader(object):
    def __init__(self):
        self.pub = rospy.Publisher('/health/pi', PiHealth, queue_size=1)
        self.pi_health = PiHealth()

    def get_cpu_temperature(self):
        process = Popen(['vcgencmd', 'measure_temp'], stdout=PIPE)
        output, _error = process.communicate()
        return float(output[output.index('=') + 1:output.rindex("'")])

    def get_health(self):
        self.pi_health.cpu_usage = psutil.cpu_percent(interval=1)
	    self.pi_health.cpu_temp = self.get_cpu_temperature()
        self.pi_health.mem_usage = psutil.virtual_memory()[2]
        self.pi_health.disk_usage =psutil.disk_usage('/')[3]

        self.pub.publish(self.pi_health)

if __name__ == "__main__":
    rospy.init_node('pi_health')
    pi_health_reader = PiHealthReader()
    rate = rospy.Rate(2)

    while not rospy.is_shutdown():
        pi_health_reader.get_health()
        rate.sleep()
