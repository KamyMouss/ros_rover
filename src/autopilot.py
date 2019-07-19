#! /usr/bin/env python

import yaml
import json
import rospy
from sensor_msgs.msg import Joy
from yqb_car.msg import AutopilotControl
from yqb_car.msg import AutopilotStatus
from yqb_car.msg import WheelStatus
from yqb_car.msg import CameraStatus
from yqb_car.msg import ADC
from yqb_car.msg import GPS
from yqb_car.msg import Barometer
from yqb_car.msg import AccelGyroMag
from yqb_car.msg import CameraControl
from std_msgs.msg import Empty

class AutopilotControl(object):
    def __init__(self):
        # Subscribe to all ros topics
        self.sub_wheels = rospy.Subscriber('/control/autopilot', AutopilotControl, self.autopilot_callback)
        self.control_autopilot = AutopilotControl()

        self.sub_wheels = rospy.Subscriber('/status/wheels', WheelStatus, self.wheels_callback)
        self.status_wheels = WheelStatus()

        self.sub_camera = rospy.Subscriber('/status/cameras', CameraStatus, self.camera_callback)
        self.status_camera = CameraStatus()

        # NOT IMPLEMENTED
        # self.sub_led = rospy.Subscriber('/status/led', Empty, self.led_callback)
        # self.status_led = Empty()

        self.sub_batteries = rospy.Subscriber('/health/batteries', ADC, self.adc_callback)
        self.health_batteries = ADC()

        # NOT IMPLEMENTED
        # self.sub_pi = rospy.Subscriber('/health/pi', Empty, self.pi_callback)
        # self.health_pi = Empty() 

        self.sub_agm = rospy.Subscriber('/sensor/accel_gyro_mag', AccelGyroMag, self.agm_callback)
        self.sensor_agm = AccelGyroMag()

        self.sub_baro = rospy.Subscriber('/sensor/barometer', Barometer, self.baro_callback)
        self.sensor_baro = Barometer()

        self.sub_gps = rospy.Subscriber('/sensor/gps', GPS, self.gps_callback)
        self.sensor_gps = GPS()

        # Publishing 
        self.pub_autopilot = rospy.Publisher('/status/autopilot', AutopilotStatus, queue_size=1)
        self.status_autopilot = AutopilotStatus()

        self.pub_cmd_vel = rospy.Publisher('/control/wheels/cmd_vel', Twist, queue_size=1)
        self.cmd_vel = Twist()

        self.pub_camera_control = rospy.Publisher('/control/cameras', CameraControl, queue_size=1)
        self.camera_control = CameraControl()

    def autopilot_callback(self, data):
        self.control_autopilot = data

    def wheels_callback(self, data):
        self.status_wheels = data

    def camera_callback(self, data):
        self.status_camera = data

    def led_callback(self, data):
        self.status_led = data

    def adc_callback(self, data):
        self.health_batteries = data

    def pi_callback(self, data):
        self.health_pi = data

    def agm_callback(self, data):
        self.sensor_agm = data

    def baro_callback(self, data):
        self.sensor_baro = data

    def gps_callback(self, data):
        self.sensor_gps = data
	
    def run_autopilot(self):
        if self.control_autopilot.command == "ACTIVATED":
            self.status_autopilot.status = "Active mission"
            rospy.loginfo("Autopilot running" + self.status_autopilot.status)
            self.pub_autopilot.publish(self.status_autopilot)
        else if self.status_autopilot.status == "Active mission":
            self.status_autopilot.status = "No Active mission"
            self.pub_autopilot.publish(self.status_autopilot)

if __name__ == "__main__":
    rospy.init_node('autopilot', log_level=rospy.INFO)
    autopilot_object = AutopilotControl()
    rate = rospy.Rate(1)

    while not rospy.is_shutdown():
        autopilot_object.run_autopilot()
        rate.sleep()
