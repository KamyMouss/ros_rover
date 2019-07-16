#! /usr/bin/env python

import yaml
import json
import rospy
from sensor_msgs.msg import Joy
from yqb_car.msg import MotorStatus
from yqb_car.msg import CameraStatus
from yqb_car.msg import ADC
from yqb_car.msg import GPS
from yqb_car.msg import Barometer
from yqb_car.msg import AccelGyroMagMsg
from std_msgs.msg import Empty

class RosWebsocket(object):
    def __init__(self):
        # Subscribe to ros topics
        self.sub_motor = rospy.Subscriber('/status/wheels', MotorStatus, self.callback)
        self.status_motor = MotorStatus()

        self.sub_camera = rospy.Subscriber('/status/camera', CameraStatus, self.callback)
        self.status_camera = CameraStatus()

        # NOT IMPLEMENTED
        # self.sub_led = rospy.Subscriber('/status/led', Empty, self.callback)
        # self.status_led = Empty()

        self.sub_batteries = rospy.Subscriber('/health/batteries', ADC, self.callback)
        self.haelth_batteries = ADC()

        # NOT IMPLEMENTED
        # self.sub_pi = rospy.Subscriber('/health/pi', Empty, self.callback)
        # self.health_pi = Empty() 

        self.sub_acm = rospy.Subscriber('/sensor/accel_gyro_mag', AccelGyroMagMsg, self.callback)
        self.sensor_acm = AccelGyroMagMsg()

        self.sub_baro = rospy.Subscriber('/sensor/barometer', Barometer, self.callback)
        self.sensor_baro = Barometer()

        self.sub_gps = rospy.Subscriber('/sensor/gps', GPS, self.callback)
        self.sensor_gps = GPS()


        # Publish joystick control
        self.pub_cmd_vel = rospy.Publisher('/joy', Joy, queue_size=1)
        self.joy_data = Joy()

    def callback(self, data):
        # send data over websocket
	    # print self.msg2json(data)

    def msg2json(self, msg):
        # Convert a ROS message to JSON format
        y = yaml.load(str(msg))
        return json.dumps(y,indent=4)
        

if __name__ == "__main__":
    rospy.init_node('ros_websocket', log_level=rospy.INFO)
    ros_websocket_object = RosWebsocket()
    rospy.spin()
