#! /usr/bin/env python

import yaml
import json
import rospy
from sensor_msgs.msg import Joy
from yqb_car.msg import WheelStatus
from yqb_car.msg import CameraStatus
from yqb_car.msg import ADC
from yqb_car.msg import GPS
from yqb_car.msg import Barometer
from yqb_car.msg import AccelGyroMag
from std_msgs.msg import Empty

class RosWebsocket(object):
    def __init__(self):
        # Subscribe to all ros topics
        self.sub_wheel = rospy.Subscriber('/status/wheels', WheelStatus, self.topic_callback_send_data)
        self.status_wheel = WheelStatus()

        self.sub_camera = rospy.Subscriber('/status/cameras', CameraStatus, self.topic_callback_send_data)
        self.status_camera = CameraStatus()

        # NOT IMPLEMENTED
        # self.sub_led = rospy.Subscriber('/status/led', Empty, self.topic_callback_send_data)
        # self.status_led = Empty()

        self.sub_batteries = rospy.Subscriber('/health/batteries', ADC, self.topic_callback_send_data)
        self.health_batteries = ADC()

        # NOT IMPLEMENTED
        # self.sub_pi = rospy.Subscriber('/health/pi', Empty, self.topic_callback_send_data)
        # self.health_pi = Empty() 

        self.sub_agm = rospy.Subscriber('/sensor/accel_gyro_mag', AccelGyroMag, self.topic_callback_send_data)
        self.sensor_agm = AccelGyroMag()

        self.sub_baro = rospy.Subscriber('/sensor/barometer', Barometer, self.topic_callback_send_data)
        self.sensor_baro = Barometer()

        self.sub_gps = rospy.Subscriber('/sensor/gps', GPS, self.topic_callback_send_data)
        self.sensor_gps = GPS()

        # Publish joystick control
        self.pub_joy = rospy.Publisher('/joy', Joy, queue_size=1)
        self.joy_data = Joy()

    def topic_callback_send_data(self, data):
        # send data over websocket
	    print self.msg2json(data)

    def websocket_receive_data(self):
        # receive control data (preferably joystick controls) over websocket
        joy_data_json = True
        self.send_joy_controls(joy_data_json)

    def msg2json(self, msg):
        # Convert a ROS message to JSON format
        y = yaml.load(str(msg))
        return json.dumps(y,indent=4)

    def send_joy_controls(self, joy_data_json):
        # Publish joy data
        self.pub_joy.publish(self.json2joy(joy_data_json))

    def json2joy(self)   
        #convert json joy data to ros message 
        pass

if __name__ == "__main__":
    rospy.init_node('ros_websocket', log_level=rospy.INFO)
    ros_websocket_object = RosWebsocket()
    rate = rospy.Rate(100)

    while not rospy.is_shutdown():
        ros_websocket_object.websocket_receive_data()
        rate.sleep()

    rospy.spin()
