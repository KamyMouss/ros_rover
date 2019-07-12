#! /usr/bin/env python

import rospy
import navio
from sensor_msgs.msg import Joy
from yqb_car.msg import CameraControl
from yqb_car.msg import CameraStatus

#NAVIO pwn out channels
SERVO_LEFT_PAN = 5
SERVO_LEFT_TILT = 6
SERVO_RIGHT_PAN = 7
SERVO_RIGHT_TILT = 8

#Camera servo pwm ranges (in ms)
LEFT_PAN_MIN = 0.600 
LEFT_PAN_CENTER = 2.025
LEFT_PAN_MAX = 4.000

LEFT_TILT_MIN = 0.650
LEFT_TILT_CENTER = 2.000
LEFT_TILT_MAX = 2.500

RIGHT_PAN_MIN = 0.400
RIGHT_PAN_CENTER = 0.950
RIGHT_PAN_MAX = 3.000

RIGHT_TILT_MIN = .800
RIGHT_TILT_CENTER = 2.000
RIGHT_TILT_MAX = 2.500

BUFFER_RANGE = .01

#Servo turn speed (default 0.001)
VELOCITY_FACTOR = 0.001

class CameraControlDriver(object):
    def __init__(self):
        # Define PWM channels on navio2 boards
        self.left_pan = navio.pwm.PWM(SERVO_LEFT_PAN)
        self.left_tilt = navio.pwm.PWM(SERVO_LEFT_TILT)
        self.right_pan  = navio.pwm.PWM(SERVO_RIGHT_PAN)
        self.right_tilt  = navio.pwm.PWM(SERVO_RIGHT_TILT)
        self.initialize_pwm()

        # Subscribe to camera direction
        self.sub_cam_ctrl = rospy.Subscriber('/control/camera', CameraControl, self.callback)
        self.cam_ctrl_dir = CameraControl()
        
        # Publish camera status
        self.pub_cam_status = rospy.Publisher('/status/camera', CameraStatus , queue_size=1)
        self.cam_status = CameraStatus()
        self.cam_status.left_pan_pwm = LEFT_PAN_CENTER
        self.cam_status.left_tilt_pwm = LEFT_TILT_CENTER
        self.cam_status.right_pan_pwm = RIGHT_PAN_CENTER
        self.cam_status.right_tilt_pwm = RIGHT_TILT_CENTER
        self.set_pwm()

        rospy.loginfo("Left Initial Pan: " + str(self.cam_status.left_pan_pwm) + " Tilt: " + str(self.cam_status.left_tilt_pwm))
        rospy.loginfo("Right Initial Pan: " + str(self.cam_status.right_pan_pwm) + " Tilt: " + str(self.cam_status.right_tilt_pwm))    

    # Capture cmd_vel 
    def callback(self, data):
        self.cam_ctrl_dir = data


    def update_pwm(self):  
        self.validate_range()
        self.cam_status.left_pan_pwm += self.cam_ctrl_dir.left_pan_dir * VELOCITY_FACTOR
        self.cam_status.left_tilt_pwm += self.cam_ctrl_dir.left_tilt_dir * VELOCITY_FACTOR
        self.cam_status.right_pan_pwm += self.cam_ctrl_dir.right_pan_dir * VELOCITY_FACTOR
        self.cam_status.right_tilt_pwm += self.cam_ctrl_dir.right_tilt_dir * VELOCITY_FACTOR
        self.set_pwm()

    def initialize_pwm(self):
        self.left_pan.initialize()
        self.left_pan.set_period(50)
        self.left_pan.enable()

        self.left_tilt.initialize()
        self.left_tilt.set_period(50)
        self.left_tilt.enable()

        self.right_pan.initialize()
        self.right_pan.set_period(50)
        self.right_pan.enable()

        self.right_tilt.initialize()
        self.right_tilt.set_period(50)
        self.right_tilt.enable()

    def set_pwm(self):
        self.left_pan.set_duty_cycle(self.cam_status.left_pan_pwm)
        self.left_tilt.set_duty_cycle(self.cam_status.left_tilt_pwm)
        self.right_pan.set_duty_cycle(self.cam_status.right_pan_pwm)
        self.right_tilt.set_duty_cycle(self.cam_status.right_tilt_pwm)

        self.pub_cam_status.publish(self.cam_status)

    def validate_range(self):
        # Left Pan
        if (self.cam_ctrl_dir.left_pan_dir >= 0 and self.cam_status.left_pan_pwm > LEFT_PAN_MAX) or (self.cam_ctrl_dir.left_pan_dir < 0 and self.cam_status.left_pan_pwm < LEFT_PAN_MIN):
            rospy.logwarn("Left pan out of range!")
            self.cam_ctrl_dir.left_pan_dir = 0.0

        # Left Tilt
        if (self.cam_ctrl_dir.left_tilt_dir > 0 and self.cam_status.left_tilt_pwm > LEFT_TILT_MAX) or (self.cam_ctrl_dir.left_tilt_dir < 0 and self.cam_status.left_tilt_pwm < LEFT_TILT_MIN):
            rospy.logwarn("Left tilt out of range!")
            self.cam_ctrl_dir.left_pan_dir = 0.0
    
        # Right Pan
        if (self.cam_ctrl_dir.right_pan_dir > 0 and self.cam_status.right_pan_pwm > RIGHT_PAN_MAX) or (self.cam_ctrl_dir.right_pan_dir < 0 and self.cam_status.right_pan_pwm < RIGHT_PAN_MIN):
            rospy.logwarn("Right pan out of range!")
            self.cam_ctrl_dir.right_pan_dir = 0

        # Right Tilt
        if (self.cam_ctrl_dir.right_tilt_dir > 0 and self.cam_status.right_tilt_pwm > RIGHT_TILT_MAX) or (self.cam_ctrl_dir.right_tilt_dir < 0 and self.cam_status.right_tilt_pwm < RIGHT_TILT_MIN):
            rospy.logwarn("Right tilt out of range!")
            self.cam_ctrl_dir.right_tilt_dir = 0 

if __name__ == "__main__":
    rospy.init_node('camera_control_driver', log_level=rospy.INFO)
    camera_control_object = CameraControlDriver()
    rate = rospy.Rate(100)
    
    while not rospy.is_shutdown():
        camera_control_object.update_pwm()
        rate.sleep()