#! /usr/bin/env python

import rospy
import navio
import os
from geometry_msgs.msg import Twist
from yqb_car.msg import WheelStatus

# Change this for higher pwm range, default 20 (0 - 2500)
MAX_PWM = rospy.get_param("/wheel_control_driver/max_pwm")   

#NAVIO pwn out channels
RIGHT_PWM_CH = rospy.get_param("/wheel_control_driver/right_pwm_out")
RIGHT_DIR_CH = rospy.get_param("/wheel_control_driver/right_dir_out")
LEFT_PWM_CH = rospy.get_param("/wheel_control_driver/left_pwm_out")
LEFT_DIR_CH = rospy.get_param("/wheel_control_driver/left_dir_out")

#Differential Drive Parameters
WHEEL_BASE = rospy.get_param("/wheel_control_driver/wheel_base")
WHEEL_RADIUS = rospy.get_param("/wheel_control_driver/wheel_radius")

class WheelControlDriver(object):
    def __init__(self):
        # Define PWM channels
        self.right_pwm_ch = navio.pwm.PWM(RIGHT_PWM_CH)
        self.right_dir_ch = navio.pwm.PWM(RIGHT_DIR_CH)
        self.left_pwm_ch  = navio.pwm.PWM(LEFT_PWM_CH)
        self.left_dir_ch  = navio.pwm.PWM(LEFT_DIR_CH)
        self.initialize_pwm()

        # Subscribing
        self.sub_right = rospy.Subscriber('/control/wheels', Twist, self.callback)
        
        # Publishing
        self.pub_status = rospy.Publisher('/status/wheels', WheelStatus, queue_size=1)

        # Creating messages
        self.wheel_status = WheelStatus()
        self.cmd_data = Twist()
        
        # Setting initial speed and pwm to 0
        self.v_right = 0
        self.v_left = 0
        self.right_pwm = 0
        self.left_pwm = 0

        # Setting initial direction
        self.left_reverse = False
        self.right_reverse = False
        self.left_is_reverse = False         
        self.right_is_reverse = False

    def callback(self, data):
        self.cmd_data = data
        self.cmd_vel_to_pwm(self.cmd_data)
        self.turn_wheels()
        self.publish_status()
        #rospy.loginfo(self.cmd_data)
    
    def cmd_vel_to_pwm(self, cmd_data):
        linear_v = cmd_data.linear.x
        angular_w = cmd_data.angular.z
        
        # Calculate differential speed
        self.v_right = (((2*linear_v) + angular_w * WHEEL_BASE) / (2*WHEEL_RADIUS)) / 50
        self.v_left = (((2*linear_v) - angular_w * WHEEL_BASE) / (2*WHEEL_RADIUS)) / 50
        #rospy.loginfo("DIFF SPEED (Left, Right): " + str(self.v_left) + ", " + str(self.v_right))
        
        # Transform to pwm values (in ms)
        self.left_pwm = abs(self.v_left * MAX_PWM)
        self.right_pwm = abs(self.v_right * MAX_PWM)
        
        # Determine if wheel direction needs to be reversed
        if self.v_left >= 0: 
            self.left_reverse = False
        else: 
            self.left_reverse = True
 
        if self.v_right >= 0: 
            self.right_reverse = False
        else: 
            self.right_reverse = True

        # Flip wheel pwm if both wheels are reversed (for realistic driving)
        if self.right_reverse == True and self.left_reverse == True:
            temp = self.left_pwm
            self.left_pwm = self.right_pwm
            self.right_pwm = temp

    def turn_wheels(self):
        # Reverse left wheel
        if self.left_reverse and not self.left_is_reverse:
            self.left_pwm_ch.enable()
            self.left_is_reverse = True
        elif not self.left_reverse and self.left_is_reverse:
            self.left_pwm_ch.disable()
            self.left_is_reverse = False
        
        # Reverse right wheel
        if self.right_reverse and not self.right_is_reverse:
            self.right_pwm_ch.enable()
            self.right_is_reverse = True
        elif not self.right_reverse and self.right_is_reverse:
            self.right_pwm_ch.disable()
            self.right_is_reverse = False

        # Set servo pwm
        self.right_pwm_ch.set_duty_cycle(self.right_pwm)
        self.left_pwm_ch.set_duty_cycle(self.left_pwm)

    def publish_status(self):
        self.wheel_status.left_pwm = self.left_pwm
        self.wheel_status.right_pwm = self.right_pwm
        self.wheel_status.left_reverse = self.left_reverse
        self.wheel_status.right_reverse = self.right_reverse
        
        self.pub_status.publish(self.wheel_status)

    def initialize_pwm(self):
        # Initialize left channels
        self.left_pwm_ch.initialize()
        self.left_pwm_ch.set_period(50)
        self.left_pwm_ch.enable()
        
        # Left channel dir disabled by default
        self.left_dir_ch.initialize()
        self.left_dir_ch.set_period(50)
        
        # Initialize right channels
        self.right_pwm_ch.initialize()
        self.right_pwm_ch.set_period(50)
        self.right_pwm_ch.enable()

        # Right channel dir disabled by default
        self.right_dir_ch.initialize()
        self.right_dir_ch.set_period(50)

        rospy.loginfo("Wheel Motors Initialized.")

if __name__ == "__main__":
    rospy.init_node('wheel_control_driver', log_level=rospy.INFO)
    wheel_control_object = WheelControlDriver()
    rate = rospy.Rate(1)
    rospy.spin()
