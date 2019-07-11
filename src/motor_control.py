#! /usr/bin/env python

import rospy
import navio
from geometry_msgs.msg import Twist
from yqb_car.msg import MotorStatus

# Change this for higher pwm range, default 20 (0 - 2500)
VEL_TO_PWM_FACTOR = 20   

#NAVIO pwn out channels
M_RIGHT_PWM = 1
M_RIGHT_DIR = 2
M_LEFT_PWM = 3
M_LEFT_DIR = 4

#Differential Drive Parameters
WHEEL_BASE = 0.10    #in meters per radian
WHEEL_RADIUS = 0.02  #in meters per radian

class MotorControl(object):
    def __init__(self):
        # Define PWM channels
        self.right_pwm_ch = navio.pwm.PWM(M_RIGHT_PWM)
        self.right_dir_ch = navio.pwm.PWM(M_RIGHT_DIR)
        self.left_pwm_ch  = navio.pwm.PWM(M_LEFT_PWM)
        self.left_dir_ch  = navio.pwm.PWM(M_LEFT_DIR)
        self.initialize_pwm()

        # Subscribing
        self.sub_right = rospy.Subscriber('/control/motor/cmd_vel', Twist, self.callback)
        
        # Publishing
        self.pub_status = rospy.Publisher('/status/motor', MotorStatus, queue_size=1)

        self.motor_status = MotorStatus()
        self.cmd_data = Twist()
        
        
        self.v_right = 0
        self.v_left = 0
        self.right_pwm = 0
        self.left_pwm = 0
        self.left_reverse = False
        self.right_reverse = False
        
    # Capture cmd_vel 
    def callback(self, data):
        self.cmd_data = data
        self.cmd_vel_to_diff(self.cmd_data)
        self.turn_wheels()
        self.publish_status()
        #rospy.loginfo(self.cmd_data)
    
    #transform to differential drive velocities (right and left wheel velocities)
    def cmd_vel_to_diff(self, cmd_data):
        linear_v = cmd_data.linear.x
        angular_w = cmd_data.angular.z

        self.v_right = ((2*linear_v) + angular_w * WHEEL_BASE) / (2*WHEEL_RADIUS)
        self.v_left = ((2*linear_v) - angular_w * WHEEL_BASE) / (2*WHEEL_RADIUS)
        #rospy.loginfo("DIFF SPEED (Left, Right): " + str(self.v_left) + ", " + str(self.v_right))
        
        if self.v_left >= 0:
            self.right_reverse = False
            #self.left_pwm.enable()
            self.left_pwm = abs(self.v_left / VEL_TO_PWM_FACTOR)
        else:
            self.right_reverse= True
            #self.left_pwm.disable()
            #Flipping left and right when reverse
            self.left_pwm = abs(self.v_right / VEL_TO_PWM_FACTOR) 
 
        if self.v_right >= 0:
            self.left_reverse = False
            #self.left_pwm.enable()
            self.right_pwm = abs(self.v_right / VEL_TO_PWM_FACTOR)
        else:
            self.left_reverse = True
            #self.left_pwm.disable()
            #Flipping left and right when reverse 
            self.right_pwm = abs(self.v_left / VEL_TO_PWM_FACTOR)

    def turn_wheels(self):
        #rospy.loginfo("DIR (Left, Right): " + str(self.right_reverse) + ", " + str(self.left_reverse))
        #rospy.loginfo("PWM (Left, Right): " + str(self.left_pwm) + ", " + str(self.right_pwm))
        
        self.right_pwm_ch.set_duty_cycle(self.right_pwm)
        self.left_pwm_ch.set_duty_cycle(self.left_pwm)

    def publish_status(self):
        self.motor_status.left_pwm = self.left_pwm
        self.motor_status.right_pwm = self.left_pwm
        self.motor_status.left_reverse = self.left_reverse
        self.motor_status.right_reverse = self.right_reverse
        
        self.pub_status.publish(self.motor_status)

    def initialize_pwm(self):
        self.right_pwm_ch.initialize()
        self.right_pwm_ch.set_period(50)
        self.right_pwm_ch.enable()

        self.right_dir_ch.initialize()
        self.right_dir_ch.set_period(50)
        self.right_dir_ch.enable()

        self.left_pwm_ch.initialize()
        self.left_pwm_ch.set_period(50)
        self.left_pwm_ch.enable()

        self.left_dir_ch.initialize()
        self.left_dir_ch.set_period(50)
        self.left_dir_ch.enable()

        return True


if __name__ == "__main__":
    rospy.init_node('motor_control', log_level=rospy.INFO)
    motor_control_object = MotorControl()
    rate = rospy.Rate(1)
    rospy.spin()
