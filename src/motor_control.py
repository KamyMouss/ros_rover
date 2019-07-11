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
        self.right_pwm = navio.pwm.PWM(M_RIGHT_PWM)
        self.right_dir = navio.pwm.PWM(M_RIGHT_DIR)
        self.left_pwm  = navio.pwm.PWM(M_LEFT_PWM)
        self.left_dir  = navio.pwm.PWM(M_LEFT_DIR)
        self.initialize_pwm()

        # Subscribing
        self.sub_right = rospy.Subscriber('/control/motor/cmd_vel', Twist, self.callback)
        
        # Publishing
        self.pub_pwm = rospy.Publisher('/status/motor/pwm', Twist, queue_size=1)

        self.cmd_data = Twist()
        
        
        self.v_right = 0
        self.v_left = 0
        self.pwm_right = 0
        self.pwm_left = 0
        self.dir_right = "FORWARDS"
        self.dir_left = "FORWARDS"
        
    # Capture cmd_vel 
    def callback(self, data):
        self.cmd_data = data
        self.cmd_vel_to_diff(self.cmd_data)
        #rospy.loginfo(self.cmd_data)
    
    #transform to differential drive velocities (right and left wheel velocities)
    def cmd_vel_to_diff(self, cmd_data):
        linear_v = cmd_data.linear.x
        angular_w = cmd_data.angular.z

        self.v_right = ((2*linear_v) + angular_w * WHEEL_BASE) / (2*WHEEL_RADIUS)
        self.v_left = ((2*linear_v) - angular_w * WHEEL_BASE) / (2*WHEEL_RADIUS)
        #rospy.loginfo("DIFF SPEED (Left, Right): " + str(self.v_left) + ", " + str(self.v_right))
        
        if self.v_left >= 0:
            self.dir_left = "FORWARDS"
            #self.left_pwm.enable()
            self.pwm_left = abs(self.v_left / VEL_TO_PWM_FACTOR)
        else:
            self.dir_left= "BACKWARDS" 
            #self.left_pwm.disable()
            #Flipping left and right when reverse
            self.pwm_left = abs(self.v_right / VEL_TO_PWM_FACTOR) 
 
        if self.v_right >= 0:
            self.dir_right = "FORWARDS"
            #self.left_pwm.enable()
            self.pwm_right = abs(self.v_right / VEL_TO_PWM_FACTOR)
        else:
            self.dir_right = "BACKWARDS" 
            #self.left_pwm.disable()
            #Flipping left and right when reverse 
            self.pwm_right = abs(self.v_left / VEL_TO_PWM_FACTOR)

        self.turnWheels()

    def turnWheels(self):
        rospy.loginfo("DIR (Left, Right): " + str(self.dir_left) + ", " + str(self.dir_right))
        rospy.loginfo("PWM (Left, Right): " + str(self.pwm_left) + ", " + str(self.pwm_right))
        
        self.right_pwm.set_duty_cycle(self.pwm_right)
        self.left_pwm.set_duty_cycle(self.pwm_left)


    def initialize_pwm(self):
        self.right_pwm.initialize()
        self.right_pwm.set_period(50)
        self.right_pwm.enable()

        self.right_dir.initialize()
        self.right_dir.set_period(50)
        self.right_dir.enable()

        self.left_pwm.initialize()
        self.left_pwm.set_period(50)
        self.left_pwm.enable()

        self.left_dir.initialize()
        self.left_dir.set_period(50)
        self.left_dir.enable()

        return True


if __name__ == "__main__":
    rospy.init_node('motor_control', log_level=rospy.INFO)
    motor_control_object = MotorControl()
    rate = rospy.Rate(1)
    rospy.spin()