#! /usr/bin/env python

import rospy
from ../lib import navio
from geometry_msgs.msg import Twist
from yqb_car.msg import MotorStatus

# Change this for higher pwm range, default 20 (0 - 2500)
VEL_TO_PWM_FACTOR = rospy.get_param("/motor_control_driver/max_pwm") / 0.125   

#NAVIO pwn out channels
RIGHT_PWM_CH = rospy.get_param("/motor_control_driver/right_pwm_out")
RIGHT_DIR_CH = rospy.get_param("/motor_control_driver/right_dir_out")
LEFT_PWM_CH = rospy.get_param("/motor_control_driver/left_pwm_out")
LEFT_DIR_CH = rospy.get_param("/motor_control_driver/left_dir_out")

#Differential Drive Parameters
WHEEL_BASE = rospy.get_param("/motor_control_driver/wheel_base")
WHEEL_RADIUS = rospy.get_param("/motor_control_driver/wheel_radius")

class MotorControlDriver(object):
    def __init__(self):
        # Define PWM channels
        self.right_pwm_ch = navio.pwm.PWM(RIGHT_PWM_CH)
        self.right_dir_ch = navio.pwm.PWM(RIGHT_DIR_CH)
        self.left_pwm_ch  = navio.pwm.PWM(LEFT_PWM_CH)
        self.left_dir_ch  = navio.pwm.PWM(LEFT_DIR_CH)
        self.initialize_pwm()

        # Subscribing
        self.sub_right = rospy.Subscriber('/control/motor/cmd_vel', Twist, self.callback)
        
        # Publishing
        self.pub_status = rospy.Publisher('/status/motor', MotorStatus, queue_size=1)

        # Creating messages
        self.motor_status = MotorStatus()
        self.cmd_data = Twist()
        
        # Setting initial speed and pwm to 0
        self.v_right = 0
        self.v_left = 0
        self.right_pwm = 0
        self.left_pwm = 0

        # Setting initial direction
        self.left_reverse = False
        self.right_reverse = False
        
    # Capture cmd_vel 
    def callback(self, data):
        self.cmd_data = data
        self.cmd_vel_to_pwm(self.cmd_data)
        self.turn_wheels()
        self.publish_status()
        #rospy.loginfo(self.cmd_data)
    
    #transform to differential drive velocities (right and left wheel velocities)
    def cmd_vel_to_pwm(self, cmd_data):
        linear_v = cmd_data.linear.x
        angular_w = cmd_data.angular.z
        
        # Calculate differential speed
        self.v_right = ((2*linear_v) + angular_w * WHEEL_BASE) / (2*WHEEL_RADIUS)
        self.v_left = ((2*linear_v) - angular_w * WHEEL_BASE) / (2*WHEEL_RADIUS)
        #rospy.loginfo("DIFF SPEED (Left, Right): " + str(self.v_left) + ", " + str(self.v_right))
        
        # Transform to pwm values (in ms)
        self.left_pwm = abs(self.v_left / VEL_TO_PWM_FACTOR)
        self.right_pwm = abs(self.v_right / VEL_TO_PWM_FACTOR)
        
        # Determine if wheel direction is reversed
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
        #rospy.loginfo("DIR (Left, Right): " + str(self.right_reverse) + ", " + str(self.left_reverse))
        #rospy.loginfo("PWM (Left, Right): " + str(self.left_pwm) + ", " + str(self.right_pwm))
        
        self.right_pwm_ch.set_duty_cycle(self.right_pwm)
        self.left_pwm_ch.set_duty_cycle(self.left_pwm)

    def publish_status(self):
        self.motor_status.left_pwm = self.left_pwm
        self.motor_status.right_pwm = self.right_pwm
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
    rospy.init_node('motor_control_driver', log_level=rospy.INFO)
    motor_control_object = MotorControlDriver()
    rate = rospy.Rate(1)
    rospy.spin()
