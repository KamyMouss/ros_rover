#! /usr/bin/env python

import rospy
import navio
from sensor_msgs.msg import Joy
from std_msgs.msg import Float32

#Joystick mapping
PAN_AXIS = 3             #Right Joystick horizontal
TILT_AXIS = 4            #Right Joystick vertical
LEFT_CAMERA_SELECT = 3   #Square button
RIGHT_CAMERA_SELECT = 1  #Circle button

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

class CameraControl(object):
    def __init__(self):
        # Define PWM channels
        self.left_pan = navio.pwm.PWM(SERVO_LEFT_PAN)
        self.left_tilt = navio.pwm.PWM(SERVO_LEFT_TILT)
        self.right_pan  = navio.pwm.PWM(SERVO_RIGHT_PAN)
        self.right_tilt  = navio.pwm.PWM(SERVO_RIGHT_TILT)
        self.left_pan_pwm = LEFT_PAN_CENTER
        self.left_tilt_pwm = LEFT_TILT_CENTER
        self.right_pan_pwm = RIGHT_PAN_CENTER
        self.right_tilt_pwm = RIGHT_TILT_CENTER
        self.initialize_pwm()
        self.set_pwm()

        self.selected_camera = "LEFT"

        self.pub_left_pan = rospy.Publisher('/status/camera/left/pan', Float32, queue_size=1)
        self.pub_left_tilt = rospy.Publisher('/status/camera/left/tilt', Float32, queue_size=1)
        self.pub_right_pan = rospy.Publisher('/status/camera/right/pan', Float32, queue_size=1)
        self.pub_right_tilt = rospy.Publisher('/status/camera/right/tilt', Float32, queue_size=1)

        self.joy_sub = rospy.Subscriber('/joy', Joy, self.callback)
        self.joy_axes = [0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0]
        self.joy_buttons = [0, 0, 0, 0, 0, 0, 0, 0]
        
        rospy.loginfo("Left Initial Pan: " + str(self.left_pan_pwm) + " Tilt: " + str(self.left_tilt_pwm))
        rospy.loginfo("Right Initial Pan: " + str(self.right_pan_pwm) + " Tilt: " + str(self.right_tilt_pwm))    

    # Capture cmd_vel 
    def callback(self, data):
        self.joy_axes = data.axes
        self.joy_buttons = data.buttons

    def update_pwm(self):  
        # Camera select switch
        if self.joy_buttons[LEFT_CAMERA_SELECT] == 1 and self.selected_camera != "LEFT":
            self.selected_camera = "LEFT"
            rospy.loginfo("Left Camera Selected.")
            rospy.loginfo("Left Pan: " + str(self.left_pan_pwm) + " Tilt: " + str(self.left_tilt_pwm))
        elif self.joy_buttons[RIGHT_CAMERA_SELECT] == 1 and self.selected_camera != "RIGHT":
            self.selected_camera = "RIGHT"
            rospy.loginfo("Right Camera Selected.")
            rospy.loginfo("Right Pan: " + str(self.right_pan_pwm) + " Tilt: " + str(self.right_tilt_pwm))   

        if abs(self.joy_axes[TILT_AXIS]) != 0.0 or abs(self.joy_axes[PAN_AXIS]) != 0.0:
            # Servo pwm update
            if self.selected_camera == "LEFT":
                # Left Tilt
                if (self.joy_axes[TILT_AXIS] > 0 and self.left_tilt_pwm > LEFT_TILT_MAX) or (self.joy_axes[TILT_AXIS] < 0 and self.left_tilt_pwm < LEFT_TILT_MIN):
                    rospy.logwarn("Left tilt out of range!")
                else:
                    self.left_tilt_pwm += self.joy_axes[TILT_AXIS] * VELOCITY_FACTOR

                # Left Pan
                if (self.joy_axes[PAN_AXIS] >= 0 and self.left_pan_pwm > LEFT_PAN_MAX) or (self.joy_axes[PAN_AXIS] < 0 and self.left_pan_pwm < LEFT_PAN_MIN):
                    rospy.logwarn("Left pan out of range!")
                else:
                    self.left_pan_pwm += self.joy_axes[PAN_AXIS] * VELOCITY_FACTOR

                rospy.loginfo("Left Pan: " + str(self.left_pan_pwm) + " Tilt: " + str(self.left_tilt_pwm))
            
            elif self.selected_camera == "RIGHT":
                # Right Tilt
                if (self.joy_axes[TILT_AXIS] > 0 and self.right_tilt_pwm > RIGHT_TILT_MAX) or (self.joy_axes[TILT_AXIS] < 0 and self.right_tilt_pwm < RIGHT_TILT_MIN):
                    rospy.logwarn("Right tilt out of range!")
                else: 
                    self.right_tilt_pwm += self.joy_axes[TILT_AXIS] * VELOCITY_FACTOR
                
                # Right Pan
                if (self.joy_axes[PAN_AXIS] > 0 and self.right_pan_pwm > RIGHT_PAN_MAX) or (self.joy_axes[PAN_AXIS] < 0 and self.right_pan_pwm < RIGHT_PAN_MIN):
                    rospy.logwarn("Right pan out of range!")
                else:
                    self.right_pan_pwm += self.joy_axes[PAN_AXIS] * VELOCITY_FACTOR

                rospy.loginfo("Right Pan: " + str(self.right_pan_pwm) + " Tilt: " + str(self.right_tilt_pwm))    

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
        self.left_pan.set_duty_cycle(self.left_pan_pwm)
        self.left_tilt.set_duty_cycle(self.left_tilt_pwm)
        self.right_pan.set_duty_cycle(self.right_pan_pwm)
        self.right_tilt.set_duty_cycle(self.right_tilt_pwm)

if __name__ == "__main__":
    rospy.init_node('camera_control', log_level=rospy.INFO)
    camera_control_object = CameraControl()
    rate = rospy.Rate(100)
    
    while not rospy.is_shutdown():
        camera_control_object.update_pwm()
        rate.sleep()