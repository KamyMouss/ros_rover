#! /usr/bin/env python

import rospy
import navio
from yqb_car.msg import CameraControl
from yqb_car.msg import CameraStatus

#NAVIO pwn out channels
SERVO_LEFT_PAN = rospy.get_param("/camera_control_driver/servo_left_pan")
SERVO_LEFT_TILT = rospy.get_param("/camera_control_driver/servo_left_tilt")
SERVO_RIGHT_PAN = rospy.get_param("/camera_control_driver/servo_right_pan")
SERVO_RIGHT_TILT = rospy.get_param("/camera_control_driver/servo_right_tilt")

#Camera servo pwm ranges (in ms)
LEFT_PAN_MIN = rospy.get_param("/camera_control_driver/left_pan_min")
LEFT_PAN_CENTER = rospy.get_param("/camera_control_driver/left_pan_center")
LEFT_PAN_MAX = rospy.get_param("/camera_control_driver/left_pan_max")

LEFT_TILT_MIN = rospy.get_param("/camera_control_driver/left_pan_min")
LEFT_TILT_CENTER = rospy.get_param("/camera_control_driver/left_tilt_center")
LEFT_TILT_MAX = rospy.get_param("/camera_control_driver/left_tilt_max")

RIGHT_PAN_MIN = rospy.get_param("/camera_control_driver/right_pan_min")
RIGHT_PAN_CENTER = rospy.get_param("/camera_control_driver/right_pan_center")
RIGHT_PAN_MAX = rospy.get_param("/camera_control_driver/right_pan_max")

RIGHT_TILT_MIN = rospy.get_param("/camera_control_driver/right_tilt_min")
RIGHT_TILT_CENTER = rospy.get_param("/camera_control_driver/right_tilt_center")
RIGHT_TILT_MAX = rospy.get_param("/camera_control_driver/right_tilt_max")

#Servo turn speed (default 0.001)
CAM_CTRL_SPEED =  rospy.get_param("/camera_control_driver/cam_ctrl_speed")

class CameraControlDriver(object):
    def __init__(self):
        # Define PWM channels on navio2 boards
        self.left_pan = navio.pwm.PWM(SERVO_LEFT_PAN)
        self.left_tilt = navio.pwm.PWM(SERVO_LEFT_TILT)
        self.right_pan  = navio.pwm.PWM(SERVO_RIGHT_PAN)
        self.right_tilt  = navio.pwm.PWM(SERVO_RIGHT_TILT)
        self.initialize_pwm()

        # Subscribe to camera direction
        self.sub_cam_ctrl = rospy.Subscriber('/control/cameras', CameraControl, self.callback)
        self.cam_ctrl_dir = CameraControl()
        
        # Publish camera status
        self.pub_cam_status = rospy.Publisher('/status/cameras', CameraStatus , queue_size=1)
        self.cam_status = CameraStatus()
        self.cam_status.left_pan_pwm = LEFT_PAN_CENTER
        self.cam_status.left_tilt_pwm = LEFT_TILT_CENTER
        self.cam_status.right_pan_pwm = RIGHT_PAN_CENTER
        self.cam_status.right_tilt_pwm = RIGHT_TILT_CENTER
        self.set_pwm()

        rospy.loginfo("Left Camera Initial Pan: " + str(self.cam_status.left_pan_pwm) + " Tilt: " + str(self.cam_status.left_tilt_pwm))
        rospy.loginfo("Right Camera Initial Pan: " + str(self.cam_status.right_pan_pwm) + " Tilt: " + str(self.cam_status.right_tilt_pwm))    

    # Capture cmd_vel 
    def callback(self, data):
        self.cam_ctrl_dir = data

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

        rospy.loginfo("Camera Control Initialized.")

    def set_pwm(self):
        # Set the servos to the appropriate pwm
        self.left_pan.set_duty_cycle(self.cam_status.left_pan_pwm)
        self.left_tilt.set_duty_cycle(self.cam_status.left_tilt_pwm)
        self.right_pan.set_duty_cycle(self.cam_status.right_pan_pwm)
        self.right_tilt.set_duty_cycle(self.cam_status.right_tilt_pwm)

        # Publish status topic
        self.pub_cam_status.publish(self.cam_status)

    def update_pwm(self):
        # Check if direction command is sent
        if self.cam_ctrl_dir.left_pan_dir != 0.0 or self.cam_ctrl_dir.left_tilt_dir != 0.0 or self.cam_ctrl_dir.right_pan_dir != 0.0 or self.cam_ctrl_dir.right_tilt_dir != 0.0:
            # Left Pan
            if (self.cam_ctrl_dir.left_pan_dir > 0 and self.cam_status.left_pan_pwm > LEFT_PAN_MAX) or (self.cam_ctrl_dir.left_pan_dir < 0 and self.cam_status.left_pan_pwm < LEFT_PAN_MIN):
                rospy.logwarn("Left pan out of range!")
            else:
                self.cam_status.left_pan_pwm += self.cam_ctrl_dir.left_pan_dir * CAM_CTRL_SPEED

            # Left Tilt
            if (self.cam_ctrl_dir.left_tilt_dir > 0 and self.cam_status.left_tilt_pwm > LEFT_TILT_MAX) or (self.cam_ctrl_dir.left_tilt_dir < 0 and self.cam_status.left_tilt_pwm < LEFT_TILT_MIN):
                rospy.logwarn("Left tilt out of range!")
            else:     
                self.cam_status.left_tilt_pwm += self.cam_ctrl_dir.left_tilt_dir * CAM_CTRL_SPEED
        
            # Right Pan
            if (self.cam_ctrl_dir.right_pan_dir > 0 and self.cam_status.right_pan_pwm > RIGHT_PAN_MAX) or (self.cam_ctrl_dir.right_pan_dir < 0 and self.cam_status.right_pan_pwm < RIGHT_PAN_MIN):
                rospy.logwarn("Right pan out of range!")
            else:
                self.cam_status.right_pan_pwm += self.cam_ctrl_dir.right_pan_dir * CAM_CTRL_SPEED

            # Right Tilt
            if (self.cam_ctrl_dir.right_tilt_dir > 0 and self.cam_status.right_tilt_pwm > RIGHT_TILT_MAX) or (self.cam_ctrl_dir.right_tilt_dir < 0 and self.cam_status.right_tilt_pwm < RIGHT_TILT_MIN):
                rospy.logwarn("Right tilt out of range!")
            else:
                self.cam_status.right_tilt_pwm += self.cam_ctrl_dir.right_tilt_dir * CAM_CTRL_SPEED
            
            self.set_pwm()

if __name__ == "__main__":
    rospy.init_node('camera_control_driver', log_level=rospy.INFO)
    camera_control_object = CameraControlDriver()
    rate = rospy.Rate(100)
    
    while not rospy.is_shutdown():
        camera_control_object.update_pwm()
        rate.sleep()