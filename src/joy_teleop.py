#! /usr/bin/env python

import rospy
from geometry_msgs.msg import Twist
from sensor_msgs.msg import Joy
from yqb_car.msg import CameraControl

# Joystick Mapping
# Motor
MOTOR_STEER_AXIS = rospy.get_param("/joy_teleop/motor_steer_axis")
MOTOR_MOVE_AXIS = rospy.get_param("/joy_teleop/motor_move_axis")

# Camera
CAMERA_PAN_AXIS = rospy.get_param("/joy_teleop/camera_pan_axis")
CAMERA_TILT_AXIS = rospy.get_param("/joy_teleop/camera_tilt_axis")
LEFT_CAMERA_SELECT = rospy.get_param("/joy_teleop/left_camera_select")
RIGHT_CAMERA_SELECT = rospy.get_param("/joy_teleop/right_camera_select")


class JoyTeleop(object):
    def __init__(self):
        # Subscribe to joystick data
        self.sub = rospy.Subscriber('/joy', Joy, self.callback)

        # Publish control
        self.pub_cmd_vel = rospy.Publisher('/control/wheels/cmd_vel', Twist, queue_size=1)
        self.pub_camera_control = rospy.Publisher('/control/camera', CameraControl, queue_size=1)
        
        self.camera_control = CameraControl()
        
        self.selected_camera = "LEFT"
        rospy.loginfo("Left Camera Control Selected.")

        self.cmd_vel = Twist()

    def callback(self, data):
        joy_axes = data.axes
        joy_buttons = data.buttons
        
        # Motor Control
        if joy_axes[MOTOR_STEER_AXIS] != 0.0 or joy_axes[MOTOR_MOVE_AXIS] != 0.0:
            self.cmd_vel.angular.z = joy_axes[MOTOR_STEER_AXIS]
            self.cmd_vel.linear.x = joy_axes[MOTOR_MOVE_AXIS]
            self.pub_cmd_vel.publish(self.cmd_vel)
        else: 
            #Complicated way to avoid publishing when not needed
            if self.cmd_vel.angular.z != 0.0 or self.cmd_vel.linear.x != 0.0:
                self.cmd_vel.angular.z = 0.0
                self.cmd_vel.linear.x = 0.0
                self.pub_cmd_vel.publish(self.cmd_vel)


        # Camera Select Control
        if joy_buttons[LEFT_CAMERA_SELECT] == 1 and self.selected_camera != "LEFT":
            self.selected_camera = "LEFT"
            rospy.loginfo("Left Camera Control Selected.")
        elif joy_buttons[RIGHT_CAMERA_SELECT] == 1 and self.selected_camera != "RIGHT":
            self.selected_camera = "RIGHT"
            rospy.loginfo("Right Camera Control Selected.")  
        
        # Camera Pan/Tilt Control
        joy_pan_dir = joy_axes[CAMERA_PAN_AXIS]
        joy_tilt_dir = joy_axes[CAMERA_TILT_AXIS]

        if joy_pan_dir != 0.0 or joy_tilt_dir != 0.0:
            if self.selected_camera == "LEFT": 
                self.camera_control.left_pan_dir = joy_pan_dir
                self.camera_control.left_tilt_dir = joy_tilt_dir
            elif self.selected_camera == "RIGHT": 
                self.camera_control.right_pan_dir = joy_pan_dir
                self.camera_control.right_tilt_dir = joy_tilt_dir 

            self.pub_camera_control.publish(self.camera_control)
        else: #Complicated way to avoid publishing when not needed
            if self.camera_control.left_pan_dir != 0.0 or self.camera_control.right_pan_dir != 0.0:
                self.camera_control.left_pan_dir = 0.0
                self.camera_control.right_pan_dir = 0.0
                self.pub_camera_control.publish(self.camera_control)

            if self.camera_control.left_tilt_dir != 0.0 or self.camera_control.right_tilt_dir != 0.0:
                self.camera_control.left_tilt_dir = 0.0
                self.camera_control.right_tilt_dir = 0.0
                self.pub_camera_control.publish(self.camera_control)

        

if __name__ == "__main__":
    rospy.init_node('joy_teleop', log_level=rospy.INFO)
    joy_teleop_object = JoyTeleop()
    rate = rospy.Rate(1)
    rospy.spin()