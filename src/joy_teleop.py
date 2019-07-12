#! /usr/bin/env python

import rospy
from geometry_msgs.msg import Twist
from sensor_msgs.msg import Joy
from std_msgs.msg import Float32
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
        self.pub_cmd_vel = rospy.Publisher('/control/motor/cmd_vel', Twist, queue_size=1)
        self.pub_camera_control = rospy.Publisher('/control/camera', CameraControl, queue_size=1)
        
        self.camera_control = CameraControl()
        
        self.selected_camera = "LEFT"
        rospy.loginfo("Left Camera Control Selected.")

        self.cmd_vel = Twist()

    def callback(self, data):
        joy_axes = data.axes
        joy_buttons = data.buttons
        
        # Motor Control
        self.cmd_vel.angular.z = joy_axes[MOTOR_STEER_AXIS]
        self.cmd_vel.linear.x = joy_axes[MOTOR_MOVE_AXIS]
        self.pub_cmd_vel.publish(self.cmd_vel)

        # Camera Select Control
        if joy_buttons[LEFT_CAMERA_SELECT] == 1 and self.selected_camera != "LEFT":
            self.selected_camera = "LEFT"
            rospy.loginfo("Left Camera Control Selected.")
        elif joy_buttons[RIGHT_CAMERA_SELECT] == 1 and self.selected_camera != "RIGHT":
            self.selected_camera = "RIGHT"
            rospy.loginfo("Right Camera Control Selected.")  
        
        # Camera Pan Control
        pan_dir = joy_axes[CAMERA_PAN_AXIS]
        if self.selected_camera == "LEFT": self.camera_control.left_pan_dir = pan_dir
        elif self.selected_camera == "RIGHT": self.camera_control.right_pan_dir = pan_dir
        
        # Camera Tilt Control
        tilt_dir = joy_axes[CAMERA_TILT_AXIS]
        if self.selected_camera == "LEFT": self.camera_control.left_tilt_dir = tilt_dir
        elif self.selected_camera == "RIGHT": self.camera_control.right_tilt_dir = tilt_dir 

        self.pub_camera_control.publish(self.camera_control)

if __name__ == "__main__":
    rospy.init_node('joy_teleop', log_level=rospy.INFO)
    joy_teleop_object = JoyTeleop()
    rate = rospy.Rate(1)
    rospy.spin()