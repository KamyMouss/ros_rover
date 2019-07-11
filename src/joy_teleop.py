#! /usr/bin/env python

import rospy
from geometry_msgs.msg import Twist
from sensor_msgs.msg import Joy
from std_msgs.msg import Float32

# Joystick Mapping
# Motor
MOTOR_STEER_AXIS = 0     #Left joystick horizontal
MOTOR_MOVE_AXIS = 1      #Left joystick vertical

# Camera
CAMERA_PAN_AXIS = 3      #Right Joystick horizontal
CAMERA_TILT_AXIS = 4     #Right Joystick vertical
LEFT_CAMERA_SELECT = 3   #Square button
RIGHT_CAMERA_SELECT = 1  #Circle button


class JoyTeleop(object):
    def __init__(self):
        # Subscribe to joystick data
        self.sub = rospy.Subscriber('/joy', Joy, self.callback)

        # Publish control
        self.pub_cmd_vel = rospy.Publisher('/control/motor/cmd_vel', Twist, queue_size=1)
        self.pub_left_pan = rospy.Publisher('/control/camera/left/pan', Float32, queue_size=1)
        self.pub_left_tilt = rospy.Publisher('/control/camera/left/tilt', Float32, queue_size=1)
        self.pub_right_pan = rospy.Publisher('/control/camera/right/pan', Float32, queue_size=1)
        self.pub_right_tilt = rospy.Publisher('/control/camera/right/tilt', Float32, queue_size=1)
        
        self.selected_camera = "LEFT"

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
            rospy.loginfo("Left Camera Selected.")
        elif joy_buttons[RIGHT_CAMERA_SELECT] == 1 and self.selected_camera != "RIGHT":
            self.selected_camera = "RIGHT"
            rospy.loginfo("Right Camera Selected.")  
        
        # Camera Tilt Control
        if self.selected_camera == "LEFT": self.pub_left_tilt.publish(joy_axes[CAMERA_TILT_AXIS])
        elif self.selected_camera == "RIGHT": self.pub_right_tilt.publish(joy_axes[CAMERA_TILT_AXIS])  

        # Camera Pan Control
        if self.selected_camera == "LEFT": self.pub_left_pan.publish(joy_axes[CAMERA_PAN_AXIS])
        elif self.selected_camera == "RIGHT": self.pub_right_pan.publish(joy_axes[CAMERA_PAN_AXIS])
            

if __name__ == "__main__":
    rospy.init_node('joy_teleop', log_level=rospy.INFO)
    joy_teleop_object = JoyTeleop()
    rate = rospy.Rate(1)
    rospy.spin()