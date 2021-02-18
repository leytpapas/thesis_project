#!/usr/bin/env python3
import rospy
import os 
from geometry_msgs.msg import Point, Pose, Quaternion, Twist
from sensor_msgs.msg import Joy
from std_msgs.msg import Bool
import tf
from math import radians

class TeleopJoy(object):
    """docstring for TeleopJoy"""
    def __init__(self):
        super(TeleopJoy, self).__init__()
        rospy.Subscriber("/joy", Joy, self.joy_callback)
        rospy.init_node('teleop_joy_node', anonymous=True)
        self.linear_ = 7 # axes[1] -> left axes up/down , axes[7] -> arrows up/down
        self.angular_ = 6 # axes[3] -> right axes left/right, axes[6] -> arrows left/right
        self.button_x = 0 # button[0] -> X

        self.l_scale_ = 30
        self.a_scale_ = 30
        self.pub = rospy.Publisher("/cmd_vel", Twist, queue_size=1)
        self.pub_s = rospy.Publisher("/take_step", Bool, queue_size=1)

        rospy.loginfo("Teleop node started")

        rospy.spin()

    def joy_callback(self, joy):
        msg = Twist()
        if int(joy.buttons[self.button_x])==1:
            self.pub_s.publish(Bool())
            # rospy.Rate(1).sleep()
        else:
            msg.linear.x = self.l_scale_ * joy.axes[self.linear_]
            msg.angular.z = -self.a_scale_ * joy.axes[self.angular_]
            self.pub.publish(msg)

if __name__ == '__main__':
    TeleopJoy()
