#!/usr/bin/env python3

from math import sin, cos, pi

import rospy
import tf
from nav_msgs.msg import Odometry
from geometry_msgs.msg import Point, Pose, Quaternion, Twist, Vector3
from sensor_msgs.msg import JointState
from md25_driver import MD25_driver

class Odom(object):
  """docstring for Odom"""
    def __init__(self,joint_frames=rospy.get_param('/joint_frames'),odom_topic=rospy.get_param("/odom_topic"), base_link_topic=rospy.get_param("/base_link_topic")):
        super(Odom, self).__init__()
        rospy.init_node('odom_pub')
        self.joint_frames = joint_frames
        self.odom_topic = odom_topic  
        self.base_link_topic = base_link_topic
        self.odom_pub = rospy.Publisher(self.odom_topic, Odometry, queue_size=10)
        self.odom_broadcaster = tf.TransformBroadcaster()
        # self.motor_driver = MD25_driver()
        # self.motor_driver.mode_set(3)
        self.dimensions = rospy.get_param("/dimensions")
        self.distance_per_count = (pi * self.dimensions["wheel_diameter"]) / rospy.get_param("/encoder_ticks")
        self.length_between_two_wheels = self.dimensions["diff"]
        rospy.Subscriber("/joint_states", JointState, self.get_encoders)

        self.x = 0.0
        self.y = 0.0
        self.th = 0.0
        self._PreviousLeftEncoderCounts = 0
        self._PreviousRightEncoderCounts = 0

        self.vx = 0
        self.vy = 0
        self.vth = 0

        # self.current_time = rospy.Time(2)
        self.past_time = rospy.Time.now()

        self.rate = rospy.Rate(rospy.get_param("/rate"))
        rospy.loginfo('odom_pub initialized')

        self.left_encoder = None
        self.right_encoder = None
        self.left_wheel_cmd = 
        self.right_wheel_cmd = 
        self.seq = 0

    def get_encoders(self, msg):
        if msg.header.frame_id == self.joint_frames[0]:
            self.left_encoder = msg.position[0]
        elif msg.header.frame_id == self.joint_frames[1]:
            self.right_encoder = msg.position[0]

    def spin(self):
        while not rospy.is_shutdown():
          if self.left_encoder != None and self.right_encoder != None:
            self.step()
            # to invalidate 
            self.left_encoder = None
            self.right_encoder = None
          self.rate.sleep()
  
    def step(self):
        current_time = rospy.Time.now()
        # if current_time==self.
        # MUST CHECK FOR LEFT/ RIGHT
        # extract the wheel velocities from the tick signals count
        tick_x = self.left_encoder
        tick_y = self.right_encoder
        
        deltaLeft = tick_x - self._PreviousLeftEncoderCounts
        deltaRight = tick_y - self._PreviousRightEncoderCounts

        omega_left = (deltaLeft * self.distance_per_count) / ((self.current_time - self.past_time).to_sec())#+0.0001)
        omega_right = (deltaRight * self.distance_per_count) / ((self.current_time - self.past_time).to_sec())#+0.0001)

        v_left = omega_left * 0.095 # radius
        v_right = omega_right * 0.095

        self.vx = ((v_right + v_left) / 2)*10
        self.vy = 0
        self.vth = ((v_right - v_left)/self.length_between_two_wheels) # *10

        dt = (self.current_time - self.past_time).to_sec()
        delta_x = (self.vx * cos(self.th)) * dt
        delta_y = (self.vx * sin(self.th)) * dt
        delta_th = self.vth * dt

        self.x += delta_x
        self.y += delta_y
        self.th += delta_th

        # odom_quat = tf::createQuaternionMsgFromYaw(th)
        odom_quat = tf.transformations.quaternion_from_euler(0, 0, self.th)

        # first, we'll publish the transform over tf
        # send the transform
        self.odom_broadcaster.sendTransform(
            (self.x, self.y, 0.),
            odom_quat,
            self.current_time,
            self.base_link_topic[1:],
            self.odom_topic[1:]
        )
        odom.header.stamp = self.current_time
        odom.header.frame_id = self.odom_topic[1:]
        odom.header.seq = self.seq
        self.seq += 1
        # set the position
        odom.pose.pose = Pose(Point(self.x, self.y, 0.), Quaternion(*odom_quat))

        # set the velocity
        odom.child_frame_id = self.base_link_topic[1:]
        odom.twist.twist = Twist(Vector3(self.vx, self.vy, 0), Vector3(0, 0, self.vth))

        # publish the message
        self.odom_pub.publish(odom)

        self.past_time = self.current_time
        self._PreviousLeftEncoderCounts = tick_x
        self._PreviousRightEncoderCounts = tick_y

if __name__ == '__main__':
    odom = Odom()
    try:
        odom.spin()
    except rospy.ROSInterruptException:
        pass