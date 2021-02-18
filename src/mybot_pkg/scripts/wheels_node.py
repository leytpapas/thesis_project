#!/usr/bin/env python3
import rospy
import sys
from geometry_msgs.msg import Point, Pose, Quaternion, Twist, Vector3
from nav_msgs.msg import Odometry
from md25_driver import MD25_driver
from sensor_msgs.msg import JointState, BatteryState
from math import radians, pi, fabs, degrees
from tf.transformations import euler_from_quaternion
import tf
import time

class RobotMover(object):

    def __init__(self,cmd_topic=rospy.get_param('/cmd_vel_topic'), joint_frames=rospy.get_param('/joint_frames'), rate=rospy.get_param('/rate'),debug=False):
        rospy.init_node('Wheels Node', anonymous=True)
        # self.joint_state_topics = joint_state_topics
        self.debug=debug
        self.joint_frames = joint_frames
        self.cmd_topic = cmd_topic
        rospy.Subscriber(self.cmd_topic, Twist, self.cmd_twist_callback)

        self.motor_driver = MD25_driver()
        self.motor_driver.mode_set(3)
        self.odom_topic=rospy.get_param("/odom_topic")
        self.odom_pub = rospy.Publisher(self.odom_topic, Odometry, queue_size=2)
        self.base_link_topic=rospy.get_param("/base_link_topic")
        self.dimensions = rospy.get_param("/dimensions")
        self.wheel_diameter = self.dimensions["wheel_diameter"]
        self.distance_per_count = (pi * self.wheel_diameter) / rospy.get_param("/encoder_ticks") # N ticks per 360 turn

        self.length_between_two_wheels = self.dimensions["diff"]
        self.odom_broadcaster = tf.TransformBroadcaster()
        self.encoder_min = self.motor_driver.get_encoder_min()
        self.encoder_max = self.motor_driver.get_encoder_max()
        self.encoder_left = self.motor_driver.encoder1_get()
        self.encoder_right = self.motor_driver.encoder2_get()
        self.battery_voltage = self.motor_driver.bat_voltage_get()
        self._PreviousRightEncoderCounts = self.encoder_left
        self._PreviousLeftEncoderCounts = self.encoder_right

        # self.pub_joint_left = rospy.Publisher(self.joint_state_topics[0], JointState, queue_size=10)
        # self.pub_joint_right = rospy.Publisher(self.joint_state_topics[1], JointState, queue_size=10)
        self.pub_battery_state = rospy.Publisher(rospy.get_param("/battery_topic"), BatteryState, queue_size=2)

        self.pub_joint_left = rospy.Publisher("/joint_states", JointState, queue_size=2)
        self.pub_joint_right = rospy.Publisher("/joint_states", JointState, queue_size=2)
        # rospy.wait_for_service('/raspicam_node/start_capture')
        self.rate = rospy.Rate(rate)
        rospy.loginfo("Wheels Started...")
        self.seq = 0

        self.pos_left_past = self.pos_right_past = 0
        self.past_time = rospy.Time.now()
        self.local_pose = {
            "x":0,
            "y":0,
            "th":0
        }
        self.local_pose = {
            "x":0,
            "y":0,
            "th":0
        }
        self.local_vel = {
            "x_lin":0,
            "y_lin":0,
            "y_ang":0
        }
        self.yaw = 0

    def spin(self):

        while not rospy.is_shutdown():
            self.publish_state()
            self.rate.sleep()

    # def set_wheels_speeds(self, left, right):
    #     self.motor_driver.speed1_set(int(left*self.max_rpm))
    #     self.motor_driver.speed2_set(int(right*self.max_rpm))

    def publish_state(self):

            current_time = rospy.Time.now()
            self.encoder_left = self.motor_driver.encoder1_get()
            self.encoder_right = self.motor_driver.encoder2_get()
            self.battery_voltage = self.motor_driver.bat_voltage_get()

            msg_battery = BatteryState()
            msg_battery.voltage = self.battery_voltage
            msg_battery.power_supply_status = 2
            msg_battery.power_supply_health = 1
            msg_battery.power_supply_technology = 0
            msg_battery.present = True

            msg_left = JointState()
            msg_left.header.frame_id = self.joint_frames[0]# + "_hinge"
            msg_left.header.stamp = current_time
            msg_left.header.seq = self.seq
            
            msg_right = JointState()
            msg_right.header.frame_id = self.joint_frames[1]# + "_hinge"
            msg_right.header.stamp = current_time
            msg_right.header.seq = self.seq
            
            self.encoder_left = self.encoder_left
            self.encoder_right = self.encoder_right
            # print("Left",left,"Right",right)
            msg_left.name.append(self.joint_frames[0]+"_hinge") # 360 ticks/ per wheel turn
            msg_left.position.append(self.encoder_left) # 360 ticks/ per wheel turn

            msg_right.name.append(self.joint_frames[1]+"_hinge")
            msg_right.position.append(self.encoder_right) # 360 ticks/ per wheel turn

            self.pub_joint_right.publish(msg_right)
            self.pub_joint_left.publish(msg_left)
            self.pub_battery_state.publish(msg_battery)

            # extract the wheel velocities from the tick signals count
            # 
            if (current_time - self.past_time).to_sec()!=0 and (self.encoder_left!=self._PreviousLeftEncoderCounts or self.encoder_right!=self._PreviousRightEncoderCounts):
                msg_odom = Odometry()
                # --- Get the distance delta since the last period --- 
                deltaLeft = (self.encoder_left - self._PreviousLeftEncoderCounts) * self.distance_per_count
                deltaRight = (self.encoder_right - self._PreviousRightEncoderCounts) * self.distance_per_count
                # --- Update the local position and orientation ---
                self.local_pose["x"] = (deltaLeft + deltaRight) / 2.0 # distance in X direction 
                self.local_pose["y"] = 0.0; # distance in Y direction
                self.local_pose["th"] = (deltaRight - deltaLeft) / self.length_between_two_wheels # Change in orientation
                if -360 > self.local_pose["th"] or self.local_pose["th"] > 360 :
                    return
                # self.yaw += self.local_pose["th"]
                
                # --- Update the velocity ---
                leftDistance = (deltaLeft - self.pos_left_past)
                rightDistance = (deltaRight - self.pos_right_past)
                delta_distance = (leftDistance + rightDistance) / 2.0
                delta_theta = (rightDistance - leftDistance) / self.length_between_two_wheels # in radians

                self.local_vel["x_lin"] = delta_distance / (current_time-self.past_time).to_sec() # Linear x velocity
                self.local_vel["y_lin"] = 0.0; 
                self.local_vel["y_ang"] = (delta_theta / (current_time-self.past_time).to_sec()) # In radians per/sec

                odom_quat = tf.transformations.quaternion_from_euler(0, 0, self.local_pose["th"])
                # first, we'll publish the transform over tf
                # send the transform
                self.odom_broadcaster.sendTransform(
                    (self.local_pose["x"], self.local_pose["y"], 0.),
                    odom_quat,
                    current_time,
                    self.base_link_topic[1:],
                    self.odom_topic[1:]
                )
                msg_odom.header.stamp = current_time
                msg_odom.header.frame_id = self.odom_topic[1:]
                msg_odom.header.seq = self.seq
                # set the position
                msg_odom.pose.pose = Pose(Point(self.local_pose["x"], self.local_pose["y"], 0.), Quaternion(*odom_quat))

                # set the velocity
                msg_odom.child_frame_id = self.base_link_topic[1:]
                msg_odom.twist.twist = Twist(Vector3(self.local_vel["x_lin"], self.local_vel["y_lin"], 0), Vector3(0, 0, self.local_vel["y_ang"]))

                # publish the message
                self.odom_pub.publish(msg_odom)
                self.seq += 1

                # ---  Save the last position values ---
                self.pos_left_past = deltaLeft 
                self.pos_right_past = deltaRight
                
                self.past_time = current_time
                self._PreviousLeftEncoderCounts = self.encoder_left
                self._PreviousRightEncoderCounts = self.encoder_right

    def calculateArcWheelRatios(self, radius):
        inside_wheel = 2*math.pi(self.wheel_diameter) - (self.length_between_two_wheels)
        outside_wheel = 2*math.pi(self.wheel_diameter) + (self.length_between_two_wheels)
        return inside_wheel/outside_wheel
        
    def cmd_twist_callback(self, msg):
        linear_speed = msg.linear.x
        angular_speed = msg.angular.z

        # Decide Speed
        self.motor_driver.speed_set(int(linear_speed))
        self.motor_driver.turn_set(int(angular_speed))
        # self.publish()

    def listener(self):
        rospy.spin()

if __name__ == '__main__':
    robot_mover = RobotMover()
    robot_mover.spin()