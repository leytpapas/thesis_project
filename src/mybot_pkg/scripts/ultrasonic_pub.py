#!/usr/bin/env python3
import rospy
import sys
from sensor_msgs.msg import Range,Temperature
from ultrasonic_sensors_driver import Ultrasonic_driver
from math import radians
class RobotUltraSensing(object):

    def __init__(self, ultrasonic_sensor_topic=rospy.get_param('/ultrasonic_sensor_topic'), gpio_list_trigger=rospy.get_param('/gpio_list_trigger') ,gpio_list_echo=rospy.get_param('/gpio_list_echo'), positions=rospy.get_param('/positions'), rate=rospy.get_param('/rate'),debug=False):

        self.ultrasonic_driver = Ultrasonic_driver(gpio_list_trigger, gpio_list_echo, positions,blocking=False)
        self.rate = rospy.Rate(rate*2*3)
        self.ultrasonic_sensor_topic = ultrasonic_sensor_topic
        self.pub = {pos:rospy.Publisher(self.ultrasonic_sensor_topic+"/"+pos, Range, queue_size=6) for pos in positions}
        # self.pub = rospy.Publisher(ultrasonic_sensor_topic, Range, queue_size=3)
        # rospy.wait_for_service('/raspicam_node/start_capture')
        rospy.loginfo("RobotUltraSensing Started...")
        self.seq = 0
        self.debug = debug
        self.buffer_size = rospy.get_param('/buffer_size')
        self.buffer = {p:[0]*self.buffer_size for p in positions}
        self.data_temp = None
        try:
            rospy.Subscriber(rospy.get_param("/imu_topic")+"/temp", Temperature, self.get_data_temp)
        except:
            pass
    def get_data_temp(self, data):
        self.data_temp = data

    def publisher(self):
        while not rospy.is_shutdown():
            if self.data_temp is not None:
                self.ultrasonic_driver.set_temp(self.data_temp.temperature)
                
            for name, distance in self.ultrasonic_driver.distance_all():
                if self.debug:
                    print(name,distance)
                msg = Range()
                msg.header.frame_id = "ultrasonic_"+ name
                msg.header.stamp = rospy.Time.now()
                msg.header.seq = self.seq
                msg.radiation_type = 0 # Ultrasonic
                msg.field_of_view = radians(15)
                msg.min_range = .02
                msg.max_range = 4
                self.buffer[name] = [distance] + self.buffer[name][:-1]
                mean_distance = sum(self.buffer[name])/self.buffer_size
                msg.range = round(mean_distance, 2)
                if msg.range >  msg.max_range:
                    continue
                self.pub[name].publish(msg)
            self.seq += 1
            self.rate.sleep()



if __name__ == '__main__':
    rospy.init_node('ultrasonic_pub', anonymous=True)
    robot_sense = RobotUltraSensing()
    try:
        robot_sense.publisher()
    except rospy.ROSInterruptException:
        pass
