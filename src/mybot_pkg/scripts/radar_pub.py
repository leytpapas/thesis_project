#!/usr/bin/env python3

import rospy
from sensor_msgs.msg import LaserScan
from adafruit_servokit import ServoKit
from ultrasonic_sensors_driver import Ultrasonic_driver
import time
import math

class Radar(object):
    """docstring for Radar"""
    def __init__(self, gpio_servo=rospy.get_param("/servo"), gpio_list_trigger=rospy.get_param("/gpio_radar_list_trigger"),gpio_list_echo=rospy.get_param("/gpio_radar_list_echo"), radar_topic=rospy.get_param("/radar_topic"), rate=rospy.get_param("/rate"), positions=["left", "right"], angle_region=180//5):
        super(Radar, self).__init__()
        rospy.init_node('radar_pub', anonymous=True)
        #GPIO Mode (BOARD / BCM)
        self.gpio_servo = gpio_servo
        #BCM
        self.gpio_list_trigger = gpio_list_trigger
        self.gpio_list_echo = gpio_list_echo
        self.radar_topic = radar_topic

        # gpio_list_trigger = [10,9]
        # gpio_list_echo = [8,7]

        self.kit = ServoKit(channels=8) #we dont need 16
        self.angle = 0
        self.spare_degrees = (180-125)//2 
        self.radar_toggle = True
        self.angle_region = angle_region
        # kit.servo[0].angle = i

        self.sensors = Ultrasonic_driver(gpio_list_trigger, gpio_list_echo, positions)

        self.rate = rospy.Rate(rate)
        self.pub = rospy.Publisher(self.radar_topic, LaserScan, queue_size=2)
        
        rospy.loginfo("RoboRadar Started...")

    def range_to_laser_scan(self):
        """
        Header header            # timestamp in the header is the acquisition time of 
                                 # the first ray in the scan.
                                 #
                                 # in frame frame_id, angles are measured around 
                                 # the positive Z axis (counterclockwise, if Z is up)
                                 # with zero angle being forward along the x axis
                                 
        float32 angle_min        # start angle of the scan [rad]
        float32 angle_max        # end angle of the scan [rad]
        float32 angle_increment  # angular distance between measurements [rad]

        float32 time_increment   # time between measurements [seconds] - if your scanner
                                 # is moving, this will be used in interpolating position
                                 # of 3d points
        float32 scan_time        # time between scans [seconds]

        float32 range_min        # minimum range value [m]
        float32 range_max        # maximum range value [m]

        float32[] ranges         # range data [m] (Note: values < range_min or > range_max should be discarded)
        float32[] intensities    # intensity data [device-specific units].  If your
                                 # device does not provide intensities, please leave
                                 # the array empty.
        """
        # actualrange = 0 <--> 125 degrees
        msg = LaserScan()
        msg.header.frame_id = "radar"
        msg.header.stamp = rospy.Time.now()
        msg.range_max = 4.00
        msg.range_min = 0.02
        msg.angle_min = math.radians(0)
        msg.angle_max = math.radians(360)

        # msg.angle_min = math.radians(-90)
        # msg.angle_max = math.radians((270))

        msg.angle_increment = math.radians(0.7)
        
        msg.ranges = [4.0] * int((math.degrees(msg.angle_max) - math.degrees(msg.angle_min))/0.7) # radius / increment = number of samples

        time_started = rospy.Time.now()
        for a in range(0 if self.radar_toggle else 180, 180 if self.radar_toggle else 0, 1 if self.radar_toggle else -1):
            self.set_angle(a)
            # time.sleep(0.1)
            for name, distance in self.sensors.distance_all():
                if name == "right":
                    msg.ranges[a+self.spare_degrees] = min([msg.ranges[a+self.spare_degrees],distance])
                else:
                    msg.ranges[a+256+self.spare_degrees] = min([msg.ranges[a+256+self.spare_degrees],distance])

        elapsed = (rospy.Time.now() - time_started).to_sec()
        msg.time_increment =  elapsed
        # if not self.radar_toggle: # if its the opposite scan, reverse the list
        #     # msg.ranges = msg.ranges[:len(msg.ranges)//2:-1]+msg.ranges[len(msg.ranges)//2::-1]
        #     msg.ranges = msg.ranges[::-1]
        self.pub.publish(msg)
        self.radar_toggle = not self.radar_toggle

    def set_angle(self, angle):
        self.kit.servo[0].angle = angle

    def get_angle(self):
        return self.kit.servo[0].angle
    
    def get_gpio_echo(self):
        return self.gpio_list_echo
    
    def get_gpio_trigger(self):
        return self.gpio_list_trigger

    def spin(self):
        while not rospy.is_shutdown():
            self.range_to_laser_scan()
            self.rate.sleep()
        self.set_angle(0)

 
if __name__ == '__main__':
    gpio_list_trigger = [4,17]
    gpio_list_echo = [14,18]
    servo = 0
    driver = Radar()
    driver.spin()