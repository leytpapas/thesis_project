#!/usr/bin/env python3
import rospy
import sys
from sensor_msgs.msg import Imu, Temperature
from geometry_msgs.msg import Pose
from tf.transformations import quaternion_from_euler
from math import radians
import serial
import subprocess


class Imu_unit(object):

    def __init__(self, imu_topic=rospy.get_param('/imu_topic'), rate=rospy.get_param('/rate'),debug=False):
        rospy.init_node('imu_pub', anonymous=True)
        # self.serial_port = 'ls /dev/ttyUSB*'
        self.serial_port = 'ls /dev/ttyACM*'

        p = subprocess.Popen(self.serial_port,
                     shell=True,
                     stdout=subprocess.PIPE, 
                     stderr=subprocess.PIPE)
        self.port, err = p.communicate()
        self.port = self.port.decode("utf-8").rstrip()
        self.ser = serial.Serial(
            port=self.port,
            baudrate=115200,
            # parity=serial.PARITY_NONE,
            # stopbits=serial.STOPBITS_ONE,
            # bytesize=serial.EIGHTBITS,
            timeout=1
            )
        self.debug = debug
        self.ser.flush()
        self.data = {
        "yaw": None,
        "pitch": None,
        "roll": None,
        "ax": None,
        "ay": None,
        "az": None,
        "gx": None,
        "gy": None,
        "gz": None
        }
        self.rate = rospy.Rate(rate)
        self.pub = rospy.Publisher(imu_topic+"/data", Imu, queue_size=1)
        self.temp_pub = rospy.Publisher(imu_topic+"/temp", Temperature, queue_size=1)
        rospy.loginfo("Robot Imu Publisher Started...")
        self.seq = 0
        # # Factors for unit conversions
        self.acc_fact = 1
        self.gyr_fact = 1


        self.time_present = rospy.Time.now()
        self.time_past = rospy.Time.now()
        self.accel_deadzone = 8
        self.gyro_deadzone = 1
        self.accel_x_offset = self.accel_y_offset = self.accel_z_offset = 0
        self.rot_x_offset = self.rot_y_offset = self.rot_z_offset = 0
        self.gyro_x_past = self.gyro_y_past = self.gyro_z_past = 0
        self.gyro_x_present = self.gyro_y_present = self.gyro_z_present = 0
        self.gyro_x_calli = self.gyro_y_calli = self.gyro_z_calli = 0
        self.rot_x = self.rot_y = self.rot_z = 0
        self.angle_x = self.angle_y = self.angle_z = 0
        # self.callibrate(100)
        self.Q = []
        self.G = []

    def serial_read(self):
        while True:
            try:
                if self.ser.in_waiting > 0:
                    # line = self.ser.readline()
                    # print(line.decode('utf-8'))
                    line = self.ser.readline().decode('utf-8').rstrip()
                    values = line.split(",")
                    for v in values:
                        key,val = v.split(":")
                        self.data[key] = float(val)
                    self.gyro_x_past = self.gyro_x_present
                    self.gyro_y_past = self.gyro_y_present
                    self.gyro_z_past = self.gyro_z_present
                    self.time_past = self.time_present
                    self.time_present = rospy.Time.now()
                    return True
            except:
                print(line)

        return False

    def calculate_angle(self):
        self.angle_x = self.angle_x + ((self.time_present - self.time_past).to_sec() * (self.gyro_x_present + self.gyro_x_past - 2 * self.gyro_z_calli)) * 0.00000382
        self.angle_y = self.angle_y + ((self.time_present - self.time_past).to_sec() * (self.gyro_x_present + self.gyro_y_past - 2 * self.gyro_z_calli)) * 0.00000382
        self.angle_z = self.angle_z + ((self.time_present - self.time_past).to_sec() * (self.gyro_x_present + self.gyro_z_past - 2 * self.gyro_z_calli)) * 0.00000382
        return self.angle_x, self.angle_y, self.angle_z

    # def callibrate(self, times):
    #     while not self.serial_read() and sum(1 for key,value in self.data.items() if value is not None )!=7: pass
    #     print(self.data)
    #     Ax, Ay, Az, self.gyro_x_present,self.gyro_y_present,self.gyro_z_present, tempC = self.data["ax"],self.data["ay"],self.data["az"],self.data["gx"],self.data["gy"],self.data["gz"],self.data["T"]
    #     ax_offset =- Ax/8
    #     ay_offset =- Ay/8
    #     az_offset = (16384-Az)/8

    #     gx_offset =- self.gyro_x_present/4
    #     gy_offset =- self.gyro_y_present/4
    #     gz_offset =- self.gyro_z_present/4
    #     print("Starting Calibration. Please wait a few seconds")
    #     while True:
    #         ready = 0
    #         self.accel_x_offset = ax_offset
    #         self.accel_y_offset = ay_offset
    #         self.accel_z_offset = az_offset

    #         self.rot_x_offset = gx_offset
    #         self.rot_y_offset = gy_offset
    #         self.rot_z_offset = gz_offset

    #         # self.enablePrint()
    #         while not self.serial_read() and sum(1 for key,value in self.data.items() if value is not None)!=7: pass
    #         Ax, Ay, Az, self.gyro_x_present,self.gyro_y_present,self.gyro_z_present, tempC = self.data["ax"]-ax_offset,self.data["ay"]-ay_offset,self.data["az"]-az_offset,self.data["gx"]-gx_offset,self.data["gy"]-gy_offset,self.data["gz"]-gz_offset,self.data["T"]
    #         # print(self.data)
    #         # Ax, Ay, Az, self.gyro_x_present,self.gyro_y_present,self.gyro_z_present, tempC, angle = self.fetch_data()
    #         # self.blockPrint()
    #         # print ("rot_x=%.2f" %self.gyro_x_present, u'\u00b0'+ "/s", "\trot_y=%.2f" %self.gyro_y_present, u'\u00b0'+ "/s", "\trot_z=%.2f" %self.gyro_z_present, u'\u00b0'+ "/s", "\tAx=%.2f g" %Ax, "\tAy=%.2f g" %Ay, "\tAz=%.2f g" %Az)
    #         if abs(Ax)<=self.accel_deadzone:
    #             ready+=1
    #         else:
    #             ax_offset = ax_offset - Ax/self.accel_deadzone
    #         if abs(Ay)<=self.accel_deadzone:
    #             ready+=1
    #         else:
    #             ay_offset = ay_offset - Ay/self.accel_deadzone
    #         if abs(16384 - Az)<=self.accel_deadzone:
    #             ready+=1
    #         else:
    #             az_offset = az_offset - (16384 - Az)/self.accel_deadzone
    #         if abs(self.gyro_x_present) <= self.gyro_deadzone:
    #             ready += 1
    #         else:
    #             gx_offset=gx_offset - self.gyro_x_present/(self.gyro_deadzone+1)

    #         if abs(self.gyro_z_present) <= self.gyro_deadzone:
    #             ready += 1
    #         else:
    #             gz_offset = gz_offset - self.gyro_z_present/(self.gyro_deadzone+1)

    #         if abs(self.gyro_z_present) <= self.gyro_deadzone:
    #             ready += 1
    #         else:
    #             gz_offset = gz_offset - self.gyro_z_present/(self.gyro_deadzone+1)

    #         if ready == 6: break

    #         # time.sleep(0.002)
    #         # sleep(.5)
    #     # self.gyro_x_calli = self.gyro_x_calli // times
    #     # self.gyro_y_calli = self.gyro_y_calli // times
    #     # self.gyro_z_calli = self.gyro_z_calli // times
    #     print("Calibration ended")
        
    def to_quaternion(self, yaw, pitch, roll): # yaw (Z), pitch (Y), roll (X)
        # Abbreviations for the various angular functions
        pose = Pose()
        quaternion = quaternion_from_euler(roll, pitch, yaw)
        # print(quaternion)
        pose.orientation.x = quaternion[0]
        pose.orientation.y = quaternion[1]
        pose.orientation.z = quaternion[2]
        pose.orientation.w = quaternion[3]
        return pose.orientation

    def step(self):

        msg = Imu()

        msg.header.frame_id = "imu_link"
        msg.header.stamp = rospy.Time.now()
        msg.header.seq = self.seq
        msg.orientation_covariance[0] = -1
        while not self.serial_read() and sum(1 for key,value in self.data.items() if value is not None)!=len(self.data.keys()): pass
        print("YAW", self.data["yaw"])
        Ax, Ay, Az = self.data["ax"]/16384, self.data["ay"]/16384, self.data["az"]/16384
        self.gyro_x_present,self.gyro_y_present,self.gyro_z_present = self.data["gx"]/131, self.data["gy"]/131, self.data["gz"]/131

        # x,y,z = Ax, Ay, Az
        msg.linear_acceleration.x = Ax / self.acc_fact
        msg.linear_acceleration.y = Ay / self.acc_fact
        msg.linear_acceleration.z = Az / self.acc_fact
        msg.linear_acceleration_covariance[0] = -1

        # x,y,z = self.gyro_x_present,self.gyro_y_present, self.gyro_z_present
        msg.angular_velocity.x = self.gyro_x_present / self.gyr_fact
        msg.angular_velocity.y = self.gyro_y_present / self.gyr_fact
        msg.angular_velocity.z = self.gyro_z_present / self.gyr_fact
        
        msg.orientation = self.to_quaternion(self.data["yaw"], self.data["pitch"], self.data["roll"])

        # msg_temp = Temperature()
        # msg_temp.header.frame_id = "imu_link"
        # msg_temp.header.stamp = rospy.Time.now()
        # msg_temp.header.seq = self.seq
        # msg_temp.temperature = tempC
        # self.temp_pub.publish(msg_temp)
        self.pub.publish(msg)
        self.seq += 1

    def spin(self):
        while not rospy.is_shutdown():
            if self.serial_read():
                self.step()
            self.rate.sleep()

if __name__ == '__main__':
    robot_sense = Imu_unit()
    try:
        robot_sense.spin()
    except rospy.ROSInterruptException:
        pass