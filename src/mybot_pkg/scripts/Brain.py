#!/usr/bin/env python3
import math
import random
import time
import cv2
# import pygame
import rospy
from cv_bridge import CvBridge, CvBridgeError
from nav_msgs.msg import Odometry
# from pyrobot.brain import Brain
from sensor_msgs.msg import Image, LaserScan, Range,Imu
from std_msgs.msg import Bool
from geometry_msgs.msg import Twist
from tf.transformations import euler_from_quaternion

import detect_mod

# -----------------------------------------------------------------------

class SubsumptionBrain():
    """docstring for SubsumptionBrain"""
    def __init__(self, camera_topic=rospy.get_param("/camera_topic"), radar_topic=rospy.get_param("/radar_topic"), ultrasonic_topic=rospy.get_param("/ultrasonic_sensor_topic"), imu_topic=rospy.get_param("/imu_topic"), cmd_vel_topic=rospy.get_param("/cmd_vel_topic"), rate=rospy.get_param("/rate")):
        # super(SubsumptionBrain, self).__init__()
        self.LAST_MOVE = 0.0  # default = LEFT
        self.behaviors = []
        # self.image_sub =
        self.rate = rospy.Rate(rate)
        self.bridge = CvBridge()
        self.data_camera = None
        rospy.Subscriber(camera_topic, Image, self.get_data_camera,tcp_nodelay=True)
        self.data_radar = None
        # rospy.Subscriber(radar_topic, LaserScan, self.get_data_radar) # check inside function whether data came from left or right UltraSensor
        
        self.data_sonar = None

        self.data_ultrasonic = { p:None for p in rospy.get_param("/positions")}

        self.positions = rospy.get_param("/positions")
        for pos in self.positions:
            rospy.Subscriber(ultrasonic_topic+"/"+pos, Range, self.get_data_ultrasonic)

        self.pub_image = rospy.Publisher("/mybot/output_im", Image, queue_size=10)
        self.image_msg = Image()

        self.pub_cmd_vel = rospy.Publisher(cmd_vel_topic, Twist, queue_size=10)
        self.movement_pub = Twist()


        self.pub_laser = rospy.Publisher("/mybot/scan", LaserScan, queue_size=10)
        self.msg = LaserScan()
        rospy.Subscriber('/odom', Odometry, self.get_rotation)
        
        self.pose = None
        # rospy.Subscriber(imu_topic+"/data", Imu, self.get_data_imu)

        # add behaviors// highest priorities first:
        self.add_behavior(AvoidBehavior())
        self.add_behavior(WanderBehavior())
        self.add_behavior(FixPositionBehavior())
        self.add_behavior(SearchBehavior())

        self.roll = self.pitch = self.yaw = 0.0
        self.flag=False
        self.ERROR_OFFSET = math.radians(10)  # 10 degrees margin of error (in radians)
        self.kP = .5
        rospy.Subscriber("/take_step", Bool, self.step)

        # while data_odom is None:
        #     try:
        #         data_odom = rospy.wait_for_message("/odom", Odometry, timeout=1)
        #     except:
        #         rospy.loginfo("Current odom not ready yet, retrying for setting up init pose")
        # # self.get_rotation(data_odom)
        # self.yaw = None
        # while self.yaw==None:
        #     time.sleep(1)

        self.target_angle = self.yaw
        self.speed = 0

    def move(self, speed, rotation):
        self.movement_pub.linear.x = speed * 10 * 4
        self.movement_pub.angular.z = -rotation * 10
        self.pub_cmd_vel.publish(self.movement_pub)

    def normalize_angle(self, angle):
        if(math.fabs(angle) > math.pi):
            angle = angle - (2 * math.pi * angle) / (math.fabs(angle))
        return round(angle,1)

    def add_behavior(self, behavior, extras=None):
        # give the behavior access to the robot object
        behavior.robot = self
        self.behaviors.append(behavior)

    def get_rotation(self, msg):
        orientation_q = msg.pose.pose.orientation
        orientation_list = [orientation_q.x, orientation_q.y, orientation_q.z, orientation_q.w]
        (roll, pitch, yaw) = euler_from_quaternion(
            orientation_list)  # self.roll, self.pitch, self.yaw == in radiants
        self.roll += roll
        self.pitch += pitch
        self.yaw += yaw
        self.yaw = self.normalize_angle(self.yaw) 


    
    def get_data_imu(self, data):
        self.pose = data
        orientation_q = self.pose.orientation
        orientation_list = [orientation_q.x, orientation_q.y, orientation_q.z, orientation_q.w]
        (self.roll, self.pitch, self.yaw) = euler_from_quaternion(
            orientation_list) # self.roll, self.pitch, self.yaw == in radiants

    def get_data_ultrasonic(self, data):
        self.data_ultrasonic[data.header.frame_id.replace("ultrasonic_","")] = data

    def get_data_radar(self, data):
        self.data_radar = data

    def get_data_camera(self, data):
        try:
            # print(data.encoding)
            self.data_camera = self.bridge.imgmsg_to_cv2(data) # should be in rgb8
        except CvBridgeError as e:
            rospy.loginfo(e)
        # self.rate.sleep()

    def laser_to_range(self):
        # split 180 degrees in front -> 5 regions
        front_ranges = len(self.data_radar.ranges) / 2
        step = int(front_ranges/5)
        return {
        "right": min(self.data_radar.ranges[step*0:step*1]),
        "front_right": min(self.data_radar.ranges[step*1:step*2]),
        "front": min(self.data_radar.ranges[step*2:step*3]),
        "front_left": min(self.data_radar.ranges[step*3:step*4]),
        "left": min(self.data_radar.ranges[step*4:step*5])
        }


    def step(self, data=None):
        # while we tell the robot to move in a certain way,
        # we usually dont succeed in completing the rotation
        # (before deciding the next step), due to mechanical parts
        # OR road is slippery etc..

        behavior = self.updateAll()
        if behavior is None:
            # print("Reducing speed")
            # self.speed *= .9
            # self.move(self.speed,0)
            return
        print ("%s is in control" % behavior)
        print ("Current move: speed:" + str(behavior.speed) + " rotate:" + str(
            math.degrees(behavior.rotate)) + " YAW " + str(
            math.degrees(self.yaw)) +" TARGET ANGLE:" + str(
            math.degrees(self.target_angle)))

        # if(math.fabs(math.degrees(self.target_angle)) > 180 or math.fabs(math.degrees(behavior.rotate)) > 90 ):
        #     print("rotate????",str(behavior),behavior.rotate)
        #     time.sleep(5)
        #     exit()
        # if self.speed == behavior.speed:
        #     return
        self.move(behavior.speed, behavior.rotate)

    def updateAll(self):
        # update all behaviors in order of priority
        worked = 0
        for behavior in self.behaviors:
            behavior.flag = False
            # if the behavior fired, return it immediately
            # if str(behavior) == "FixPositionBehavior":
            #     behavior.update()
            #     if behavior.flag:
            #         self.speed = behavior.speed
            #         # self.LAST_MOVE = behavior.rotate
            #         return behavior
            # el
            if str(behavior) == "AvoidBehavior" and sum([1 for key,value in self.data_ultrasonic.items() if value is not None]) == len(self.data_ultrasonic.keys()):
                worked += 1
                behavior.update()
                # self.data_radar=None
                # self.data_ultrasonic_r = self.data_ultrasonic_l = None # invalidate data
                if behavior.flag:
                    self.LAST_MOVE = -behavior.rotate
                    self.target_angle = self.normalize_angle(self.yaw + behavior.rotate) #if self.yaw>0 else self.yaw - behavior.rotate)
                    # if -math.pi > self.target_angle or self.target_angle > math.pi:
                    #     self.target_angle = - (1 if self.target_angle > 0 else -1 if self.target_angle < 0 else 0) * ((2 * math.pi) - self.target_angle)
                    #     print("Happen 2")
                    # if self.behaviors[2].has_line: # if has_line: go around object
                    #     self.LAST_MOVE = -1 if behavior.rotate < 0 else 1
                    # else:
                    #     self.LAST_MOVE = -1 if behavior.rotate > 0 else 1

                    self.speed = behavior.speed
                    return behavior
            elif str(behavior) == "WanderBehavior" and self.data_camera is not None:
                worked += 1
                behavior.update()
                # self.data_camera = None # invalidate data
                if behavior.flag:
                    self.LAST_MOVE = behavior.rotate
                    # self.target_angle = (self.yaw + behavior.rotate)  # target angle = current.angle + rotate.angle
                    self.target_angle = self.normalize_angle(self.yaw + behavior.rotate)
                    # if self.target_angle != 0:
                    #     print(target_angle)
                    #     exit()

                    self.speed = behavior.speed
                    return behavior
            elif worked == 2: # camera + avoid
                behavior.update()
                if behavior.flag:
                    self.speed = behavior.speed
                    # self.LAST_MOVE = behavior.rotate
                    return behavior
            # if behavior.flag:
        # if none fired, return the last (lowest-priority) behavior:
        return None #self.behaviors[-1]

    def main(self):
        # rospy.spin()
        count = 0
        steps = 1
        while not rospy.is_shutdown():
            # for i in range(len(self.behaviors)):
            for i in range(steps):
                self.step()
                self.rate.sleep()
            
            self.move(0,0)
            try:
                steps = int(input("How many steps?"))
            except:                
                pass #steps = 3 # default
            # break
        self.move(0,0)

# -----------------------------------------------------------------------
# base class for behaviors

class SubsumptionBehavior:
    def __init__(self):
        self.TOLERANCE = .20

        FACTOR = 5
        self.NO_FORWARD = 0
        self.SLOW_FORWARD = 0.1
        self.MED_FORWARD = 0.5 
        self.FULL_FORWARD = 1.0  # 1@1Hz travels for approx 0.132241 meters

        self.NO_TURN = 0

        self.MED_LEFT = 0.5 
        self.HARD_LEFT = 1.0

        self.MED_RIGHT = -0.5
        self.HARD_RIGHT = -1.0

        self.speed = 0
        self.rotate = 0
        self.has_line = False
        self.flag = False
        # this will be set later when the behavior is created
        self.robot = None

    def __repr__(self):
        return self.__class__.__name__

    def requestMove(self, speed, rotate):
        self.speed = speed
        self.rotate = rotate
        self.flag = True


# -----------------------------------------------------------------------


class RestBehavior(SubsumptionBehavior):
    def update(self, _):
        self.requestMove(0, 0)


class AvoidBehavior(SubsumptionBehavior):
    AVOIDED = False
    flip = 0
    TURN = 0.3
    each_region_degrees = 10 # 8.62692656 degrees is the minimum to find a passage (if exists) for our robot (30x20 approximately) to pass through
    # regions = 180 /each_region_degrees # laser has 180 degrees radius
    # step = each_region_degrees/(180/720) # beams in each region
    regions = 5

    def update(self):  # if True stick on the obstacle,else just avoid it
        # case = 0b00 # [0,1,2,3] # 0 => we got 0 data, 1 => we got only radar data, 2 => we got only ultrasonic data, 3 => we got both
        # print("case",case)
        # if sum([1 for key,value in self.robot.data_ultrasonic.items() if value is not None]) == len(self.robot.data_ultrasonic.keys()):
        #     print("Ultra data ready", case)

        sensors = {}
        for key,value in self.robot.data_ultrasonic.items():
            sensors[key] = value.range if value.range < .7 else .7
            self.robot.data_ultrasonic[key] = None    

        if min(value for key,value in sensors.items() if key[:5]=="front")< self.TOLERANCE or min(value for key,value in sensors.items() if key[:5]!="front")< self.TOLERANCE/2:
            a0 = math.pi/len(sensors.keys())  # * 2 to make it more quick
            if len(sensors.keys()) % 2 == 0:  # if we have even/odd number of sensors our calculation changes
                ai = [int(i) * a0 for i in range(-len(sensors.keys()) / 2, (len(sensors.keys()) / 2) + 1) if i != 0]
            else:
                ai = [int(i+1) * a0 for i in range(-len(sensors.keys()) // 2, (len(sensors.keys()) // 2)+1)]
            Di = [value for key,value in sensors.items()]
            

            rotate = -sum([x * y for x, y in zip(ai, Di)]) / sum([d for d in Di])
            speed = 0
            # the turning angle
            if -.1 < rotate < .1:
                if self.flip == 0:
                    rotate *= random.choice([-1, 1])
                    self.flip += rotate
                else:
                    if self.flip > 0:
                        self.flip = (self.flip + 1) % 10
                    else:
                        self.flip = (self.flip - 1) % -10
                    rotate = -1 if self.flip < 0 else 1
                    # speed = 0.05
            self.requestMove(speed,rotate)

class SearchBehavior(SubsumptionBehavior):
    RANDOM_SIDE = 0
    RANDOM_MOVE = 0

    LAST_MOVE = random.choice([-1, 1])

    def update(self):
        print("SearchBehavior", self.robot.LAST_MOVE)
        self.requestMove(0.5, self.robot.LAST_MOVE)

class WanderBehavior(SubsumptionBehavior):
    NO_ERROR = math.radians(20)

    low_hsv = [40,10,95]
    high_hsv = [110,75,245]
    line_color=[0,255,0] # BGR
    symbol_color=[0,0,255]
    eucl_dist_line = 200
    color_mode = "euclidean"
    detector = detect_mod.Detector(contour_threshold=2000,cut_top_pixels=0, eps_line=.01, eps_symbol=.001, line_width=400, low_hsv=low_hsv, high_hsv=high_hsv, line_color=line_color, symbol_color=symbol_color, eucl_dist_line=eucl_dist_line,
                     eucl_dist_symbol=eucl_dist_line, color_mode=color_mode, resize_width=150, resize_height=150, mode="detect", debug=True)

    # def pyshow(self, image):
    #     surface = pygame.display.set_mode(image.shape[-2::-1])
    #     pygimage = pygame.image.frombuffer(cv2.cvtColor(image,
    #                                                     cv2.COLOR_BGR2RGB),
    #                                        image.shape[-2::-1], 'RGB')
    #     surface.blit(pygimage, (0, 0))
    #     pygame.display.flip()  # update the dis
    prev_frame_t = rospy.Time(0)
    new_frame_t = rospy.Time(0)
    def update(self):
        rospy.loginfo("Proccessing image")
        self.new_frame_t = rospy.Time.now()
        fps = 1/(self.new_frame_t-self.prev_frame_t).to_sec()
        self.prev_frame_t = self.new_frame_t
        print("fps",fps)
        # self.robot.data_camera = self.robot.data_camera[-int(self.robot.data_camera.shape[0]//2):,:,:]
        # frame = frame[-100:,:,]
        # self.robot.data_camera = cv2.cvtColor(self.robot.data_camera, cv2.COLOR_YUV2BGR_YUYV)
        # self.robot.data_camera = cv2.cvtColor(self.robot.data_camera, cv2.COLOR_RGB2BGR)
        self.has_line, output, angle, follow_symbol = self.detector.main(self.robot.data_camera)
        # return
        # angle is already in radiants
        if len(output) == 2:
            h,w,_ = output[0].shape
            self.robot.pub_image.publish(self.robot.bridge.cv2_to_imgmsg(output[0][:,w//2:], encoding="bgr8"))
        else:
            self.robot.pub_image.publish(self.robot.bridge.cv2_to_imgmsg(output, encoding="bgr8"))
        if angle < 0:
            angle = -(math.pi / 2) - angle
        else:
            angle = (math.pi / 2) - angle
        # angle *= -1
        if angle > math.pi or angle < -math.pi:
            print("Shouldn't be possible. Please check") # just to check if it ever happens
            exit()
        if self.has_line:
            print("Found Line", angle)
            # cv2.imshow("img",output[0])
            # self.pyshow(output[0])
            if math.fabs(angle) > self.NO_ERROR:
                if math.fabs(angle) > math.radians(45):
                    self.requestMove(self.SLOW_FORWARD,angle)
                else:
                    self.requestMove(self.MED_FORWARD, angle)
            else:
                self.requestMove(self.FULL_FORWARD, 0)
        else:
            print("Dont see any line")
        self.robot.data_camera = None # invalidate picture proccessed
        

        # self.rate = rospy.Rate(int(fps) if int(fps)!=0 else 1)

class FixPositionBehavior(SubsumptionBehavior):
    kP = .5
    ERROR_OFFSET = math.radians(10)  # 10 degrees error_offset

    def update(self):
        # if self.robot.pose == None:
        #     return
        yaw, target_angle = self.robot.yaw, self.robot.target_angle

        if -math.pi > target_angle or target_angle > math.pi:
            print("Still possible. Error?")
            time.sleep(5)
            exit()

        if math.fabs(target_angle - yaw) > self.ERROR_OFFSET:
            self.requestMove(round(self.robot.speed*0.9,2), self.kP*(target_angle-yaw))
            # self.requestMove(round(self.robot.speed,2),.3 if (target_angle - yaw)>0 else -.3)


if __name__ == '__main__':
    print("try")
    rospy.init_node('LefosNode', log_level=rospy.INFO)
    rospy.loginfo("%s: starting LefosNode node", rospy.get_name())
    master = SubsumptionBrain(camera_topic="/usb_cam_node/image_raw", imu_topic="/imu", cmd_vel_topic="/cmd_vel")
    # master = SubsumptionBrain(camera_topic=rospy.get_param("/camera_topic"), radar_topic=rospy.get_param("/radar_topic"), imu_topic=rospy.get_param("/imu_topic"), cmd_vel_topic=rospy.get_param("/cmd_vel_topic"))
    # master.setup()
    rospy.loginfo("Successfully started LefosNode")
    master.main()