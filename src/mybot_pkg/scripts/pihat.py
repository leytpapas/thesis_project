from time import sleep
from adafruit_servokit import ServoKit
from ultrasonic_sensors_driver import Ultrasonic_driver
kit = ServoKit(channels=8) #we dont need 16

#BCM
gpio_list_trigger = [14,18]
gpio_list_echo = [4,17]
#BOARD
# gpio_list_trigger = [19,21]
# gpio_list_echo = [24,26]

positions = ["r_left","r_right"]
sensors = Ultrasonic_driver(gpio_list_trigger, gpio_list_echo, positions)
for t in range(10):
	sleep(.5)
	for i in range(180):
	    kit.servo[0].angle = i
	    # print(sensors.distance_all())
	    # sleep(.01)
	sleep(.5)
	for i in range(180,0,-1):
	    kit.servo[0].angle = i
	    # print(sensors.distance_all())
	    # sleep(.01)

kit.servo[0].angle = 0
