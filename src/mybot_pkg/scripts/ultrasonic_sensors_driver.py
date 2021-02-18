#Libraries
import RPi.GPIO as GPIO
import time
# import rospy
# from sensor_msgs.msg import Range

class Ultrasonic_driver(object):
    """docstring for Ultrasonic_driver"""
    def __init__(self, gpio_list_trigger, gpio_list_echo, positions, blocking=False, debug=False):
        super(Ultrasonic_driver, self).__init__()

        #GPIO Mode (BOARD / BCM)
        # GPIO.setmode(GPIO.BOARD)
        GPIO.setmode(GPIO.BCM)
        self.gpio_list_trigger = gpio_list_trigger
        self.gpio_list_echo = gpio_list_echo
        self.blocking = blocking
        self.debug = debug
        for direction_in,direction_out in zip(self.gpio_list_echo, self.gpio_list_trigger):
            #set GPIO direction (IN / OUT)
            if self.debug:
                print("Trigger",direction_out)
                print("Echo",direction_in)
            GPIO.setup(direction_out, GPIO.OUT)
            GPIO.output(direction_out, False)
            GPIO.setup(direction_in, GPIO.IN)
            time.sleep(.5)
        self.positions = positions
        self.sampling = [[]*len(self.positions)]
        self.temp = None
        self.speed_of_sound = 34300
        if len(self.positions) != len(self.gpio_list_echo) != len(self.gpio_list_trigger):
            raise Exception("Input args should be same length each")

        if self.debug:
            print("Ultrasonic_sensor_driver initialized")

    def set_temp(self, temp):
        self.temp = temp

    def get_positions(self):
        return self.positions
    
    def get_gpio_echo(self):
        return self.gpio_list_echo
    
    def get_gpio_trigger(self):
        return self.gpio_list_trigger

    def distance_by_name(self, name):
        for i,n in enumerate(self.positions):
            if n == name:
                name, distance = self.distance_by_index(i)
                return [name, distance]
        return []

    def distance_by_index(self, i):
        # set Trigger to HIGH
        GPIO.output(self.gpio_list_trigger[i], True)
     
        # set Trigger after 0.01ms to LOW
        time.sleep(0.00001)
        GPIO.output(self.gpio_list_trigger[i], False)
     
        StartTime = time.time()
        StopTime = time.time()
        # save StartTime
        while GPIO.input(self.gpio_list_echo[i])==0 : #0.0234:
            StartTime = time.time()
            if not self.blocking and StartTime - StopTime > 0.0234:
                return [self.positions[i],4.0]

        # save time of arrival
        while GPIO.input(self.gpio_list_echo[i])==1 and (not self.blocking and StopTime - StartTime < 0.0234): #0.0234
            StopTime = time.time()
     
        # time difference between start and arrival
        TimeElapsed = StopTime - StartTime
        # multiply with the sonic speed (34300 cm/s)
        # and divide by 2, because there and back
        if self.temp is None:
            distance = (TimeElapsed * self.speed_of_sound) / 2
        else:
            distance = (TimeElapsed * (331.3 + 0.606 * self.temp))/2
            self.temp = None
        distance = round(distance/100, 2)
        # if distance > 4:
        # 	distance=.01 # indicating wrong measurement. valid measurements are [0.02, 4.00] 
        if self.debug:
            print("Returning",self.positions[i], distance)
        return [self.positions[i], distance]

    def distance_all(self):
        results = []
        for i in range(len(self.positions)):
             results.append(self.distance_by_index(i))
        return results
 
if __name__ == '__main__':
    driver = Ultrasonic_driver([16, 20, 21],[13, 19, 26],["left","front","right"],debug=True)

    try:
        while True:
            print ("Measured Distance:\n", driver.distance_all())
 
        # Reset by pressing CTRL + C
    except KeyboardInterrupt:
        print("Measurement stopped by User")
        GPIO.cleanup()