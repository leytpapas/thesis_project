import sys
import os
import smbus
from time import time,sleep          #import
class Gyro_driver(object):
    """docstring for Gyro_driver"""
    def __init__(self):
        super(Gyro_driver, self).__init__()
        #some MPU6050 Registers and their Address
        self.PWR_MGMT_1   = 0x6B
        self.SMPLRT_DIV   = 0x19
        self.CONFIG       = 0x1A
        self.GYRO_CONFIG  = 0x1B
        self.INT_ENABLE   = 0x38
        self.ACCEL_XOUT_H = 0x3B
        self.ACCEL_YOUT_H = 0x3D
        self.ACCEL_ZOUT_H = 0x3F
        self.GYRO_XOUT_H  = 0x43
        self.GYRO_YOUT_H  = 0x45
        self.GYRO_ZOUT_H  = 0x47
        self.TEMP = 0x41
        self.bus = smbus.SMBus(1)     # or bus = smbus.SMBus(0) for older version boards
        self.Device_Address = 0x68   # MPU6050 device address

        self.MPU_Init()
        self.time_present = time()
        self.time_past = time()
        self.accel_deadzone = 8
        self.gyro_deadzone = 1
        self.accel_x_offset = self.accel_y_offset = self.accel_z_offset = 0
        self.rot_x_offset = self.rot_y_offset = self.rot_z_offset = 0
        self.gyro_x_past = self.gyro_y_past = self.gyro_z_past = 0
        self.gyro_x_present = self.gyro_y_present = self.gyro_z_present = 0
        self.gyro_x_calli = self.gyro_y_calli = self.gyro_z_calli = 0
        self.rot_x = self.rot_y = self.rot_z = 0
        self.angle_x = self.angle_y = self.angle_z = 0
        self.callibrate(100)
        self.Q = []
        self.G = []

    def MPU_Init(self):
        #write to sample rate register
        self.bus.write_byte_data(self.Device_Address, self.SMPLRT_DIV, 7)

        #Write to power management register
        self.bus.write_byte_data(self.Device_Address, self.PWR_MGMT_1, 1)

        #Write to Configuration register
        self.bus.write_byte_data(self.Device_Address, self.CONFIG, 0)

        #Write to Gyro configuration register
        self.bus.write_byte_data(self.Device_Address, self.GYRO_CONFIG, 24)

        #Write to interrupt enable register
        self.bus.write_byte_data(self.Device_Address, self.INT_ENABLE, 1)

    def read_raw_data(self, addr):
        #Accelero and Gyro value are 16-bit
        high = self.bus.read_byte_data(self.Device_Address, addr)
        low = self.bus.read_byte_data(self.Device_Address, addr+1)
        #concatenate higher and lower value
        value = ((high << 8) | low)

        #to get signed value from mpu6050
        if value > 32768:
            value = value - 65536
        return value

    # Disable
    def blockPrint(self):
        sys.stdout = open(os.devnull, 'w')
    # Restore
    def enablePrint(self):
        sys.stdout = sys.__stdout__

    def respawn(self):
        while True:
            print("----------------")
            for device in range(128):
                try:
                    self.bus.read_byte(device)
                    if device == self.Device_Address:
                        try:
                            print("Respawning", hex(self.Device_Address))
                            self.MPU_Init()
                            return
                        except OSError:
                            print("Error trying to init or device",hex(self.Device_Address)," not available. Retrying in a 2 seconds...")
                            sleep(2)
                            break
                except: # exception if read_byte fails
                    pass
            sleep(1)

    def callibrate(self, times):
        Ax, Ay, Az, rot_x, rot_y, rot_z, tempC, angle = self.fetch_data()
        ax_offset =- Ax/8
        ay_offset =- Ay/8
        az_offset = (16384-Az)/8

        gx_offset =- rot_x/4
        gy_offset =- rot_y/4
        gz_offset =- rot_z/4
        print("Starting Calibration. Please wait a few seconds")
        while True:
            ready = 0
            self.accel_x_offset = ax_offset
            self.accel_y_offset = ay_offset
            self.accel_z_offset = az_offset

            self.rot_x_offset = gx_offset
            self.rot_y_offset = gy_offset
            self.rot_z_offset = gz_offset

            self.enablePrint()
            Ax, Ay, Az, self.gyro_x_present,self.gyro_y_present,self.gyro_z_present, tempC, angle = self.fetch_data()
            self.blockPrint()
            # print ("rot_x=%.2f" %self.gyro_x_present, u'\u00b0'+ "/s", "\trot_y=%.2f" %self.gyro_y_present, u'\u00b0'+ "/s", "\trot_z=%.2f" %self.gyro_z_present, u'\u00b0'+ "/s", "\tAx=%.2f g" %Ax, "\tAy=%.2f g" %Ay, "\tAz=%.2f g" %Az)
            if abs(Ax)<=self.accel_deadzone:
                ready+=1
            else:
                ax_offset = ax_offset - Ax/self.accel_deadzone
            if abs(Ay)<=self.accel_deadzone:
                ready+=1
            else:
                ay_offset = ay_offset - Ay/self.accel_deadzone
            if abs(16384 - Az)<=self.accel_deadzone:
                ready+=1
            else:
                az_offset = az_offset - (16384 - Az)/self.accel_deadzone
            if abs(self.gyro_x_present) <= self.gyro_deadzone:
                ready += 1
            else:
                gx_offset=gx_offset - self.self.gyro_x_present/(self.gyro_deadzone+1)

            if abs(self.gyro_z_present) <= self.gyro_deadzone:
                ready += 1
            else:
                gz_offset = gz_offset - self.self.gyro_z_present/(self.gyro_deadzone+1)

            if abs(self.gyro_z_present) <= self.gyro_deadzone:
                ready += 1
            else:
                gz_offset = gz_offset - self.self.gyro_z_present/(self.gyro_deadzone+1)

            if ready == 6: break

            # time.sleep(0.002)
            # sleep(.5)
        # self.gyro_x_calli = self.gyro_x_calli // times
        # self.gyro_y_calli = self.gyro_y_calli // times
        # self.gyro_z_calli = self.gyro_z_calli // times
        print("Calibration ended")
        

    def get_accel(self):
        # Read Accelerometer raw value
        acc_x = self.read_raw_data(self.ACCEL_XOUT_H)
        acc_y = self.read_raw_data(self.ACCEL_YOUT_H)
        acc_z = self.read_raw_data(self.ACCEL_ZOUT_H)

        # Full scale range +/- 250 degree/C as per sensitivity scale factor
        Ax = acc_x / 16384.0
        Ay = acc_y / 16384.0
        Az = acc_z / 16384.0
        return Ax, Ay, Az

    def get_gyro(self):

        self.gyro_x_past = self.gyro_x_present
        self.gyro_y_past = self.gyro_y_present
        self.gyro_z_past = self.gyro_z_present

        # Read Gyroscope raw value
        self.gyro_x_present = self.read_raw_data(self.GYRO_XOUT_H)
        self.gyro_y_present = self.read_raw_data(self.GYRO_YOUT_H)
        self.gyro_z_present = self.read_raw_data(self.GYRO_ZOUT_H)
        
        # angular_velocity
        self.rot_x = self.gyro_x_present / 131.0
        self.rot_y = self.gyro_y_present / 131.0
        self.rot_z = self.gyro_z_present / 131.0

        return self.rot_x, self.rot_y, self.rot_z

    def get_temp(self):
        tempRow = self.read_raw_data(self.TEMP)
        tempC = (tempRow / 340.0) + 36.53
        tempC = "%.2f" % tempC
        # print("Temp: ", tempC)
        return tempC

    # angular_displacement
    def get_angle(self):
        self.angle_x = self.angle_x + ((self.time_present - self.time_past) * (self.gyro_x_present + self.gyro_x_past - 2 * self.gyro_z_calli)) * 0.00000382
        self.angle_y = self.angle_y + ((self.time_present - self.time_past) * (self.gyro_x_present + self.gyro_y_past - 2 * self.gyro_z_calli)) * 0.00000382
        self.angle_z = self.angle_z + ((self.time_present - self.time_past) * (self.gyro_x_present + self.gyro_z_past - 2 * self.gyro_z_calli)) * 0.00000382
        return self.angle_x, self.angle_y, self.angle_z

    def fetch_data(self):
        Ax = Ay = Az = 0
        rot_x = rot_y = rot_z = 0
        tempC = 0
        angle = 0
        times = 100
        for i in range(times):
            sleep(0.002)
            while True:
                try:
                    ax, ay, az = self.get_accel()
                    Ax+=ax
                    Ay+=ay
                    Az+=az
                    r_x, r_y, r_z = self.get_gyro()
                    rot_x+=r_x
                    rot_y+=r_y
                    rot_z+=r_z
                    tempC += self.get_temp()
                    angle = self.get_angle()
                    break
                except:
                    print("respawning")
                    self.respawn()
                    self.bus = smbus.SMBus(1) 
                    sleep(.5)
        return Ax/times, Ay/times, Az/times, self.rot_x/times,self.rot_y/times,self.rot_z/times, tempC/times, angle/times

    def loop(self):
        print (" Reading Data of Gyroscope and Accelerometer")
        count = 0
        while True:
            Ax, Ay, Az, self.gyro_x_present,self.gyro_y_present,self.gyro_z_present, tempC, angle = self.fetch_data()    
            print("Temp = %s" %tempC)
            print ("rot_x=%.2f" %self.rot_x, u'\u00b0'+ "/s", "\trot_y=%.2f" %self.rot_y, u'\u00b0'+ "/s", "\trot_z=%.2f" %self.rot_z, u'\u00b0'+ "/s", "\tAx=%.2f g" %Ax, "\tAy=%.2f g" %Ay, "\tAz=%.2f g" %Az)
            count+=1

if __name__ == '__main__':
    driver = Gyro_driver()
    driver.loop()