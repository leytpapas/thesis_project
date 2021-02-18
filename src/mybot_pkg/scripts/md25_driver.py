#!/usr/bin/env python3
from smbus import SMBus
import time
import numpy as np

class MD25_driver(object):
    """docstring for MD25_driver"""

    def __init__(self):
        super(MD25_driver, self).__init__()
        # REGISTERS
        self.mode = 0
        self.REG_SPEED1 = 0x00  # R/W | Motor1 speed (mode 0,1) or speed (mode 2,3)
        self.REG_SPEED2 = 0x01  # R/W | Motor2 speed (mode 0,1) or turn (mode 2,3)
        self.REG_ENC1A = 0x02  # R | Encoder 1 position, 1st byte (highest), capture count when read
        self.REG_ENC1B = 0x03  # R | Encoder 1 position, 2nd byte
        self.REG_ENC1C = 0x04  # R | Encoder 1 position, 3rd byte
        self.REG_ENC1D = 0x05  # R | Encoder 1 position, 4th (lowest byte)
        self.REG_ENC2A = 0x06  # R | Encoder 2 position, 1st byte (highest), capture count when read
        self.REG_ENC2B = 0x07  # R | Encoder 2 position, 2nd byte
        self.REG_ENC2C = 0x08  # R | Encoder 2 position, 3rd byte
        self.REG_ENC2D = 0x09  # R | Encoder 2 position, 4th byte (lowest byte)
        self.REG_BATTERY_VOLTS = 0x0A  # R | The supply battery voltage
        self.REG_MOTOR1_CURRENT = 0x0B  # R | The current through motor 1
        self.REG_MOTOR2_CURRENT = 0x0C  # R | The current through motor 2
        self.REG_SOFTWARE_REVISION = 0x0D  # R | Software Revision Number
        self.REG_ACCELERATION_RATE = 0x0E  # R/W | Optional Acceleration register
        self.REG_MODE = 0x0F  # R/W | Mode of operation (see below)
        self.REG_COMMAND = 0x10  # R/W | Used for reset of encoder counts and module address changes

        # MODES
        self.MODE_0 = 0x00  # The meaning of the speed registers is literal speeds in the range of 0 (Full Reverse), 128 (Stop), 255 (Full Forward) (Default Setting).
        self.MODE_1 = 0x01  # The meaning of the speed registers is literal speeds in the range of -128 (Full Reverse), 0 (Stop), 127 (Full Forward).
        self.MODE_2 = 0x02  # Speed1 control both motors speed, and speed2 becomes the turn value. Data is in the range of 0 (Full Reverse), 128 (Stop), 255 (Full Forward).
        self.MODE_3 = 0x03  # Speed1 control both motors speed, and speed2 becomes the turn value. Data is in the range of -128 (Full Reverse), 0 (Stop), 127 (Full Forward).

        # COMMANDS
        self.CMD_ENCODER_RESET = 0x20  # Resets the encoder registers to zero
        self.CMD_AUTO_SPEED_DISABLE = 0x30  # Disables automatic speed regulation
        self.CMD_AUTO_SPEED_ENABLE = 0x31  # Enables automatic speed regulation (default)
        self.CMD_TIMEOUT_DISABLE = 0x32  # Disables 2 second timeout of motors (Version 2 onwards only)
        self.CMD_TIMEOUT_ENABLE = 0x33  # Enables 2 second timeout of motors when no I2C comms (default) (Version 2 onwards only)
        self.CMD_CHANGE_I2C_ADDR_1 = 0xA0  # 1st in sequence to change I2C address
        self.CMD_CHANGE_I2C_ADDR_2 = 0xAA  # 2nd in sequence to change I2C address
        self.CMD_CHANGE_I2C_ADDR_3 = 0xA5  # 3rd in sequence to change I2C address

        # I2C ADDRESS
        self.I2C_START_ADDR = 0xB0  # The start address of valid I2C addresses. LSB indicates R/W, so add 2 to get next valid. Last valid is 0xBE.
        self.I2C_WRITE_BIT = 0x00  # Add this to I2C address to perform a write.
        self.I2C_READ_BIT = 0x01  # Add this to I2C address to perform a read.

        self.MD25ADDRESS = 0x58  # Address of the MD25
        self.i2cbus = SMBus(1)  # Create a new I2C bus /dev/i2c-dev1
        self.i2caddress = self.MD25ADDRESS  # Address of MCP23017 device

        self.MIN = -128 # for mode_2,4
        self.MAX = 127 # for mode_2,4
        self.encoder_min = 0
        self.encoder_max = 4294967295
        self.encoders_reset()
        print("MD25_driver initialized")

    def motor_stop(self):
        self.i2cbus.write_byte_data(self.i2caddress, self.REG_SPEED1, 0)
        self.i2cbus.write_byte_data(self.i2caddress, self.REG_SPEED2, 0)

    def mode_set(self, mode):
        if 0 > mode or mode > 3: return False
        self.i2cbus.write_byte_data(self.i2caddress, self.REG_MODE, mode)
        data = self.mode_get()
        if data != mode: return False
        self.mode = mode
        return True
        
    def get_encoder_min(self):
        return self.encoder_min
    
    def get_encoder_max(self):
        return self.encoder_max

    def mode_get(self):
        return self.i2cbus.read_byte_data(self.i2caddress, self.REG_MODE)

    def encoders_reset(self):
        self.i2cbus.write_byte_data(self.i2caddress, self.REG_COMMAND, self.CMD_ENCODER_RESET)

    def auto_speed_set(self, enabled):
        if enabled:
            self.i2cbus.write_byte_data(self.i2caddress, self.REG_COMMAND, self.CMD_AUTO_SPEED_ENABLE)
        else:
            self.i2cbus.write_byte_data(self.i2caddress, self.REG_COMMAND, self.CMD_AUTO_SPEED_DISABLE)

    def timeout_set(self, enabled):
        if enabled:
            self.i2cbus.write_byte_data(self.i2caddress, self.REG_COMMAND, self.CMD_TIMEOUT_ENABLE)
        else:
            self.i2cbus.write_byte_data(self.i2caddress, self.REG_COMMAND, self.CMD_TIMEOUT_DISABLE)

    def i2c_addr_set(self, address):
        if self.I2C_START_ADDR > address or address < self.I2C_START_ADDR + 0x0F or address % 2 == 1:
            return False
        self.i2cbus.write_i2c_block_data(self.i2caddress, self.REG_COMMAND,
                                         [self.CMD_CHANGE_I2C_ADDR_1, self.CMD_CHANGE_I2C_ADDR_2,
                                          self.CMD_CHANGE_I2C_ADDR_3, address])
        self.i2caddress = address
        return True

    def speed1_set(self, speed):
        if self.mode == 0:
            if speed < 0 or speed > 255: return False
            speed = abs(speed)
        elif self.mode == 1:
            if speed < -128 or speed > 127: return False
        else:
            return True
        self.i2cbus.write_byte_data(self.i2caddress, self.REG_SPEED1, speed)
        return True

    def speed1_get(self):
        if self.mode < 0 or self.mode > 1: return False
        data = self.i2cbus.read_byte_data(self.i2caddress, self.REG_SPEED1)
        if self.mode == 0:
            data = abs(data)
        return data

    def speed2_set(self, speed):
        if self.mode == 0:
            if speed < 0 or speed > 255: return False
            speed = abs(speed)
        elif self.mode == 1:
            if speed < -128 or speed > 127: return False
        else:
            return True
        self.i2cbus.write_byte_data(self.i2caddress, self.REG_SPEED2, speed)
        return True

    def speed2_get(self):
        if self.mode < 0 or self.mode > 1: return False
        data = self.i2cbus.read_byte_data(self.i2caddress, self.REG_SPEED2)
        if self.mode == 0:
            data = abs(data)
        return data

    def speed_set(self, speed):
        if self.mode == 2:
            if speed < 0 or speed > 255: return False
        elif self.mode == 3:
            if speed < -128 or speed > 127: return False
        else:
            return True
        self.i2cbus.write_byte_data(self.i2caddress, self.REG_SPEED1, speed)
        return True

    def speed_get(self):
        if self.mode < 2 or self.mode > 3: return False
        return self.i2cbus.read_byte_data(self.i2caddress, self.REG_SPEED1)

    def turn_set(self, turn):
        if self.mode == 2:
            if turn < 0 or turn > 255: return False
        elif self.mode == 3:
            if turn < -128 or turn > 127: return False
        else:
            return True
        self.i2cbus.write_byte_data(self.i2caddress, self.REG_SPEED2, turn)

    def turn_get(self):
        if self.mode < 2 or self.mode > 3: return False
        return self.i2cbus.read_byte_data(self.i2caddress, self.REG_SPEED2)

    def acceleration_set(self, acceleration):
        if acceleration < 0 or acceleration > 255: return False
        acceleration = abs(acceleration)
        self.i2cbus.write_byte_data(self.i2caddress, self.REG_ACCELERATION_RATE, acceleration)
        return True

    def acceleration_get(self):
        return self.i2cbus.read_byte_data(self.i2caddress, self.REG_ACCELERATION_RATE)

    def encoder1_get(self):
        data = self.i2cbus.read_i2c_block_data(self.i2caddress, self.REG_ENC1A, 4)
        return data[0] << 24 | data[1] << 16 | data[2] << 8 | data[3]

    def encoder2_get(self):
        data = self.i2cbus.read_i2c_block_data(self.i2caddress, self.REG_ENC2A, 4)
        return data[0] << 24 | data[1] << 16 | data[2] << 8 | data[3]

    def bat_voltage_get(self):
        return self.i2cbus.read_byte_data(self.i2caddress, self.REG_BATTERY_VOLTS) / 10

    def motor1_current_get(self):
        return self.i2cbus.read_byte_data(self.i2caddress, self.REG_MOTOR1_CURRENT) / 10

    def motor2_current_get(self):
        return self.i2cbus.read_byte_data(self.i2caddress, self.REG_MOTOR2_CURRENT) / 10

    def software_rev_num_get(self):
        return self.i2cbus.read_byte_data(self.i2caddress, self.REG_SOFTWARE_REVISION)

    def normalize(self, value, a, b):
    	return ((b-a)*(value-self.MIN)/(self.MAX-self.MIN)) + a

    def main(self):
        self.mode_set(2) # -128 full reverse 0 stop 127 full forward
        speed = 0
        for i in range(1000):
            # self.speed_set(speed)
            # self.turn_set(128)
            speed = (speed + 30)%255
            # self.turn_set(0)
            print("Encoder 1:", self.encoder1_get())
            print("Encoder 2:", self.encoder2_get())
            print("Volts:", self.bat_voltage_get())
            print("Amp motor 1:", self.motor1_current_get())
            print("Amp motor 2:", self.motor2_current_get())
            time.sleep(.5)
        print("STOP")
        self.motor_stop()


if __name__ == '__main__':
    driver = MD25_driver()
    driver.timeout_set(True)
    driver.main()
    # print(driver.normalize(200,-128,127))