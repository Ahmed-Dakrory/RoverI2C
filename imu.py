# coding: utf-8
## @package FaBo9Axis_GY87
#  This is a library for the FaBo 9AXIS I2C Brick.
#
#  http://fabo.io/202.html
#
#  Released under APACHE LICENSE, VERSION 2.0
#
#  http://www.apache.org/licenses/
#
#  FaBo <info@fabo.io>

import math
import threading
import time
## GY87 Default I2C slave address
SLAVE_ADDRESS        = 0x68
## AK8963 I2C slave address
AK8963_SLAVE_ADDRESS = 0x1E
## Device id
DEVICE_ID            = 0x71

''' MPU-9250 Register Addresses '''
## sample rate driver
SMPLRT_DIV     = 0x19
CONFIG         = 0x1A
GYRO_CONFIG    = 0x1B
ACCEL_CONFIG   = 0x1C
ACCEL_CONFIG_2 = 0x1D
LP_ACCEL_ODR   = 0x1E
WOM_THR        = 0x1F
FIFO_EN        = 0x23
I2C_MST_CTRL   = 0x24
I2C_MST_STATUS = 0x36
INT_PIN_CFG    = 0x37
INT_ENABLE     = 0x38
INT_STATUS     = 0x3A
ACCEL_OUT      = 0x3B
TEMP_OUT       = 0x41
GYRO_OUT       = 0x43

I2C_MST_DELAY_CTRL = 0x67
SIGNAL_PATH_RESET  = 0x68
MOT_DETECT_CTRL    = 0x69
USER_CTRL          = 0x6A
PWR_MGMT_1         = 0x6B
PWR_MGMT_2         = 0x6C
FIFO_R_W           = 0x74
WHO_AM_I           = 0x75

## Gyro Full Scale Select 250dps
GFS_250  = 0x00
## Gyro Full Scale Select 500dps
GFS_500  = 0x01
## Gyro Full Scale Select 1000dps
GFS_1000 = 0x02
## Gyro Full Scale Select 2000dps
GFS_2000 = 0x03
## Accel Full Scale Select 2G
AFS_2G   = 0x00
## Accel Full Scale Select 4G
AFS_4G   = 0x01
## Accel Full Scale Select 8G
AFS_8G   = 0x02
## Accel Full Scale Select 16G
AFS_16G  = 0x03


MagAddress=0x1E
gauss=8.10
declination = (0,0)
# Some class description
class GY87(threading.Thread):

    ## HMC5883l class
    __scales = {
        0.88: [0, 0.73],
        1.30: [1, 0.92],
        1.90: [2, 1.22],
        2.50: [3, 1.52],
        4.00: [4, 2.27],
        4.70: [5, 2.56],
        5.60: [6, 3.03],
        8.10: [7, 4.35],
    }
    ## Constructor
    #  @param [in] address MPU-9250 I2C slave address default:0x68
    def __init__(self,bus):
        super(GY87,self).__init__(name="GY87")
        self.bus=bus
        self.address = SLAVE_ADDRESS
        self.configGY87(GFS_250, AFS_2G)
        self.MagAddress = MagAddress
        self.gyroAngle = 0
        (degrees, minutes) = declination
        self.__declDegrees = degrees
        self.__declMinutes = minutes
        self.__declination = (degrees + minutes / 60) * math.pi / 180

        (reg, self.__scale) = self.__scales[gauss]
        self.bus.write_byte_data(self.MagAddress, 0x00, 0x70) # 8 Average, 15 Hz, normal measurement
        self.bus.write_byte_data(self.MagAddress, 0x01, reg << 5) # Scale
        self.bus.write_byte_data(self.MagAddress, 0x02, 0x00) # Continuous measurement
        self.MpuGyroZandHeadingReading=[0.0,0.0]
        
    def run(self):
        headingMagOld = 0
        while True:
            rate = self.rateGyro()
            headingMag = self.headingMag()
        
            headingNow = 0.6 * headingMag + 0.4 * headingMagOld
            headingMagOld = self.headingMag()
            
            if headingNow>180:
                headingNow -=180
                headingNow = -headingNow
                headingNow +=180
                headingNow = -headingNow
            self.MpuGyroZandHeadingReading[0]=rate
            self.MpuGyroZandHeadingReading[1]=headingNow
            
            
    def declination(self):
        return (self.__declDegrees, self.__declMinutes)

    def twos_complement(self, val, len):
        # Convert twos compliment to integer
        if (val & (1 << len - 1)):
            val = val - (1<<len)
        return val

    def __convert(self, data, offset):
        val = self.twos_complement(data[offset] << 8 | data[offset+1], 16)
        if val == -4096: return None
        return round(val * self.__scale, 4)

    def axes(self):
        data = self.bus.read_i2c_block_data(self.MagAddress, 0x00)
        #print map(hex, data)
        x = self.__convert(data, 3)
        y = self.__convert(data, 7)
        z = self.__convert(data, 5)
        return (x,y,z)

    def headingMag(self):
        (x, y, z) = self.axes()
        headingRad = math.atan2(y, x) + 55 * math.pi /180
        #headingRad += self.__declination

        # Correct for reversed heading
        if headingRad < 0:
            headingRad += 2 * math.pi

        # Check for wrap and compensate
        elif headingRad > 2 * math.pi:
            headingRad -= 2 * math.pi

        # Convert to degrees from radians
        headingDeg = headingRad * 180 / math.pi
        return headingDeg
        
    def rateGyro(self):
       
            
            
        
        
        gyro = self.readGyro()
        self.gyroAngle = gyro['z']/1310*3
        #if self.gyroAngle >360:
        #    self.gyroAngle = self.gyroAngle -360
        #if self.gyroAngle < 0 :
        #    self.gyroAngle = self.gyroAngle + 360
        
        rate =  self.gyroAngle
        #if headingDeg>180:
        #    headingDeg -=180
        #    headingDeg = -headingDeg
        #    headingDeg +=180
        #    headingDeg = -headingDeg

        return rate


    ## Search Device
    #  @param [in] self The object pointer.
    #  @retval true device connected
    #  @retval false device error
    def searchDevice(self):
        who_am_i = self.bus.read_byte_data(self.address, WHO_AM_I)
        if(who_am_i == DEVICE_ID):
            return true
        else:
            return false

    ## Configure MPU-9250
    #  @param [in] self The object pointer.
    #  @param [in] gfs Gyro Full Scale Select(default:GFS_250[+250dps])
    #  @param [in] afs Accel Full Scale Select(default:AFS_2G[2g])
    def configGY87(self, gfs, afs):
        if gfs == GFS_250:
            self.gres = 250.0/32768.0
        elif gfs == GFS_500:
            self.gres = 500.0/32768.0
        elif gfs == GFS_1000:
            self.gres = 1000.0/32768.0
        else:  # gfs == GFS_2000
            self.gres = 2000.0/32768.0

        if afs == AFS_2G:
            self.ares = 2.0/32768.0
        elif afs == AFS_4G:
            self.ares = 4.0/32768.0
        elif afs == AFS_8G:
            self.ares = 8.0/32768.0
        else: # afs == AFS_16G:
            self.ares = 16.0/32768.0

        # sleep off
        self.bus.write_byte_data(self.address, PWR_MGMT_1, 0x00)
        time.sleep(0.1)
        # auto select clock source
        self.bus.write_byte_data(self.address, PWR_MGMT_1, 0x01)
        time.sleep(0.1)
        # DLPF_CFG
        self.bus.write_byte_data(self.address, CONFIG, 0x03)
        # sample rate divider
        self.bus.write_byte_data(self.address, SMPLRT_DIV, 0x04)
        # gyro full scale select
        self.bus.write_byte_data(self.address, GYRO_CONFIG, gfs << 3)
        # accel full scale select
        self.bus.write_byte_data(self.address, ACCEL_CONFIG, afs << 3)
        # A_DLPFCFG
        self.bus.write_byte_data(self.address, ACCEL_CONFIG_2, 0x03)
        # BYPASS_EN
        self.bus.write_byte_data(self.address, INT_PIN_CFG, 0x02)
        time.sleep(0.1)

  
    ## brief Check data ready
    #  @param [in] self The object pointer.
    #  @retval true data is ready
    #  @retval false data is not ready
    def checkDataReady(self):
        drdy = self.bus.read_byte_data(self.address, INT_STATUS)
        if drdy & 0x01:
            return True
        else:
            return False

    ## Read accelerometer
    #  @param [in] self The object pointer.
    #  @retval x : x-axis data
    #  @retval y : y-axis data
    #  @retval z : z-axis data
    def readAccel(self):
        data = self.bus.read_i2c_block_data(self.address, ACCEL_OUT, 6)
        x = self.dataConv(data[1], data[0])
        y = self.dataConv(data[3], data[2])
        z = self.dataConv(data[5], data[4])

        x = round(x*self.ares, 3)
        y = round(y*self.ares, 3)
        z = round(z*self.ares, 3)

        return {"x":x, "y":y, "z":z}

    ## Read gyro
    #  @param [in] self The object pointer.
    #  @retval x : x-gyro data
    #  @retval y : y-gyro data
    #  @retval z : z-gyro data
    def readGyro(self):
        data = self.bus.read_i2c_block_data(self.address, GYRO_OUT, 6)

        x = self.dataConv(data[1], data[0])
        y = self.dataConv(data[3], data[2])
        z = self.dataConv(data[5], data[4])

        x = round(x*self.gres, 3)
        y = round(y*self.gres, 3)
        z = round(z*self.gres, 3)

        return {"x":x, "y":y, "z":z}

    ## Read temperature
    #  @param [out] temperature temperature(degrees C)
    def readTemperature(self):
        data = self.bus.read_i2c_block_data(self.address, TEMP_OUT, 2)
        temp = self.dataConv(data[1], data[0])

        temp = round((temp / 333.87 + 21.0), 3)
        return temp

    ## Data Convert
    # @param [in] self The object pointer.
    # @param [in] data1 LSB
    # @param [in] data2 MSB
    # @retval Value MSB+LSB(int 16bit)
    def dataConv(self, data1, data2):
        value = data1 | (data2 << 8)
        if(value & (1 << 16 - 1)):
            value -= (1<<16)
        return value

