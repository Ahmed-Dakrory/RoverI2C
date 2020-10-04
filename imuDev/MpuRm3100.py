#Connections
#MPU6050 - Raspberry pi
#VCC - 5V  (2 or 4 Board)
#GND - GND (6 - Board)
#SCL - SCL (5 - Board)
#SDA - SDA (3 - Board)


from .Kalman import KalmanAngle
import smbus			#import SMBus module of I2C
import time
import math

kalmanX = KalmanAngle()
kalmanY = KalmanAngle()

radToDeg = 57.2957786
kalAngleX = 0
kalAngleY = 0
#some MPU6050 Registers and their Address
PWR_MGMT_1   = 0x6B
SMPLRT_DIV   = 0x19
CONFIG       = 0x1A
GYRO_CONFIG  = 0x1B
INT_ENABLE   = 0x38
ACCEL_XOUT_H = 0x3B
ACCEL_YOUT_H = 0x3D
ACCEL_ZOUT_H = 0x3F
GYRO_XOUT_H  = 0x43
GYRO_YOUT_H  = 0x45
GYRO_ZOUT_H  = 0x47
DeviceAddress = 0x68   # MPU6050 device address

from .rm3100  import RM3100



import threading

#Read the gyro and acceleromater values from MPU6050
def MPU_Init(bus):
	#write to sample rate register
	bus.write_byte_data(DeviceAddress, SMPLRT_DIV, 7)

	#Write to power management register
	bus.write_byte_data(DeviceAddress, PWR_MGMT_1, 1)

	#Write to Configuration register
	#Setting DLPF (last three bit of 0X1A to 6 i.e '110' It removes the noise due to vibration.) https://ulrichbuschbaum.wordpress.com/2015/01/18/using-the-mpu6050s-dlpf/
	bus.write_byte_data(DeviceAddress, CONFIG, int('0000110',2))

	#Write to Gyro configuration register
	bus.write_byte_data(DeviceAddress, GYRO_CONFIG, 24)

	#Write to interrupt enable register
	bus.write_byte_data(DeviceAddress, INT_ENABLE, 1)


def read_raw_data(bus,addr):
	#Accelero and Gyro value are 16-bit
    high = bus.read_byte_data(DeviceAddress, addr)
    low = bus.read_byte_data(DeviceAddress, addr+1)

    #concatenate higher and lower value
    value = ((high << 8) | low)

    #to get signed value from mpu6050
    if(value > 32768):
            value = value - 65536
    return value

class IMU (threading.Thread):
    def __init__(self, SSN, DRDY):
        threading.Thread.__init__(self)
        self.SSN = SSN
        self.DRDY = DRDY
        self.rm3100 = RM3100(self.SSN,self.DRDY)
        self.Readings = None
        self.Rates = None
        # Sensor initialization
        
        self.bus = smbus.SMBus(1) 	# or bus = smbus.SMBus(0) for older version boards

        MPU_Init(self.bus)

        time.sleep(1)
        #Read Accelerometer raw value
        self.accX = read_raw_data(self.bus,ACCEL_XOUT_H)
        self.accY = read_raw_data(self.bus,ACCEL_YOUT_H)
        self.accZ = read_raw_data(self.bus,ACCEL_ZOUT_H)


        self.roll = math.atan2(self.accY,self.accZ) * radToDeg
        self.pitch = math.atan(-self.accX/math.sqrt((self.accY**2)+(self.accZ**2))) * radToDeg

        kalmanX.setAngle(self.roll)
        kalmanY.setAngle(self.pitch)
        self.gyroXAngle = self.roll;
        self.gyroYAngle = self.pitch;
        self.compAngleX = self.roll;
        self.compAngleY = self.pitch;

        self.timer = time.time()
        self.flag = 0
        self.HeadingNew = 0
        self.ThetaYaw = 0
        self.Alfa = 0.5


    def run(self):
        while True:
            if(self.flag >100):
                print("There is a problem with the connection")
                self.flag=0
                continue

            try:
                self.accX = read_raw_data(self.bus,ACCEL_XOUT_H)
                self.accY = read_raw_data(self.bus,ACCEL_YOUT_H)
                self.accZ = read_raw_data(self.bus,ACCEL_ZOUT_H)
                
                #Read Gyroscope raw value
                self.gyroX = read_raw_data(self.bus,GYRO_XOUT_H)
                self.gyroY = read_raw_data(self.bus,GYRO_YOUT_H)
                self.gyroZ = read_raw_data(self.bus,GYRO_ZOUT_H)

                self.dt = time.time() - self.timer
                self.timer = time.time()
                self.roll = math.atan2(self.accY,self.accZ)*radToDeg
                self.pitch = math.atan(-self.accX/math.sqrt((self.accY**2)+(self.accZ**2)))*radToDeg
                
                self.gyroXRate = self.gyroX/131
                self.gyroYRate = self.gyroY/131
                
                if((self.roll < -90 and self.kalAngleX >90) or (self.roll > 90 and self.kalAngleX < -90)):
                    kalmanX.setAngle(self.roll)
                    self.complAngleX = self.roll
                    self.kalAngleX   = self.roll
                    self.gyroXAngle  = self.roll
                else:
                    self.kalAngleX = kalmanX.getAngle(self.roll,self.gyroXRate,self.dt)

                
                if((self.pitch < -90 and self.kalAngleY >90) or (self.pitch > 90 and self.kalAngleY < -90)):
                    kalmanY.setAngle(self.pitch)
                    self.complAngleY = self.pitch
                    self.kalAngleY   = self.pitch
                    self.gyroYAngle  = self.pitch
                else:
                    self.kalAngleY = kalmanY.getAngle(self.pitch,self.gyroYRate,self.dt)
                    
                self.gyroXAngle = self.gyroXRate * self.dt
                self.gyroYAngle = self.gyroYAngle * self.dt
                
                
                self.compAngleX = 0.93 * (self.compAngleX + self.gyroXRate * self.dt) + 0.07 * self.roll
                self.compAngleY = 0.93 * (self.compAngleY + self.gyroYRate * self.dt) + 0.07 * self.pitch
                
                if ((self.gyroXAngle < -180) or (self.gyroXAngle > 180)):
                    self.gyroXAngle = self.kalAngleX
                if ((self.gyroYAngle < -180) or (self.gyroYAngle > 180)):
                    self.gyroYAngle = self.kalAngleY
                    
                self.pitchNew = self.kalAngleY *math.pi / 180
                self.rollNew = -self.kalAngleX *math.pi / 180

                mag = self.rm3100.readMag()
                # print(self.kalAngleX, self.kalAngleY)
                # print(mag)
                if mag !=None:
                    # CMx = mag['x']*math.cos(self.pitchNew) + mag['z']*math.sin(self.pitchNew)
                    # CMy = mag['x']*math.sin(self.rollNew)*math.sin(self.pitchNew) + mag['y']*math.cos(self.rollNew)- mag['z']*math.sin(self.rollNew)*math.cos(self.pitchNew)


                    CMx = mag['x']*math.cos(self.pitchNew)+ mag['y']*math.sin(self.rollNew)*math.sin(self.pitchNew) - mag['z']*math.cos(self.rollNew)*math.sin(self.pitchNew)
                    CMy = mag['y']*math.cos(self.rollNew) + mag['z']*math.sin(self.rollNew)

                    # if CMx < 0:
                    #     self.HeadingNew = -(math.atan2(CMy,CMx) * 180 / math.pi)
                    #     pass
                    # elif CMx > 0 and CMy < 0:
                    #     self.HeadingNew = -(math.atan2(CMy,CMx) * 180 / math.pi)
                    # elif CMx > 0 and CMy > 0:
                    #     self.HeadingNew = 360 -(math.atan2(CMy,CMx) * 180 / math.pi)
                    # elif CMx == 0 and CMy < 0:
                    #     self.HeadingNew = 90
                    # elif CMx == 0 and CMy > 0:
                    #     self.HeadingNew = 270
                    # else:
                    #     self.HeadingNew = -(math.atan2(CMy,CMx) * 180 / math.pi)

                    self.HeadingNew = -(math.atan2(CMy,CMx) * 180 / math.pi)+25
                    # if self.HeadingNew > 180:
                    #     self.HeadingNew = self.HeadingNew - 360


                                                
                
                    self.ThetaYaw = self.Alfa * self.ThetaYaw + (1 - self.Alfa) * self.HeadingNew
                    self.Readings ={'Roll':self.rollNew * 180 /math.pi,'Pitch':self.pitchNew* 180 /math.pi,'Yaw':self.ThetaYaw }
                else:
                    self.Readings = None

            except Exception as exc:
                print(exc)
                self.flag += 1