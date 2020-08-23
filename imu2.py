from Routing import RoutingUsage # Import the router
import simulation as sim
import GPS as gps# Import the router
import serial
import simulation as sim
import imu



    
    
from smbus import SMBus

addr = 8 # bus address
bus = SMBus(1) # indicates /dev/ic2-1

gy87=imu.GY87(bus)
gy87.start()

while True:
    angleRover=gy87.MpuGyroZandHeadingReading[1]        
    sim.sendDistanceAndAngleToMicroController("23", angleRover,8,bus)
        