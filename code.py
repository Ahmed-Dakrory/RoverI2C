from smbus import SMBus
from Routing import RoutingUsage # Import the router
from time import sleep
import saveToFile
from datetime import datetime



mode = int(input("Press 1 for Routing or 2 for Editing: "))

if mode == 1:
	print("Routing Mode Welcome.................")
	



	addr = 8 # bus address
	bus = SMBus(1) # indicates /dev/ic2-1



	def convertNumberIntoAsciValue(valueToConvert):
		latLongList = list(str(valueToConvert))
		latLongAsciBytes = []
		for i in range(0,len(latLongList)):
			latLongAsciBytes.append(ord(latLongList[i]))
		return latLongAsciBytes

	def sendArrayOfBytes(address,data):
		for i in range(0,len(data)):
			bus.write_byte(address,data[i])
			
	nLocations = int(input("Press Number Of Locations: "))
	routingMode = int(input("Press 1 for cycle mode or 2 for car mode: "))
	
	if routingMode ==1 :
		routingClass = RoutingUsage("cycle")
	if routingMode ==2 :
		routingClass = RoutingUsage("car")

	nodes = []
	#make Different Routing with multiple routing

	lat1 = float(input("Enter Lat start: "))
	long1 = float(input("Enter Long start: "))
	node1=routingClass.node(lat1,long1)
	nodes.append(node1)
	for j in range(0,nLocations):
		latNext = float(input("Enter Lat Next Point: "))
		longNext = float(input("Enter Long Next Point: "))
		
		nodeNext=routingClass.node(latNext,longNext)
		nodes.append(nodeNext)
		
	nodesNew = routingClass.arrangeNodesDependsOnLength(nodes)
				
	queueNodesNewRight = routingClass.getRouteMultiple(nodesNew)
		
	if queueNodesNewRight == None:
		print("No Path for on of the Paths")
	else:
		for n in range(0,len(queueNodesNewRight)):
			for i in queueNodesNewRight[n][0]:
				sendArrayOfBytes(addr,convertNumberIntoAsciValue(i[0])) # switch it on
				sendArrayOfBytes(addr,convertNumberIntoAsciValue(':')) # switch it on
				sendArrayOfBytes(addr,convertNumberIntoAsciValue(i[1])) # switch it on

				sendArrayOfBytes(addr,convertNumberIntoAsciValue(',')) # switch it on

				sendArrayOfBytes(addr,convertNumberIntoAsciValue(i[2])) # switch it on

				sendArrayOfBytes(addr,convertNumberIntoAsciValue(';')) # switch it on

elif mode == 2:
	
	modeEdit = int(input("Press 1 to add width for ways or 2 to add Tag: "))
	if modeEdit == 1:
		width = float(input("Enter the way width: "))
		saveToFile.setWayWidth(51.927017,4.480630,'cycle','data.osm',width)
	elif modeEdit == 2:
		
		now = datetime.now() # current date and time
		RandomId = now.strftime("%Y%m%d%H%M%S")
		typeOfPlace = (input("Enter the tag type: "))
		saveToFile.setTagLocation('fileTag.osm',51.9259928,4.4718395,RandomId,typeOfPlace)

	
	
	
	
		
	

