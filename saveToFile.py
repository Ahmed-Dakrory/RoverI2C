import os
import re
import sys
import xml.etree.ElementTree as ET
from pyroutelib3 import Router # Import the router
from datetime import datetime
def setTagLocation(fileToWrite,lat,longit,iddata,typeLocation):
    
    now = datetime.now() # current date and time
    date_time = now.strftime("%Y-%m-%dT%H:%M:%SZ")
    xmlString = "\r\n\r\n <node changeset=\"123123\" id=\""+str(iddata)+"\" lat=\""+str(lat)+"\"    lon=\""+str(longit)+"\" timestamp=\""+date_time+"\" uid=\"0\" user=\"NewDakroryTags\" version=\"5\" visible=\"true\"> \r\n <tag k=\"amenity\" v=\""+typeLocation+"\" />\r\n </node> \r\n \r\n \r\n \r\n\r\n"
    tree = ET.parse(fileToWrite)
    root = tree.getroot()

    findedNodeWay = "."
    data= root.findall(findedNodeWay)
    xml= ET.fromstring(xmlString)  
    data[0].append( xml)
    #print(ET.tostring(data[0]))
    tree.write(fileToWrite)
    print("Done")

    
def setWayWidth(lat,longit,mode,fileRead,width):
    
    router = Router(mode)
    node = router.findNode(lat,longit)
    #print(node)
    tree = ET.parse(fileRead)
    root = tree.getroot()

    findedNodeWay = ".//*[@ref='"+str(node)+"'].."
    data= root.findall(findedNodeWay)
    xmlString= "<tag k=\"width\" v=\""+str(width)+"\"></tag>"
    xml= ET.fromstring(xmlString)  
    data[0].append( xml)
    tree.write(fileRead)
    print("Done")


#setWayWidth(51.927017,4.480630,'cycle','data.osm',5)

#now = datetime.now() # current date and time
#RandomId = now.strftime("%Y%m%d%H%M%S")
#setTagLocation('fileTag.osm',51.9259928,4.4718395,RandomId,'joy')
