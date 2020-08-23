from Routing import RoutingUsage # Import the router
from time import sleep
from datetime import datetime
import GPS as gps# Import the router


GpsData = [0,51.9271369,4.4871358]
gps = gps.GpsThreadReadings(GpsData)
gps.start()




while True:
    mode = int(input("Press 1 for Routing or 2 for Gps File Generating and 3 for Exit: "))

    if mode == 1:
        print("Routing Mode Welcome.................")
        nLocations = int(input("Press Number Of Locations: "))
        routingMode = int(input("Press 1 for cycle mode or 2 for car mode: "))
        
        if routingMode ==1 :
                routingClass = RoutingUsage("cycle")
        if routingMode ==2 :
                routingClass = RoutingUsage("car")

        nodes = []
        #make Different Routing with multiple routing

        lat1 = gps.getGpsReadings()[1]
        long1 = gps.getGpsReadings()[2]
        node1=routingClass.node(lat1,long1)
        print(node1)
        nodes.append(node1)
        for j in range(0,nLocations):
                latNext = float(input("Enter Lat Next Point: "))
                longNext = float(input("Enter Long Next Point: "))
                
                nodeNext=routingClass.node(latNext,longNext)
                nodes.append(nodeNext)
                
        nodes.reverse()
        
        nodesNew = routingClass.arrangeNodesDependsOnLength(nodes)
                                
        queueNodesNewRight = routingClass.getRouteMultiple(nodesNew)
                
        if queueNodesNewRight == None:
            print("No Path for one of the Paths")
        else:
            f= open("route.osm","w+")
            f.write("<?xml version=\"1.0\" encoding=\"UTF-8\"?>\r\n\r\n<osm version=\"0.6\" generator=\"Overpass API 0.7.55.7 8b86ff77\">\r\n\r\n<note>The data included in this document is from www.openstreetmap.org. The data is made available under ODbL.</note>\r\n\r\n<meta osm_base=\"2019-07-22T12:50:02Z\"/>\r\n\r\n  <bounds minlat=\"30.0059000\" minlon=\"31.1707000\" maxlat=\"30.0271000\" maxlon=\"31.2036000\"/>\r\n")

            for j in range(0,len(queueNodesNewRight)):
                for i in queueNodesNewRight[j][0]:
                    f.write("<node id=\"%d1%d3\" lat=\"%f\" lon=\"%f\" version=\"2\" timestamp=\"2019-05-10T12:30:39Z\" changeset=\"70107889\" uid=\"9535075\" user=\"JacksonWard\"/>\r\n" % (int(i[0]),j,i[1],i[2]))
                    
            f.write(" <way id=\"348553335\" version=\"10\" timestamp=\"2019-04-08T04:19:00Z\" changeset=\"99992695\" uid=\"9990114\" user=\"The_Plateau\">\r\n")
            for j in range(0,len(queueNodesNewRight)):
                for i in queueNodesNewRight[j][0]:
                    f.write("<nd ref=\"%d1%d3\"/>\r\n" % (int(i[0]),j))
                
                
            f.write("</way>\r\n\r\n </osm>\r\n")
            f.close()

    elif mode == 2:
        fileName = input("Enter File Name: ")
        listOfGpsReadings = []
        while True:
            try:
                lat = gps.getGpsReadings()[1]
                lon = gps.getGpsReadings()[2]
                listOfGpsReadings.append([lat,lon])
                sleep(0.01)
            except KeyboardInterrupt:
                break  # The answer was in the question!

        f= open(fileName+".osm","w+")

        f.write("<?xml version=\"1.0\" encoding=\"UTF-8\"?>\r\n\r\n<osm version=\"0.6\" generator=\"Overpass API 0.7.55.7 8b86ff77\">\r\n\r\n<note>The data included in this document is from www.openstreetmap.org. The data is made available under ODbL.</note>\r\n\r\n<meta osm_base=\"2019-07-22T12:50:02Z\"/>\r\n\r\n  <bounds minlat=\"30.0059000\" minlon=\"31.1707000\" maxlat=\"30.0271000\" maxlon=\"31.2036000\"/>\r\n")
        

        for j in range(0,len(listOfGpsReadings)):
            lat = listOfGpsReadings[j][0]
            longit = listOfGpsReadings[j][1]
            
            f.write("<node id=\"%d1%d3\" lat=\"%f\" lon=\"%f\" version=\"2\" timestamp=\"2019-05-10T12:30:39Z\" changeset=\"70107889\" uid=\"9535075\" user=\"JacksonWard\"/>\r\n" % (j,2*j,lat,longit))
        

        f.write(" <way id=\"348553335\" version=\"10\" timestamp=\"2019-04-08T04:19:00Z\" changeset=\"99992695\" uid=\"9990114\" user=\"The_Plateau\">\r\n")

        for j in range(0,len(listOfGpsReadings)):
            f.write("<nd ref=\"%d1%d3\"/>\r\n" % (j,2*j))
            
            
        f.write("</way>\r\n\r\n </osm>\r\n")
        f.close()
    
    elif mode == 3:
        gps.GpsRun = False
        break
gps.GpsRun = False
