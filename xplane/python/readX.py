#!/usr/bin/python
# -*- coding: utf-8 -*-import xpc

import xpc
import serial
import random
from colorama import Fore, Back, Style
from socket import timeout as timeout_exception
import time
start_time = time.time()
# s = serial.Serial('/dev/ttyUSB2', baudrate=115200,timeout = 3 )
fc= serial.Serial('/dev/ttyUSB0',baudrate=115200, timeout = 3 )
fc1= serial.Serial('/dev/ttyUSB1',baudrate=115200, timeout = 3 )
count = 0
lasttime = 0
lasttime2 =0 
running = True
noiseflag=0
stringsend1 = ''
secondhertz = 0
while ( running ):
	try:
		with xpc.XPlaneConnect('localhost') as client:
				client.setCONN(49008)
				while True:
						count = count + 1

						result = client.readDATA();
						
						
						if ( result ):
							loopstart = time.time()
							
							
							# a, b, c, d, e, f, g, h, i =  result[4]
							a, b, c, d, e, f, g, airspeed, gpsspeed =  result[0]
							a,pitch_a, roll_a, rudder, d, e, f, g, h =  result[1]
							a,pitch, roll, headg, d, e, f, g, h =  result[2]
							aa, Lat, Long, e,Alt, f, g, h, i =  result[3]
							throttle_a,b, c, z, d, e, f, g, h =  result[4]


							if noiseflag:
								# airspeed = airspeed + random.uniform(-4,4)
								# gpsspeed=gpsspeed + random.uniform(-1,1)
								# pitch=pitch + random.uniform(-5,5)
								# roll=roll + random.uniform(-2,2)
								# headg=headg + random.uniform(-2,2)

								# Lat=Lat + random.uniform(-.000001,.000001)
								# Long=Long + random.uniform(-.000001,.000001)
								Alt=Alt +random.uniform(-2,2)

							# a, vel, c, d, e, f, g, h, i =  result[3]
							if count > 1:
								
								listsend2 = ['<08',',',str(Lat)[:8],',',str(Long)[:9],',',str(Alt)[:2],'>\n']
								stringsend2 = ''.join("%s" % ''.join(map(str, x)) for x in listsend2)
								# fc.write(stringsend2)
								# print(chr(27) + "[2J")
								looptime =(loopstart - start_time)
								print(chr(27) + "[2J")
								print round(1/(looptime-lasttime))
								print secondhertz
								lasttime=looptime
								# print result
								# print a
								# print b
								# print c
								print 'Pitch : ','%.4f' % pitch
								print 'Roll  : ','%.4f' % roll
								print 'Headg : ','%.4f' % headg
								# print d
								# print e
								# print g

								print ''

								print 'Lat   : ','%.6f' % Lat
								print 'Long  : ','%.6f' % Long
								print 'Alt   : ','%.2f' % Alt
								print ''

								print 'GPSspeed   : ','%.3f' % gpsspeed
								print 'Airspeed   : ','%.3f' % airspeed
								print ''
								print count
								print stringsend1


							
							listsend1 = ['<10,' , str(pitch)[:5],',',str(roll)[:5],',',str(headg)[:5] ,'>\n']
							stringsend1 = ''.join("%s" % ''.join(map(str, x)) for x in listsend1)
							fc.write(stringsend1)	
								 # print stringsend1	


							if (count % 30)==0:
								
								looptime2 =(loopstart - start_time)
								
								secondhertz=(1/(looptime2-lasttime2))
								lasttime2=looptime2
								# print stringsend2
								# count = 0
								# listsend = ['<$OA009,' , str(Lat)[:8],',',str(Long)[:9],',',str(Alt)[:2],',',str(pitch)[:5],',',str(roll)[:5],',',str(headg)[:5],',',str(speed)[:2],'>\n']
								# stringsend=''
								# stringsend = ''.join("%s" % ''.join(map(str, x)) for x in listsend)
								listsend = ['<08,' , str(Lat)[:8],',',str(Long)[:9],',',str(gpsspeed)[:5],',',str(Alt)[:9] ,'>\n']
								stringsend=''
								stringsend = ''.join("%s" % ''.join(map(str, x)) for x in listsend)
								# print stringsendGpsSpeed
								# # print stringsend2
								fc.write(stringsend)
								
								# fcdata= fc.readline().strip()
								# print fcdata

							if (count % 20)==0:

								listsend = ['<08,' , str(Lat)[:8],',',str(Long)[:9],',',str(gpsspeed)[:5],',',str(headg)[:9] ,'>\n']
								stringsend=''
								stringsend = ''.join("%s" % ''.join(map(str, x)) for x in listsend)
								# print stringsendGpsSpeed
								# # print stringsend2
								fc1.write(stringsend)
							if (count % 10)==0:

								listsend = ['<10,' , str(pitch)[:5],',',str(roll)[:5],',',str(headg)[:5], ',',str(Alt)[:9],'>\n']
								stringsend=''
								stringsend = ''.join("%s" % ''.join(map(str, x)) for x in listsend)
								# print stringsendGpsSpeed
								# # print stringsend2
								fc1.write(stringsend)
								
								# fcdata= fc.readline().strip()
								# print fcdata
							if (count % 20)==0:

								
								listsend3 = ['<09,',str(Alt)[:9] ,'>\n']
								stringsend3=''
								stringsend3 = ''.join("%s" % ''.join(map(str, x)) for x in listsend3)
								# print stringsendGpsSpeed
								# # print stringsend2
								fc.write(stringsend3)

							if count > 301:
								count = 0



	except ValueError as e:
		print (Fore.RED + e.message)
		print "string to float fail. trying again..."
		print(Style.RESET_ALL)
		continue

	except timeout_exception as e: 
		print (Fore.RED + e.message)
		print "Net Time Out, Restarting ......"
		print(Style.RESET_ALL)

		# [(4.203895392974451e-45, 32.007877349853516, 32.00171661376953, 32.03706359863281, 26.910985946655273, -999.0, 36.834007263183594, 36.86759948730469, 36.86760330200195), (1.1210387714598537e-44, 0.1599999964237213, 0.029999999329447746, 0.0, -999.0, -999.0, -999.0, -999.0, -999.0), (1.5414283107572988e-44, 0.09394659847021103, 0.023118983954191208, -0.006562217604368925, -999.0, 0.0, -999.0, -999.0, -999.0), (2.1019476964872256e-44, 0.0001062502633430995, 4.525366966845468e-05, 0.004870241973549128, -999.0, -999.0, -999.0, -999.0, -999.0), (2.2420775429197073e-44, 0.125583678483963, 0.04948854446411133, 0.04621975123882294, -999.0, -999.0, -999.0, -999.0, -999.0), (2.382207389352189e-44, 33.74857711791992, 6.6730055809021, 318.48052978515625, 305.28424072265625, -999.0, -999.0, -999.0, -999.0), (2.5223372357846707e-44, 0.7268245220184326, -0.21701274812221527, 318.74871826171875, 32.86037826538086, -999.0, -999.0, -999.0, -0.8215440511703491), (2.6624670822171524e-44, 293.22967529296875, -13.196281433105469, -999.0, -999.0, -999.0, -999.0, -999.0, -999.0), (2.802596928649634e-44, 37.45866394042969, -122.11271667480469, 76.13797760009766, 72.36016082763672, 1.0, 75.64216613769531, 36.0, -124.0), (2.942726775082116e-44, 34222.21484375, -273.53155517578125, -51127.75390625, -9.1283540725708, 8.942646026611328, -10.408421516418457, 266.291015625, 0.04382586106657982), (4.90454462513686e-44, 1.5786479711532593, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0), (9.528829557408756e-44, 2.783281087875366, -999.0, 0.11419054120779037, 0.041027311235666275, -999.0, -999.0, -999.0, 0.8408443331718445)]
# <$OA009,6.3,-23,182,41.32496,-72.04393,51,37>
