#!/usr/bin/python
# -*- coding: utf-8 -*-import xpc

from colorama import Fore, Back, Style
import serial
import time

fc= serial.Serial('/dev/ttyUSB2',baudrate=115200,   timeout = 3 )
data=''
result=''
count = 0
running = True
while ( running ):
	try:
		while True:
			data = fc.readline().strip()
			print data
			result=data.split(",")
			 # print result
			myLat, myLong, targetLat,targetLong,targetHeading,trueHeading,gpsHeading,GpsSpeed,airspeed,wapointNumber,distanceToTarget,myAltitude,rollInput,pitchInput, armed ,yokex,yokey,rolltarget,pitchtarget,winddir,windmag,throttle,lastlat,lastlong=result
			print(chr(27) + "[2J")
			print 'myLat        : ',myLat
			print 'myLong       : ',myLong
			print 'targetLat    : ',targetLat
			print 'targetLong   : ',targetLong
			print 'targetHeading: ',targetHeading
			print 'trueHeading  : ',trueHeading
			print 'gpsHeading  : ',gpsHeading
			print 'GpsSpeed     : ',GpsSpeed
			print 'AirSpeed     : ',airspeed
			print 'wapointNumber: ',wapointNumber
			print 'distanceToTar: ',distanceToTarget
			print 'myAltitude   : ',myAltitude
			print 'rollInput    : ',rollInput
			print 'pitchInput   : ',pitchInput
			print 'armed        :',armed
			print 'Yoke Aileron :', yokex
			print 'Yoke Elevator:', yokey
			print 'Roll Target  :', rolltarget
			print 'Pitch Target :', pitchtarget
			print 'Wind Directi :', winddir
			print 'Wind Mag     :', windmag
			print 'Throttle    :', throttle
			print 'Throttle    :', lastlat
			print 'Throttle    :', lastlong
			# time.sleep(1)
			
			
			
	except ValueError as e:
		print (Fore.GREEN + data)
		print (Fore.RED + e.message)
		print "string to float fail. trying again..."
		print(Style.RESET_ALL)
		continue
	# except timeout_exception as e: 
	# 	print e.message
	# 	print "Net Time Out, Restarting ......"

		# [(4.203895392974451e-45, 32.007877349853516, 32.00171661376953, 32.03706359863281, 26.910985946655273, -999.0, 36.834007263183594, 36.86759948730469, 36.86760330200195), (1.1210387714598537e-44, 0.1599999964237213, 0.029999999329447746, 0.0, -999.0, -999.0, -999.0, -999.0, -999.0), (1.5414283107572988e-44, 0.09394659847021103, 0.023118983954191208, -0.006562217604368925, -999.0, 0.0, -999.0, -999.0, -999.0), (2.1019476964872256e-44, 0.0001062502633430995, 4.525366966845468e-05, 0.004870241973549128, -999.0, -999.0, -999.0, -999.0, -999.0), (2.2420775429197073e-44, 0.125583678483963, 0.04948854446411133, 0.04621975123882294, -999.0, -999.0, -999.0, -999.0, -999.0), (2.382207389352189e-44, 33.74857711791992, 6.6730055809021, 318.48052978515625, 305.28424072265625, -999.0, -999.0, -999.0, -999.0), (2.5223372357846707e-44, 0.7268245220184326, -0.21701274812221527, 318.74871826171875, 32.86037826538086, -999.0, -999.0, -999.0, -0.8215440511703491), (2.6624670822171524e-44, 293.22967529296875, -13.196281433105469, -999.0, -999.0, -999.0, -999.0, -999.0, -999.0), (2.802596928649634e-44, 37.45866394042969, -122.11271667480469, 76.13797760009766, 72.36016082763672, 1.0, 75.64216613769531, 36.0, -124.0), (2.942726775082116e-44, 34222.21484375, -273.53155517578125, -51127.75390625, -9.1283540725708, 8.942646026611328, -10.408421516418457, 266.291015625, 0.04382586106657982), (4.90454462513686e-44, 1.5786479711532593, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0), (9.528829557408756e-44, 2.783281087875366, -999.0, 0.11419054120779037, 0.041027311235666275, -999.0, -999.0, -999.0, 0.8408443331718445)]
