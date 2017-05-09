#!/usr/bin/python
# -*- coding: utf-8 -*-import xpc

import xpc
import serial
from colorama import Fore, Back, Style
from socket import timeout as timeout_exception
import time
start_time = time.time()
s = serial.Serial('/dev/ttyUSB0', timeout = 3 )
# fc= serial.Serial('/dev/ttyUSB1',baudrate=115200, timeout = 3 )
count = 0
lasttime = 0
running = True

def translate(value, leftMin, leftMax, rightMin, rightMax):
	# Figure out how 'wide' each range is
	leftSpan = leftMax - leftMin
	rightSpan = rightMax - rightMin
	# Convert the left range into a 0-1 range (float)
	valueScaled = float(value - leftMin) / float(leftSpan)
	# Convert the 0-1 range into a value in the right range.
	return rightMin + (valueScaled * rightSpan)

def saturate(inputnum , lower ,upper):
	if (inputnum > upper):
		inputnum = upper
	if (inputnum < lower):
		inputnum = lower
	return inputnum

while ( running ):
	try:
		with xpc.XPlaneConnect() as client:
				# client.setCONN(49008)
				while True:
						count = count + 1
						data = s.readline().strip()
						dref = "sim/operation/override/override_joystick"
						dref1= "sim/joystick/yoke_roll_ratio"
						dref2= "sim/joystick/yoke_pitch_ratio"
						dref3= "sim/flightmodel/engine/ENGN_thro_use"
						dref4= "sim/operation/override/override_throttles"
						dref5= "sim/cockpit2/engine/actuators/throttle_ratio_all"
						a=data.split(",")
						# print a
						myLat  =float(a[0])
						myLong  =float(a[1])
						targetLat  =float(a[2])
						targetLong  =float(a[3])
						targetHeading  =float(a[4])
						trueHeading  =float(a[5])
						Gpsheading  =float(a[6])
						GpsSpeed  =float(a[7])
						AirSpeed  =float(a[8])
						waypointNumber  =float(a[9])
						distanceToTarget  =float(a[10])
						myAltitude  =float(a[11])
						rollInput  =float(a[12])
						pitchInput  =float(a[13])
						armed  =float(a[14])
						rollServoOutput  =float(a[15])
						pitchServoOutput  =float(a[16])
						rollSetpoint  =float(a[17])
						pitchSetpoint  =float(a[18])
						winddir  =float(a[19])
						windmag  =float(a[20])
						throtSetpoint  =float(a[21])
						pitchAccum  =float(a[22])
						masterLat  =float(a[23])
						masterLong  =float(a[24])
						masterGPSspeed  =float(a[25])
						masterAlt =float(a[26])
						masterHeading =float(a[27])
						fcright= ( 0 - fcright) + 0 
						middle = (fcleft + fcright)/2
						ailerondefct = (fcleft - middle)
						fcpitch = middle
						fcroll = ailerondefct
			
						if count > 10:


							print 'FCright: ', translate(fcright,-1,1,1100,1900)
							print 'FCleft: ', translate(fcleft,-1,1,1100,1900)
							print 'FCPitch: ', translate(fcpitch,-1,1,1100,1900)
							print 'FCRoll  : ', translate(fcroll,-1,1,1100,1900)
							print 'FCThrot : ',translate(fcthrottle,-1,1,1100,1900)
							count = 0

						pitch = saturate(pitch,-1,1)
						roll = saturate(roll,-1,1)
						throttle = saturate(throttle,0,1)

						client.sendDREF(dref, 1)
						client.sendDREF(dref4, 1)
						client.sendDREF(dref2, pitch)
						client.sendDREF(dref1, roll)
						client.sendDREF(dref3, throttle)


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

