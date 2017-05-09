#!/usr/bin/python
# -*- coding: utf-8 -*-import xpc

import xpc
import serial
from socket import timeout as timeout_exception
import socket
# import sys
UDP_IP = "127.0.0.1"
UDP_PORT = 5005
sock = socket.socket(socket.AF_INET, # Internet
                     socket.SOCK_DGRAM) # UDP
sock.bind((UDP_IP, UDP_PORT))
s = serial.Serial('/dev/ttyUSB0', timeout = 3 )
count = 0
running = True
while ( running ):
	try:
		with xpc.XPlaneConnect() as client:
				client.setCONN(49008)
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
						#print a
						pitch=float(a[0])
						roll=float(a[1])*-1
						throttle=float(a[2])
						value = float(1)
						client.sendDREF(dref, value)
						client.sendDREF(dref4, value)
						client.sendDREF(dref2, roll)
						client.sendDREF(dref1, pitch)
						client.sendDREF(dref3, throttle)
						result = client.readDATA();
						data, addr = sock.recvfrom(1024) 
						
						if ( result ):
							a,pitch, roll, headg, d, e, f, g, h =  result[1]
							aa, Lat, Long, Alt, e, f, g, h, i =  result[2]
							a, vel, c, d, e, f, g, h, i =  result[3]
							
							a, speed, c, d, e, f, g, h, i =  result[0]
							a, b, c, d, e, f, g, h, i =  result[4]

							if count > 10:
								
								print(chr(27) + "[2J")
								# print result
								# print a
								# print b
								# print c
								print 'Pitch : ', pitch
								print 'Roll  : ', roll
								print 'Headg : ', headg
								print ''

								print 'Lat   : ', Lat
								print 'Long  : ', Long
								print 'Alt   : ', Alt
								print ''

								print 'Vel   : ', speed
								print "received message:", data
								count = 0
								stringsend = '<$OA009,' , str(pitch),',',str(roll),',',str(headg),',',str(Lat),',',str(Long),',',str(Alt),',',str(speed),','
								s.sendline(stringsend)






	except ValueError as e:
		print e.message
		print "string to float fail. trying again..."
		continue

	except timeout_exception as e: 
		print e.message
		print "Net Time Out, Restarting ......"

		# [(4.203895392974451e-45, 32.007877349853516, 32.00171661376953, 32.03706359863281, 26.910985946655273, -999.0, 36.834007263183594, 36.86759948730469, 36.86760330200195), (1.1210387714598537e-44, 0.1599999964237213, 0.029999999329447746, 0.0, -999.0, -999.0, -999.0, -999.0, -999.0), (1.5414283107572988e-44, 0.09394659847021103, 0.023118983954191208, -0.006562217604368925, -999.0, 0.0, -999.0, -999.0, -999.0), (2.1019476964872256e-44, 0.0001062502633430995, 4.525366966845468e-05, 0.004870241973549128, -999.0, -999.0, -999.0, -999.0, -999.0), (2.2420775429197073e-44, 0.125583678483963, 0.04948854446411133, 0.04621975123882294, -999.0, -999.0, -999.0, -999.0, -999.0), (2.382207389352189e-44, 33.74857711791992, 6.6730055809021, 318.48052978515625, 305.28424072265625, -999.0, -999.0, -999.0, -999.0), (2.5223372357846707e-44, 0.7268245220184326, -0.21701274812221527, 318.74871826171875, 32.86037826538086, -999.0, -999.0, -999.0, -0.8215440511703491), (2.6624670822171524e-44, 293.22967529296875, -13.196281433105469, -999.0, -999.0, -999.0, -999.0, -999.0, -999.0), (2.802596928649634e-44, 37.45866394042969, -122.11271667480469, 76.13797760009766, 72.36016082763672, 1.0, 75.64216613769531, 36.0, -124.0), (2.942726775082116e-44, 34222.21484375, -273.53155517578125, -51127.75390625, -9.1283540725708, 8.942646026611328, -10.408421516418457, 266.291015625, 0.04382586106657982), (4.90454462513686e-44, 1.5786479711532593, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0), (9.528829557408756e-44, 2.783281087875366, -999.0, 0.11419054120779037, 0.041027311235666275, -999.0, -999.0, -999.0, 0.8408443331718445)]