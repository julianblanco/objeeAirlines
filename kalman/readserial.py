#!/usr/bin/python
# -*- coding: utf-8 -*-import xpc


import serial
from colorama import Fore, Back, Style
import time
import subprocess

subprocess.call(["rm", "/home/capstone/objeeairlines/kalman/kalmantestdata.txt"])

start_time = time.time()
s = serial.Serial('/dev/ttyACM1',baudrate=115200, timeout = 3 )
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
				count = count + 1
				data = s.readline().strip()
				# print data
				a=data.split(",")
				# print a
				kalmanlat  =float(a[2])
				kalmanlong  =float(a[3])
				myLat  =float(a[0])
				myLong  =float(a[1])
				xpos  =float(a[4])
				ypos  =float(a[5])
				kalmanxpos  =float(a[6])
				kalmanypos  =float(a[7])
				timediff  =float(a[8])
				P00  =float(a[9])
				P01  =float(a[10])
				P02  =float(a[11])
				P03  =float(a[12])

				P10  =float(a[13])
				P11 =float(a[14])
				P12  =float(a[15])
				P13  =float(a[16])

				P20  =float(a[17])
				P21  =float(a[18])
				P22  =float(a[19])
				P23  =float(a[20])

				P30  =float(a[21])
				P31  =float(a[22])
				P32  =float(a[23])
				P33  =float(a[24])

				APM0  =float(a[25])
				APM1  =float(a[26])
				APM2  =float(a[27])
				APM3  =float(a[28])

				# rollServoOutput= ( 0 - rollServoOutput) + 0 
				# middle = (pitchServoOutput + rollServoOutput)/2
				# ailerondefct = (pitchServoOutput + middle)
				# fcpitch = middle
				# fcroll = ailerondefct
	
				if count > 10:

					print(chr(27) + "[2J")
					print 'myLat        : ',myLat
					print 'kalmanLat    : ',kalmanlat
					print 'myLong       : ',myLong
					print 'kalmanLong   : ',kalmanlong
					print 'xpos         : ',xpos
					print 'kalmanxpos   : ',kalmanxpos
					print 'ypos         : ',ypos
					print 'kalmanypos   : ',kalmanypos

					print 'timediff     : ',timediff
					print ''
					print str(P00) + ' ' + str(P01) + ' ' + str(P02) + ' ' + str(P03)
					print str(P10) + ' ' + str(P11) + ' ' + str(P12) + ' ' + str(P13)
					print str(P20) + ' ' + str(P21) + ' ' + str(P22) + ' ' + str(P23)
					print str(P30) + ' ' + str(P31) + ' ' + str(P32) + ' ' + str(P33)
					print ''
					print str(APM0)
					print str(APM1)
					print str(APM2)
					print str(APM3)
					count = 0
					timesincestart= time.time() - start_time
					listsend = [str(myLat)[:8],',',str(myLong)[:9],',',str(kalmanlat),',',str(kalmanlong),',',str(xpos),',0,',str(ypos)[:5],',',str(kalmanxpos)[:5],',',str(kalmanypos),'\n']
					stringsend=''
					stringsend = ''.join("%s" % ''.join(map(str, x)) for x in listsend)
					openfile = open('kalmantestdata.txt', 'a')
					openfile.write(stringsend)	
					openfile.close()
					




	except ValueError as e:
		print (Fore.RED + e.message)
		print "string to float fail. trying again..."
		print(Style.RESET_ALL)
		continue

