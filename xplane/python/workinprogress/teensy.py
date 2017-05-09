#!/usr/bin/python
# -*- coding: utf-8 -*-import xpc

import xpc
import threading
import serial
import random
from colorama import Fore, Back, Style
from socket import timeout as timeout_exception
import time
start_time = time.time()
running = True
# count = 0
lasttime = 0
exitFlag = 0
buffer_string = ''

class myThread (threading.Thread):
    def __init__(self, threadID):
        threading.Thread.__init__(self)
        self.threadID = threadID
    def run(self):
        print "Starting " + self.name
        if self.threadID == 1:
        	main_code()
        if self.threadID == 2:
        	receiving()
        print "Exiting " + self.name

def receiving():
								global last_received
								while 1:
									ser = serial.Serial('/dev/ttyACM0', timeout = 3 )
									buffer_string = ''
									while True:
										# buffer_string = ser.readline().strip()
										# print buffer_string
										buffer_string = buffer_string + ser.read(ser.inWaiting())
										if '\n' in buffer_string:
											lines = buffer_string.split('\n') # Guaranteed to have at least 2 entries
											last_received = lines[-2]
											#If the Arduino sends lots of empty lines, you'll lose the
											#last filled line, so you could make the above statement conditional
											#like so: if lines[-2]: last_received = lines[-2]
											buffer_string = lines[-1]
											print buffer_string
											pause(.001)




def main_code():
	count = 0
	while ( running ):
		try:
			with xpc.XPlaneConnect() as client:
					# client.setCONN(49008)
					while True:
							count = count + 1
							# data = s.readline().strip()
							dref = "sim/operation/override/override_joystick"
							dref1= "sim/joystick/yoke_roll_ratio"
							dref2= "sim/joystick/yoke_pitch_ratio"
							dref3= "sim/flightmodel/engine/ENGN_thro_use"
							dref4= "sim/operation/override/override_throttles"
							dref5= "sim/cockpit2/engine/actuators/throttle_ratio_all"
							a=buffer_string.split(",")
							print a
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

							# rollServoOutput= ( 0 - rollServoOutput) + 0 
							# middle = (pitchServoOutput + rollServoOutput)/2
							# ailerondefct = (pitchServoOutput + middle)
							# fcpitch = middle
							# fcroll = ailerondefct
							
							if (count % 1)==0:
								
								
								# print stringsend2
								# count = 0
								# listsend = ['<$OA009,' , str(Lat)[:8],',',str(Long)[:9],',',str(Alt)[:2],',',str(pitch)[:5],',',str(roll)[:5],',',str(headg)[:5],',',str(speed)[:2],'>\n']
								# stringsend=''
								# stringsend = ''.join("%s" % ''.join(map(str, x)) for x in listsend)
								
								timesincestart= time.time() - start_time
								listsend = [str(Lat)[:8],',',str(Long)[:9],',0,0,0,0,',str(headg)[:5],',',str(gpsspeed)[:5],',0,',str(roll)[:4],',',str(pitch)[:4],',',str(roll_a),',',str(pitch_a),',0,0,0,0,',str(Alt)[:2],',0,',str(timesincestart),'\n']
								stringsend=''
								stringsend = ''.join("%s" % ''.join(map(str, x)) for x in listsend)
								# print stringsend
								# # print stringsend2
								openfile = open('xplanetestdata.txt', 'a')
								openfile.write(stringsend)	
								openfile.close()

							if count > 500:

								print(chr(27) + "[2J")
								print 'rollServoOutput: ', rollServoOutput
								print 'pitchServoOutput: ',pitchServoOutput
								# print 'FCRoll  : ', fcroll
								print '%%%%%%%%%%%'
								print 'myLat        : ',myLat
								print 'myLong       : ',myLong
								print 'targetLat    : ',targetLat
								print 'targetLong   : ',targetLong
								print '%%%%%%%%%%%'
								print 'targetHeading: ',targetHeading
								print 'trueHeading  : ',trueHeading
								print '%%%%%%%%%%%'
								print 'rollInput    : ',rollInput
								print 'Roll Target  :', rollSetpoint
								print '%%%%%%%%%%%'
								print 'Pitch Target :', pitchSetpoint
								print 'pitchInput   : ',pitchInput
								print '%%%%%%%%%%%'
								print 'myAltitude   : ',myAltitude
								# print 'gpsHeading  : ',gpsHeading
								print 'GpsSpeed     : ',GpsSpeed
								# print 'AirSpeed     : ',airspeed
								print 'wapointNumber: ',waypointNumber
								print 'distanceToTar: ',distanceToTarget
								
								
								
								# print 'armed        :',armed
								# print 'Yoke Aileron :', yokex
								# # print 'Yoke Elevator:', yokey
								# print 'Roll Target  :', rollSetpoint
								# print 'Pitch Target :', pitchSetpoint
								# print 'Wind Directi :', winddir
								# print 'Wind Mag     :', windmag
								# print 'Throttle    :', throttle
								# print 'Throttle    :', lastlat
								# print 'Throttle    :', lastlong
								count = 0

							pitch = saturate(pitchServoOutput,-1,1)
							roll = saturate(rollServoOutput,-1,1)
							# throttle = 1
							# throttle = saturate(throttle,0,1)
							print(chr(27) + "[2J")
							print 'yes'
							client.sendDREF(dref, 1)
							client.sendDREF(dref4, 1)
							client.sendDREF(dref2, pitch)
							client.sendDREF(dref1, roll)
							client.sendDREF(dref3, 1)
							pause(.0001)


		except ValueError as e:
			# print (Fore.RED + e.message)
			# print "string to float fail. trying again..."
			# print(Style.RESET_ALL)
			continue

		except timeout_exception as e: 
			print (Fore.RED + e.message)
			print "Net Time Out, Restarting ......"
			print(Style.RESET_ALL)

			# [(4.203895392974451e-45, 32.007877349853516, 32.00171661376953, 32.03706359863281, 26.910985946655273, -999.0, 36.834007263183594, 36.86759948730469, 36.86760330200195), (1.1210387714598537e-44, 0.1599999964237213, 0.029999999329447746, 0.0, -999.0, -999.0, -999.0, -999.0, -999.0), (1.5414283107572988e-44, 0.09394659847021103, 0.023118983954191208, -0.006562217604368925, -999.0, 0.0, -999.0, -999.0, -999.0), (2.1019476964872256e-44, 0.0001062502633430995, 4.525366966845468e-05, 0.004870241973549128, -999.0, -999.0, -999.0, -999.0, -999.0), (2.2420775429197073e-44, 0.125583678483963, 0.04948854446411133, 0.04621975123882294, -999.0, -999.0, -999.0, -999.0, -999.0), (2.382207389352189e-44, 33.74857711791992, 6.6730055809021, 318.48052978515625, 305.28424072265625, -999.0, -999.0, -999.0, -999.0), (2.5223372357846707e-44, 0.7268245220184326, -0.21701274812221527, 318.74871826171875, 32.86037826538086, -999.0, -999.0, -999.0, -0.8215440511703491), (2.6624670822171524e-44, 293.22967529296875, -13.196281433105469, -999.0, -999.0, -999.0, -999.0, -999.0, -999.0), (2.802596928649634e-44, 37.45866394042969, -122.11271667480469, 76.13797760009766, 72.36016082763672, 1.0, 75.64216613769531, 36.0, -124.0), (2.942726775082116e-44, 34222.21484375, -273.53155517578125, -51127.75390625, -9.1283540725708, 8.942646026611328, -10.408421516418457, 266.291015625, 0.04382586106657982), (4.90454462513686e-44, 1.5786479711532593, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0), (9.528829557408756e-44, 2.783281087875366, -999.0, 0.11419054120779037, 0.041027311235666275, -999.0, -999.0, -999.0, 0.8408443331718445)]
	# <$OA009,6.3,-23,182,41.32496,-72.04393,51,37>



# Create new threads
thread1 = myThread(1)
thread2 = myThread(2)

# Start new Threads
thread1.start()
thread2.start()

print "Exiting Main Thread"