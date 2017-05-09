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


exitFlag = 0

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
										buffer_string = buffer_string + ser.read(ser.inWaiting())
										if '\n' in buffer_string:
											lines = buffer_string.split('\n') # Guaranteed to have at least 2 entries
											last_received = lines[-2]
											#If the Arduino sends lots of empty lines, you'll lose the
											#last filled line, so you could make the above statement conditional
											#like so: if lines[-2]: last_received = lines[-2]
											buffer_string = lines[-1]




def main_code():
	count = 0
	lasttime = 0
	running = True
	noiseflag=0
	stringsend1 = ''
	choice_a=0
	pitch_a=0
	throttle_a=0
	value_a =0
	fcleft_a=0
	fcright_a=0

	while ( running ):
		try:
			with xpc.XPlaneConnect() as client:
					client.setCONN(49008)
					while True:
							

							count = count + 1

							result = client.readDATA();
							
							
							if ( result ):
								
								
							
								
								# a, b, c, d, e, f, g, h, i =  result[4]
								a, b, c, d, e, f, g, airspeed, gpsspeed =  result[0]
								a,pitch, roll, headg, d, e, f, g, h =  result[1]
								aa, Lat, Long, Alt, e, f, g, h, i =  result[2]


								if noiseflag:
									airspeed = airspeed + random.uniform(-4,4)
									gpsspeed=gpsspeed + random.uniform(-1,1)
									pitch=pitch + random.uniform(-2,2)
									roll=roll + random.uniform(-2,2)
									headg=headg + random.uniform(-2,2)

									Lat=Lat + random.uniform(-.000001,.000001)
									Long=Long + random.uniform(-.000001,.000001)
									Alt=Alt +random.uniform(-2,2)

								# a, vel, c, d, e, f, g, h, i =  result[3]
								if count > 1:
									
									data = last_received.strip()
									a=data.split(",")
									# print a
									choice_a=float(a[0])
									roll_a=float(a[1])
									pitch_a=float(a[2])*-1
									throttle_a=float(a[3])
									value_a = float(1)
									fcleft_a=float(a[4])
									fcright_a=float(a[5])
									
									listsend2 = ['<08',',',str(Lat)[:8],',',str(Long)[:9],',',str(Alt)[:2],'>\n']
									stringsend2 = ''.join("%s" % ''.join(map(str, x)) for x in listsend2)
									# fc.write(stringsend2)
									# print(chr(27) + "[2J")
									looptime =(time.time() - start_time)
									print(chr(27) + "[2J")
									print looptime-lasttime
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

									print 'Pitch_send : ','%.4f' % pitch_a
									print 'Roll_send  : ','%.4f' % roll_a
									print 'throttle : ','%.4f' % throttle_a


								# if (count % 6)==0:
								# 	listsend1 = ['<10,' , str(pitch)[:4],',',str(roll)[:4],',',str(headg)[:5],',',str(Alt)[:4],'>\n']
								# 	stringsend1 = ''.join("%s" % ''.join(map(str, x)) for x in listsend1)
									
								# 	 # print stringsend1	


								if (count % 1)==0:
									
									
									# print stringsend2
									# count = 0
									# listsend = ['<$OA009,' , str(Lat)[:8],',',str(Long)[:9],',',str(Alt)[:2],',',str(pitch)[:5],',',str(roll)[:5],',',str(headg)[:5],',',str(speed)[:2],'>\n']
									# stringsend=''
									# stringsend = ''.join("%s" % ''.join(map(str, x)) for x in listsend)
									
									timesincestart= time.time() - start_time
									listsend = [str(Lat)[:8],',',str(Long)[:9],',0,0,0,0,',str(headg)[:5],',',str(gpsspeed)[:5],',0,',str(roll),',',str(pitch),',',str(roll_a),',',str(pitch_a),',0,0,0,0,',str(Alt)[:2],',0,',str(timesincestart),'\n']
									stringsend=''
									stringsend = ''.join("%s" % ''.join(map(str, x)) for x in listsend)
									# print stringsend
									# # print stringsend2
									openfile = open('xplanetestdata.txt', 'a')
									openfile.write(stringsend)	
									openfile.close()
									# fcdata= fc.readline().strip()
									# print fcdata
								if count > 31:
									count = 0

								if count > 1000:
									running = false 	
								

								
				


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

# Create new threads
thread1 = myThread(1)
thread2 = myThread(2)

# Start new Threads
thread1.start()
thread2.start()

print "Exiting Main Thread"