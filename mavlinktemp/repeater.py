#!/usr/bin/python
# -*- coding: utf-8 -*-import xpc

import serial
import socket
from colorama import Fore, Back, Style
xbee= serial.Serial('/dev/ttyUSB0',baudrate=57600, timeout = 3 )
# UDP_IP = "127.0.0.1"
UDP_IP = "192.168.1.130"
UDP_PORT = 14550
running =1
sock = socket.socket(socket.AF_INET, # Internet
                     socket.SOCK_DGRAM) # UDP
while ( running ):

	data = xbee.read()
	print data
	sock.sendto(data, (UDP_IP, UDP_PORT))
	data=''

	



	# except ValueError as e:
	# 	print (Fore.RED + e.message)
	# 	print "string to float fail. trying again..."
	# 	print(Style.RESET_ALL)
	# 	continue

	# except timeout_exception as e: 
	# 	print (Fore.RED + e.message)
	# 	print "Net Time Out, Restarting ......"
	# 	print(Style.RESET_ALL)
