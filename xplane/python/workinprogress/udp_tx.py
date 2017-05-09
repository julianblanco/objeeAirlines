#!/usr/bin/env python
import socket
import serial

path = '/dev/ttyACM1'
UDP_IP = "10.1.214.239"
s = serial.Serial('/dev/ttyUSB0', timeout = 3 )

UDP_PORT = 49000
MESSAGE = "Hello, World!"

print "UDP target IP:", UDP_IP
print "UDP target port:", UDP_PORT
print "message:", MESSAGE

# sock2 = socket.socket(socket.AF_INET, # Internet
                     # socket.SOCK_DGRAM) # UDP
# sock2.bind(('127.0.0.1', 49003))
sock = socket.socket(socket.AF_INET, # Internet
	                     socket.SOCK_DGRAM) # UDP

while True:
	data = s.readline().strip()
	# data='$0,.4,0'
	databytes=str.encode(data)
	sock.sendto(databytes, (UDP_IP, UDP_PORT))
	# data2, addr = sock.recvfrom(1024) # buffer size is 1024 bytes
	print 'TX:', data
	# print 'RX:', data2