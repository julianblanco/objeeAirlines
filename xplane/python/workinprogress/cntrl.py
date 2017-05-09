#!/usr/bin/python
# -*- coding: utf-8 -*-import xpc

import xpc
import serial


s = serial.Serial('/dev/ttyUSB1', timeout = 3 )

with xpc.XPlaneConnect() as client:
		while True:
				data = s.readline().strip()
				dref = "sim/operation/override/override_joystick"
				dref1= "sim/joystick/yoke_roll_ratio"
				dref2= "sim/joystick/yoke_pitch_ratio"
				dref3= "sim/flightmodel/engine/ENGN_thro_use"
				dref4= "sim/operation/override/override_throttles"
				dref5= "sim/cockpit2/engine/actuators/throttle_ratio_all"
				a=data.split(",")
				print a
				pitch=float(a[0])
				roll=float(a[1])*-1
				throttle=float(a[2])
				value = float(1)
				client.sendDREF(dref, value)
				client.sendDREF(dref4, value)
				client.sendDREF(dref2, roll)
				client.sendDREF(dref1, pitch)
				client.sendDREF(dref3, throttle)
		