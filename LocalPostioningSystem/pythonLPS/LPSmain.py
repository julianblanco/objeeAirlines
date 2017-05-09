#!/usr/bin/env python
import localization as lx
import serial


P=lx.Project(mode="3D",solver="LSE")

path = '/dev/ttyACM0'

anchorList=["anch0","anch1","anch2", "anch3"]
anchorCoordinates={"anch0": (30,30,1.4), "anch1": (30,0,5), "anch2": (0,0,.6), "anch3" : (0,30,1.4)}
distanceDict={"anch0": (44.721), "anch1": (44.721), "anch2": (44.721), "anch3" : (45)}

for anchor in anchorCoordinates:
			P.add_anchor(anchor, anchorCoordinates[anchor])

	
for x in range(0,150):

	# try:
		s = serial.Serial('/dev/ttyACM0', timeout = 3 )
		data = s.readline().split()
		if data[0] == 'mc':
			strn1='0'
			strn2='0'
			strn3='0'
			strn0='0'
		#print "read data as a list:"
		#print data
			mask = int(data[1],16)
			if (mask & 0x01):
			  #print "range0 good"
			  range0 = int(data[2],16)/1000.0
			  distanceDict["anch0"]=range0
			  strn0=str(range0)

			else:
			  print "range0 bad"
			  # range0 = -1
			if (mask & 0x02):
			  #print "range1 good"
			  range1 = int(data[3],16)/1000.0
			  distanceDict["anch1"]=range0
			  strn1=str(range1)


			else:
			  print "range1 bad"
			  # range1 = -1
			if (mask & 0x04):
			  #print "range2 good"
			  range2 = int(data[4],16)/1000.0
			  distanceDict["anch2"]=range0
			  strn2=str(range2)

			else:
			  print "range2 bad"
			  # range2 = -1
			if (mask & 0x08):
			  #print "range3 good"
			  range3 = int(data[5],16)/1000.0
			  distanceDict["anch3"]=range0
			  strn3=str(range3)

			else:
			  pass
			  #print "range3 bad"
			  # range3 = -1


		
		print data


	# print range3
		print( "rnstr:"+strn0+","+strn1+","+strn2+","+strn3)
		

		t,label=P.add_target()
		
		for anchor in anchorList:
			t.add_measure(anchor,distanceDict[anchor])


		P.solve()
		B=t.loc

		coordonnees=[B.x, B.y, B.z]
		print B.x
		print B.y
		print B.z


s.close()