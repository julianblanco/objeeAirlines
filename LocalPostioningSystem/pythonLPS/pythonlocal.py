#!/usr/bin/env python
import localization as lx

P=lx.Project(mode="3D",solver="LSE")

path = '/dev/ttyACM0'

anchorList=["anch0","anch1","anch2", "anch3"]
anchorCoordinates={"anch0": (40,20,1), "anch1": (40,-20,1), "anch2": (-40,20,1), "anch3" : (-40,-20,6)}
distanceDict={"anch0": (44.721), "anch1": (44.721), "anch2": (44.721), "anch3" : (45)}


for anchor in anchorCoordinates:
	P.add_anchor(anchor, anchorCoordinates[anchor])

t,label=P.add_target()
for x in range(0,50):
	for anchor in anchorList:
		t.add_measure(anchor,distanceDict[anchor])


	P.solve()
	B=t.loc

	coordonnees=[B.x, B.y, B.z]
	print B.x
	print B.y
	print B.z
#print ('x' +str(B.x)+','+'y'+','B.y)

	