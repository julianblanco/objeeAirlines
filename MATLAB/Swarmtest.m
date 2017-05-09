clc
clear all
clf
time=0:1:100;
startlat=42;
startlong=71;
targetlat=44;
targetlong=71;
currentheading =90;
speed=10;
postion=0;
distance=distance2waypoint(startlat,startlong,targetlat,targetlong);
targetheading=course2waypoint(startlat,startlong,targetlat,targetlong);
disp(strcat('Distance to waypoint',{' '},num2str(distance)));
disp(strcat('Course to waypoint',{' '},num2str(targetheading)));
[newlat,newlong]=latgenerator(44,71,110575,180);
disp(strcat('NewLat:',num2str(newlat)));
disp(strcat('NewLong:',num2str(newlong)));
[currentlat,currentlong]=currentpostion(currentheading,42,75,10,200);
for i=1:1:(length(currentlat))
    clf
    [lat2,long2]=latgenerator(currentlat(i),currentlong(i),400,currentheading-160);
    [lat3,long3]=latgenerator(currentlat(i),currentlong(i),400,currentheading-200);
    plot(currentlong(i),currentlat(i),'go')
    hold on
    plot(long2,lat2,'ro')
    plot(long3,lat3,'bo')
    axis([74.95,75.05,41.95,42.05])
    pause(.001)
    
     
end
currentheading=currentheading+90;
[secondlat,secondlong]=currentpostion(currentheading,currentlat(199),currentlong(199),10,200);

for i=1:1:(length(secondlat))
    clf
    [lat2,long2]=latgenerator(secondlat(i),secondlong(i),400,currentheading-160);
    [lat3,long3]=latgenerator(secondlat(i),secondlong(i),400,currentheading-200);
    plot(secondlong(i),secondlat(i),'go')
    hold on
    plot(long2,lat2,'ro')
    plot(long3,lat3,'bo')
    axis([74.95,75.05,41.95,42.05])
    pause(.001)
    
    
     
end