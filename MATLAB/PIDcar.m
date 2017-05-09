clc
clear all
clf
time=0;
startlat=42;
startlong=71;
targetlat=44;
targetlong=71;
trueheading =90;
speed=100;
speed1=100;
speed2=100;
postion=0;
traildistance =5000;
traildistance1=2500;
[currentlat,currentlong]=currentpostion(trueheading,startlat,startlong,10,1);
[currentlat1,currentlong1]=currentpostion(trueheading,41.85,72,10,1);
[currentlat2,currentlong2]=currentpostion(trueheading,41.95,71,10,1);
for i=1:1:1200
    clf
    [currentlat,currentlong]=currentpostion(trueheading,currentlat,currentlong,speed,1);
    [targetlat2,targetlong2]=latgenerator(currentlat(2),currentlong(2),traildistance,trueheading-180);
    [targetlat3,targetlong3]=latgenerator(currentlat(2),currentlong(2),traildistance1,trueheading-180);
    trueheading1 =course2waypoint(currentlat1(2),currentlong1(2),targetlat2,targetlong2);
    trueheading2 =course2waypoint(currentlat2(2),currentlong2(2),targetlat3,targetlong3);
    distance2 =distance2waypoint(currentlat2(2),currentlong2(2),currentlat(2),currentlong(2));
    distance1 =distance2waypoint(currentlat1(2),currentlong1(2),currentlat(2),currentlong(2));
   
    
    
    
    
    if distance1 > 2500
       speed1=120;
   end
   if distance1 <2500
       speed1=100;
   end
   if distance2 < 2500
       speed2=100;
   end   
   if distance2 > 2500
       speed2=120;
   end   
    
    [currentlat1,currentlong1]=currentpostion(trueheading1,currentlat1,currentlong1,speed1,1);
    [currentlat2,currentlong2]=currentpostion(trueheading2,currentlat2,currentlong2,speed2,1);
    hold on
    axis([71,72,41.5,42.5])
    plot(currentlong(2),currentlat(2),'go')
    plot(currentlong1(2),currentlat1(2),'bo')
    plot(currentlong2(2),currentlat2(2),'ro')
    pause(.005)
    
    disp(currentlat1(2))
    disp(currentlong1(2))
    disp(trueheading1)
    if i>300&&i<400
        trueheading=trueheading+1;
    end
    if i>400&&i<500
        trueheading=trueheading-1;
    end
     if i>500&&i<550
        trueheading=trueheading-2;
    end
    
    if i>700&&i<800
        trueheading=trueheading+1;
    end
    
    if i>900&&i<1000
        trueheading=trueheading+1;
    end
    
    
    
end