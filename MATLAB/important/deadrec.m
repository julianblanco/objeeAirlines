clc
lastreportedpos = [0 0 0];
estimatedtravel = [0 0 0];
actualpostion         = [0 0 0];

velocity=1;
roll=0;
pitch=deg2rad(0);
yaw=deg2rad(90);

postion=lastreportedpos+estimatedtravel;

%estimatedtravel = [cos(yaw)*velocity*cos(pitch) cos(pitch)*sin(yaw)*velocity sin(pitch)*velocity]
%sqrt((sqrt(estimatedtravel(1)^2 +estimatedtravel(2)^2))^2 +estimatedtravel(3)^2)
tic
starttiempo =tic;
for i=1:20
    i
   estimatedtravel = [cos(yaw)*velocity*cos(pitch)+(rand(1)-1) (rand(1)-1)+cos(pitch)*sin(yaw)*velocity (rand(1)-1)+sin(pitch)*velocity];
   postion=postion+estimatedtravel
   actualpostion =actualpostion+ [cos(yaw)*velocity*cos(pitch) cos(pitch)*sin(yaw)*velocity sin(pitch)*velocity];
      
   if mod(i,5)==0
       lastreportedpos = i*[cos(yaw)*velocity*cos(pitch) cos(pitch)*sin(yaw)*velocity sin(pitch)*velocity];
       postion=lastreportedpos;
   end
    
   plot3(postion(1), postion(2), postion(3))
    
    tiempo=toc;
    pause((i)-tiempo)
end
actualpostion
    