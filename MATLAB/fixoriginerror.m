function [ steeringcompass ] = fixoriginerror( combineddata,GL,GC )
%UNTITLED4 Summary of this function goes here
%   Detailed explanation goes here


left=0;
right=0;
steeringcompass=0;
 
if (abs(GC -GL)) >180
    
   
        if GC > GL
            GC = GC -360;
            
        else
            GL=GL-360;
        end
        
        
        if (GC - GL) < 0
            left =1;
        end
        if (GL -GC) <0
            right =1;
        end
        
        if left==1
            steeringcompass = combineddata-(abs(GL)+abs(GC));
        end
        if right==1
            steeringcompass = combineddata+(abs(GL)+abs(GC));
        end
        
        if steeringcompass>360
            steeringcompass=steeringcompass-360;
        end
        if steeringcompass<0
            steeringcompass=steeringcompass+360;
        end
        
else 
    steeringcompass=combineddata + (GC-GL);
      if steeringcompass>360
            steeringcompass=steeringcompass-360;
        end
        if steeringcompass<0
            steeringcompass=steeringcompass+360;
        end
end

