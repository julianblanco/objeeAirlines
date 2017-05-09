clf
clear all
hold on
matlabImage = imread('football2.png');

image([-1 1],[1 -1],matlabImage);   %image(matlabImage,'Parent',map);
%h=scatter(rand(1,20)-0.5,rand(1,20)-0.5,'r','filled');  %# Plot some random data

CurrentLat=41.372153;
CurrentLong= 72.098668;
if CurrentLat>41.372156
        x=CurrentLat*-.0115;
    else
        x=CurrentLat*.011;
end
    
     if CurrentLong>72.099064
        y=CurrentLong*-.0051;
    else
        y=CurrentLong*.001;
     end
     
     
    j='';
    j=scatter(x,y,'r','filled');  %# Plot some random data
 %   set(j,'Parent',map);
    
    drawnow;