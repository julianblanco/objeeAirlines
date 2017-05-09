%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
%JACK
%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
comPort ='COM5';
BAUD = 9600;

if (~exist('serialFlag', 'var'))
    [arduino.s,serialFlag] = setupSerial(comPort,BAUD);
end
%intialize figure
if ~exist('h','var')|| ishandle(h)
    h = figure(1);
end
%add start stop button
if ~exist('button','var')
    button=uicontrol('Style','pushbutton','String','Stop',...
        'pos',[0 0 50 25], 'parent',h,...
        'Callback', 'stop_call_animation3d','UserData',1);
end
    
 if ~exist('button2','var')
    button=uicontrol('Style','pushbutton','String','Close Serial Port',...
        'pos',[250 0 150 25], 'parent',h,...
        'Callback','closeSerial','UserData',1);
 end

if ~exist('slider1','var')
    slider1 = uicontrol('Style','slider','String', 'Filter 1',...
        'pos',[740 0 150 25],'val',1,'parent',h);
    
    textbox = uicontrol('Style','text',Position', [740 25 150 15],...
        'String',['Filter Coeffiecient =', num2str(get(slider1,'val'))],...
        'parent',h);
end


if ~exist('myAxes','var')
    myaxes = axes('XLim', [-10 10],'YLim',[0 50],'Zlim',[-5 5]);
    view(3);
    grid on;
    axis equal;
    hold on;
    xlabel('x')
    ylabel('y(cm)')
    zlabel('z')
    
    [x y z ] =cylinder([0.2 0.2]);
    [cx cy cz] = cylinder([.2 .2]);
    [sx sy sz] = sphere(20);
    
    s(1) = surface(x-0.5,y,z-.5);
    s(2) = surface(x+0.5,y,z-.5);
    s(3) = surface(z-0.5,x,y-.5);
    s(4) = surface(z-0.5,x,y+.5);
    s(5) = surface(.2*sx-0.5,0.2*sy,0.2*sz-0.5);
    s(6) = surface(.2*sx+0.5,0.2*sy,0.2*sz-0.5);
    s(7) = surface(.2*sx-0.5,0.2*sy,0.2*sz+0.5);
    s(8) = surface(.2*sx+0.5,0.2*sy,0.2*sz+0.5);
    s(9) = surface(0.5*cx, 0.5*cy,0.35*cz);
    s(10)= surface(0.5*cx,0.5*cy,-0.35*cz);
    s(11)= surface(0.35*cz,0.5*cx,0.5*cy);
    s(12)= surface(-.035*cz,0.5*cx,0.5*cy);
    
    combinedObject = hgtransform('Parent',myAxes);
    set(s, 'Parent', combinedObject)
    set(h,'Postion',[200 200 890 660])
    drawnow;
end
    
mode = 'P';

dyFilt = 0;
alpha=1;

while(get(button,'UserData'))
    alpha = get(slider1, 'Value');
    
    dyFilt= readDist(arduino.s, mode);
    
    translation = makehgtform('translate',[0 dyfilt 0 ]);
    scaling = makehgtform('scale',35/dyFilt);
    set(combinedObject,'Matrix',translation*scaling);
    
    drawnow;
    
end
    
    
