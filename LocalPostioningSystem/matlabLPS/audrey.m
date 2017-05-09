%-------------------------------------------------

    %Establish the serial communication
    try
        comPort = '/dev/ttyUSB0';
        BAUD=9600;
        [trek1000 ,flag] = setupSerial(comPort,BAUD);
    catch
        try 
            comPort = '/dev/ttyS101';
            BAUD=9600;
            [trek1000 ,flag] = setupSerial(comPort,BAUD);
        catch
             try 
                comPort = '/dev/ttyS102';
                BAUD=9600;
                [trek1000 ,flag] = setupSerial(comPort,BAUD);
             catch
                 try 
                    comPort = '/dev/ttyS103';
                    BAUD=9600;
                    [trek1000 ,flag] = setupSerial(comPort,BAUD);
                catch
                    comPort = '/dev/ttyS104';
                    BAUD=9600;
                    [trek1000 ,flag] = setupSerial(comPort,BAUD);
                end
               
             end
            
        end
    end

%-----------
for k = 1:1000;
    dist = fgetl(trek1000);
    
    c = strsplit(dist,',');
    
    x(k)= str2num(c{1});
    y(k)= str2num(c{2});
    z(k)= str2num(c{3});
    pause (.5)
    %hold on
%     t(k)=k
    plot(t,x)
    
    
    
end