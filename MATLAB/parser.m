clear all
clc
string = 'GPGLL,4122.3467,N,07205.9866,W,144935,A,D*5C';
lengthi =length(string);
flag=0;
lat=0;
long=0;
counter=0;
time=0;
dummy=0;
counter2=0;
for i=1:1:lengthi
    
    
    if strcmp(',',string(i))
           
            counter=counter+1;
           
         if flag==1
             flag =0;
             
         else
             flag =1;
         end
        
     end 
    
    if flag==1
        if strcmp(',',string(i))
        else
        if counter ==1
        lat=[lat,string(i)];
        end
      
        end
    end
    
     if flag==1
         if strcmp(',',string(i))
        else
         if counter==2
       dummy=[dummy,string(i)];
         end
         end
     end
      
       if flag==1
           if strcmp(',',string(i))
        else
         if counter==3
       long=[long,string(i)];
         end
           end
       end
     
         if flag==1
             if strcmp(',',string(i))
        else
         if counter==4
       dummy=[dummy,string(i)];
         end
         end
         end
           if flag==1
               if strcmp(',',string(i))
        else
         if counter==5
       time=[time,string(i)];
         end
               end
           end
   
end
   
   
    



stringdisp = ['The lat is',lat,', the long is',long,'the time is ',time];
disp(stringdisp);
















 