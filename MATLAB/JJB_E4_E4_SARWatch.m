disp('This function will assist a SAR watchstander develop a');
disp('priortized response list to a SAR case based on nearby assets ');
disp('The response will be priortized based on the distance from the case.');
disp('Please type in the file contain the most recent ship location:');



AISdata = input('Please type in the file contain the most recent ship location: ','s');
fopen(AISdata);
%while loop with data
Count = 0;
while(~feof)
    