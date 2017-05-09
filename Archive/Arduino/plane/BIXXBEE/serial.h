
#ifdef serialCommands

String inputString = "";         // a string to hold incoming data
boolean stringComplete = false;  // whether the string is complete

void Serial_Check(){
  if (xbee.available()) 
  {
    char inChar = (char)Serial.read();
    inputString += inChar;
      if (inChar == '\n') 
      {
      stringComplete = true;
      }
  }
}


  int newheading(int current_heading)
    {
       int newhead =0;
    if (stringComplete)
    {
      xbee.println(inputString);
     newhead=inputString.toInt();
     inputString="";
     xbee.print("newhead: ");xbee.println(newhead);
     stringComplete=0;
    }
    else
    {
      newhead =current_heading;
    }
    return newhead;
    }
  #endif
