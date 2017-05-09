#include <Wire.h>
#include "message.h"



int new_message = 0;
float sum = 0;

String instring = "";
size_t index = 0;
int serial_message_ready = 0;

int pi_reply = 0;

vector_t vector = { 0, 0, 0 };

void setup()
{
    // Begin serial connection
    Serial.begin(9600);
    
    // Setup Arduino I2C as a slave
    Wire.begin(I2C_SLAVE_ADDR);
    Wire.onReceive(I2C_RX_Event);
    Wire.onRequest(I2C_TX_Event);
    Wire.write("hey there sweetie");
}



void loop()
{
    if (new_message)
    {
        sum = vector.x + vector.y + vector.z;
        
        Serial.println("You have mail!");
        Serial.print("x="); Serial.println(vector.x, 5);
        Serial.print("y="); Serial.println(vector.y, 5);
        Serial.print("z="); Serial.println(vector.z, 5);
        Serial.print("sum="); Serial.println(sum, 6);
        delay(100);
        
        //Wire.write((int)sum); // Write back truncated sum
        new_message = 0;
    }
    
    /*
    if (pi_reply)
    {
        Wire.write(42);
        pi_reply = 0;
    }
    */
    
    /*
    if (serial_message_ready)
    {
        Serial.println(instring.c_str());
        Wire.write("penis");
        instring = "";
        serial_message_ready = 0;
    }
    */
}



// Upon receiving an I2C message
void I2C_RX_Event(int bytes)
{
    static uint8_t byte_num = 0;    // Counts bytes as they arrive
    static uint8_t size;            // Size of message
    static uint8_t message[255];       // Byte array to store the message
    
    // While there is data available read it
    while (Wire.available())
    {
        uint8_t data = Wire.read();
        
        if( byte_num == 0 && data == 0 ){
            index = 0;
            continue;
        }
        
        // First byte sent is the size of the message in bytes
        if (byte_num == 0) {
            size = Wire.read();
            if( size == 0 ){
                //Serial.println("sending sum");
                pi_reply = 1;
                continue;
            }
        }
        // All the rest of the bytes that follow are the message
        else {
            message[byte_num - 1] = Wire.read();
        }
        
        // Increment the total number of bytes received
        byte_num++;
    }
    
    // If the transmission is complete reset byte_num and send to recv_data
    if (byte_num - 1 == size)
    {
        // Reset byte count and set new message flag
        memcpy(&vector, message, sizeof(vector));
        
        byte_num = 0;
        new_message = 1;
    }
}


// Upon request of an I2C message
void I2C_TX_Event()
{
    Wire.write('A'+index);
    index++;
}


// If I send something over the COM port
void serialEvent()
{
    instring = "";
    
    while (Serial.available())
    {
        instring += (char)Serial.read();
    }
    
    //serial_message_ready = 1;
}