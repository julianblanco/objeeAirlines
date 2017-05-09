

// // #ifdef pitot

// //   void pitotSample(){
// //     float IN = analogRead(A0);
// //     float pressure = ((IN/1023.0 - .2) / .2); //transfer function from manufacturer
// //     float windspeed = sqrt ( (2.0 * (pressure-0.14) * 1000)/1.2); 
// //     float windspeedcorrected = 2*sqrt ( (2.0 * (pressure-0.14)*2000)/1.2);

// // }



// // // For the airspeed sensor, the I2C address (SENSOR_ADDRESS in the pseudo code below) is
// // // 0xEA
// // // For the alti

// float readi2cpitot(void){
// byte data[2];
//  signed short reading = 0xFFFF;
//  i2c_start();
//  // select sensor in write mode
//  if (!(i2c.write(SENSOR_ADDRESS | I2C_WRITE_BIT))) {
//  // send "read data" command to sensor

//  if (!i2c_write(0x07)) { 
//  i2c.restart(); // perform I2C restart
//  // select sensor in read mode
//  if (!i2c.write(| SENSOR_ADDRESS | I2C_READ_BIT)) {
//  // read two bytes of sensor data
//  data[0] = i2c_read(1);
//  data[1] = i2c_read(0);

//  reading = *((signed short *)(&data[0]));
//  }
//  }
//  }
//  i2c_stop(); 
// return reading
// }
// #endif
// // // Serial print to com port and xbee to matlab
// // ////////////////////////////////////////////////////////////

