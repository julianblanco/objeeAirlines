

#ifdef pitot

  void pitotSample(){
    float IN = analogRead(A0);
    float pressure = ((IN/1023.0 - .2) / .2); //transfer function from manufacturer
    float windspeed = sqrt ( (2.0 * (pressure-0.14) * 1000)/1.2); 
    float windspeedcorrected = 2*sqrt ( (2.0 * (pressure-0.14)*2000)/1.2);
}
#endif
// Serial print to com port and xbee to matlab
////////////////////////////////////////////////////////////

