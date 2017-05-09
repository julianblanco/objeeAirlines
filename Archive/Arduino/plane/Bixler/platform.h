//platform.h

#ifndef _PLATFORM_H_
#define _PLATFORM_H_




void TC3_Handler();

void startTimer(Tc *tc, uint32_t channel, IRQn_Type irq, uint32_t frequency);
void digitalWriteDirect(int pin, boolean val);
 int digitalReadDirect(int pin);


#endif