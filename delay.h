#ifndef __DELAY_H
#define __DELAY_H

#include "stm8s.h"

extern __IO uint32_t TimingDelay;
extern __IO uint16_t g_Timing_tim4;

void timer4_init(void);
void Delay_ms(__IO uint16_t nTime);
void TimingDelay_Decrement(void);
void Delay(uint16_t nCount);

#endif
