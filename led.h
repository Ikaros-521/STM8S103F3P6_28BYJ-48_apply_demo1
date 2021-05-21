#ifndef __LED_H
#define __LED_H

#include "stm8s.h"

#define ON 0
#define OFF 1

void LED_Init(void);
void LED_Reverse(void);
void LED_ON_OFF(u8 ON_OFF);

#endif
