#include "led.h"

void LED_Init(void)
{
  GPIO_Init(GPIOB, GPIO_PIN_5, GPIO_MODE_OUT_PP_LOW_FAST);
}

void LED_Reverse(void)
{
  GPIO_WriteReverse(GPIOB, GPIO_PIN_5);
}

void LED_ON_OFF(u8 ON_OFF)
{
  if(ON_OFF)
    GPIO_WriteLow(GPIOB, GPIO_PIN_5);
  else
    GPIO_WriteHigh(GPIOB, GPIO_PIN_5);
}