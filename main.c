#include "stm8s.h"
#include "step.h"
#include "delay.h"
#include "led.h"

void main(void)
{
  u8 flag = 0;

  CLK_HSIPrescalerConfig(CLK_PRESCALER_HSIDIV1);        // 设置为内部高速时钟
  timer4_init();                // TIM4初始化
  LED_Init();                   // LED初始化
  Step_Motor_GPIO_Init();       // 步进电机初始化 IN1-IN4接C7-C4

  while (1)
  {
    if(flag)
    {
      // 电机正转半圈
      motor_circle(32, 1, 2);
      flag = 0;
    }
    else
    {
      // 电机反转半圈
      motor_circle(32, 0, 2);
      flag = 1;
    }
    
    // 翻转LED
    LED_Reverse();
    Delay_ms(3000);
  }
  
}

#ifdef USE_FULL_ASSERT

/**
  * @brief  Reports the name of the source file and the source line number
  *   where the assert_param error has occurred.
  * @param file: pointer to the source file name
  * @param line: assert_param error line source number
  * @retval : None
  */
void assert_failed(u8* file, u32 line)
{ 
  /* User can add his own implementation to report the file name and line number,
     ex: printf("Wrong parameters value: file %s on line %d\r\n", file, line) */

  /* Infinite loop */
  while (1)
  {
  }
}
#endif
