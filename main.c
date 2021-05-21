#include "stm8s.h"
#include "step.h"
#include "delay.h"
#include "led.h"

void main(void)
{
  u8 flag = 0;

  CLK_HSIPrescalerConfig(CLK_PRESCALER_HSIDIV1);        // ����Ϊ�ڲ�����ʱ��
  timer4_init();                // TIM4��ʼ��
  LED_Init();                   // LED��ʼ��
  Step_Motor_GPIO_Init();       // ���������ʼ�� IN1-IN4��C7-C4

  while (1)
  {
    if(flag)
    {
      // �����ת��Ȧ
      motor_circle(32, 1, 2);
      flag = 0;
    }
    else
    {
      // �����ת��Ȧ
      motor_circle(32, 0, 2);
      flag = 1;
    }
    
    // ��תLED
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
