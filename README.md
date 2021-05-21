# 前言

本文是根据[STM32F103+步进电机28BYJ-48+ULN2003 实现简单的正反转demo](https://blog.csdn.net/Ikaros_521/article/details/116598144)这篇文章的拓展，针对此文章的程序进行了STM8S103F3版本的粗略适配，具体细节可以参考这篇文章。

源码参考：

&nbsp;&nbsp;&nbsp;&nbsp;[步进电机28BYJ-48的驱动程序(stm32f103c8t6)](https://blog.csdn.net/qq_17280755/article/details/78459842)

&nbsp;&nbsp;&nbsp;&nbsp;[STM32F103+步进电机28BYJ-48+ULN2003 实现简单的正反转demo](https://blog.csdn.net/Ikaros_521/article/details/116598144)

开发板：STM8S103F3P6 最小系统

![在这里插入图片描述](https://img-blog.csdnimg.cn/202105211411340.png?x-oss-process=image/watermark,type_ZmFuZ3poZW5naGVpdGk,shadow_10,text_aHR0cHM6Ly9ibG9nLmNzZG4ubmV0L0lrYXJvc181MjE=,size_16,color_FFFFFF,t_70)

烧录：正点原子 ST-LINK/V2 ，软件STVP
![在这里插入图片描述](https://img-blog.csdnimg.cn/20210521140932536.png?x-oss-process=image/watermark,type_ZmFuZ3poZW5naGVpdGk,shadow_10,text_aHR0cHM6Ly9ibG9nLmNzZG4ubmV0L0lrYXJvc181MjE=,size_16,color_FFFFFF,t_70)
![在这里插入图片描述](https://img-blog.csdnimg.cn/20210521140956173.png)

语言：C语言

开发环境：IAR EW For STM8 （官网：[https://www.iar.com/](https://www.iar.com/)）

![在这里插入图片描述](https://img-blog.csdnimg.cn/20210521141010882.png)

**开发板**使用了 LED TIMER4 步进电机28BYJ-48  ULN2003驱动

## 代码下载：

[码云](https://gitee.com/ikaros-521/STM8S103F3P6_28BYJ-48_apply_demo1) [GitHub](https://github.com/Ikaros-521/STM8S103F3P6_28BYJ-48_apply_demo1)

## 功能介绍：

电机正转半圈，翻转LED，延时3秒，电机反转半圈，翻转LED，延时3秒。比stm32版更加简单。

# 参考图
## STM8S103F3最小系统原理图
![在这里插入图片描述](https://img-blog.csdnimg.cn/20210521140530237.png?x-oss-process=image/watermark,type_ZmFuZ3poZW5naGVpdGk,shadow_10,text_aHR0cHM6Ly9ibG9nLmNzZG4ubmV0L0lrYXJvc181MjE=,size_16,color_FFFFFF,t_70)

## 步进电机28BYJ-48
![在这里插入图片描述](https://img-blog.csdnimg.cn/20210521140640867.png?x-oss-process=image/watermark,type_ZmFuZ3poZW5naGVpdGk,shadow_10,text_aHR0cHM6Ly9ibG9nLmNzZG4ubmV0L0lrYXJvc181MjE=,size_16,color_FFFFFF,t_70)
![在这里插入图片描述](https://img-blog.csdnimg.cn/20210521140709776.png?x-oss-process=image/watermark,type_ZmFuZ3poZW5naGVpdGk,shadow_10,text_aHR0cHM6Ly9ibG9nLmNzZG4ubmV0L0lrYXJvc181MjE=,size_16,color_FFFFFF,t_70)
![在这里插入图片描述](https://img-blog.csdnimg.cn/20210521140720613.png?x-oss-process=image/watermark,type_ZmFuZ3poZW5naGVpdGk,shadow_10,text_aHR0cHM6Ly9ibG9nLmNzZG4ubmV0L0lrYXJvc181MjE=,size_16,color_FFFFFF,t_70)

## ST-Link/V2
参考文章：[ST-link/V2引脚定义](https://blog.csdn.net/qq_32693119/article/details/88799879)
![在这里插入图片描述](https://img-blog.csdnimg.cn/20210521150026218.png?x-oss-process=image/watermark,type_ZmFuZ3poZW5naGVpdGk,shadow_10,text_aHR0cHM6Ly9ibG9nLmNzZG4ubmV0L0lrYXJvc181MjE=,size_16,color_FFFFFF,t_70)
![在这里插入图片描述](https://img-blog.csdnimg.cn/20210521150129399.png?x-oss-process=image/watermark,type_ZmFuZ3poZW5naGVpdGk,shadow_10,text_aHR0cHM6Ly9ibG9nLmNzZG4ubmV0L0lrYXJvc181MjE=,size_16,color_FFFFFF,t_70)

# 接线
## ST-Link/V2

```c
VDD      ——> 3V3
SWIM     ——> SWIM
GND      ——> GND
RESET    ——> NRST
```

![在这里插入图片描述](https://img-blog.csdnimg.cn/20210521151127112.png?x-oss-process=image/watermark,type_ZmFuZ3poZW5naGVpdGk,shadow_10,text_aHR0cHM6Ly9ibG9nLmNzZG4ubmV0L0lrYXJvc181MjE=,size_16,color_FFFFFF,t_70)

## stm8s103f3
![在这里插入图片描述](https://img-blog.csdnimg.cn/20210521151504828.png?x-oss-process=image/watermark,type_ZmFuZ3poZW5naGVpdGk,shadow_10,text_aHR0cHM6Ly9ibG9nLmNzZG4ubmV0L0lrYXJvc181MjE=,size_16,color_FFFFFF,t_70)

## 步进电机28BYJ-48+ULN2003
![在这里插入图片描述](https://img-blog.csdnimg.cn/20210521151937737.png?x-oss-process=image/watermark,type_ZmFuZ3poZW5naGVpdGk,shadow_10,text_aHR0cHM6Ly9ibG9nLmNzZG4ubmV0L0lrYXJvc181MjE=,size_16,color_FFFFFF,t_70)

# 效果图
运行时需要**拔掉 NRST脚**的接线，不然会一直**复位**。或者 **关闭STVP**程序。
图片进行过处理，不是实际效果。
![在这里插入图片描述](https://img-blog.csdnimg.cn/20210521153018315.gif#pic_center)

# 工程介绍+烧写

## 工程结构
![在这里插入图片描述](https://img-blog.csdnimg.cn/20210521153222963.png)

## 工程设置
![在这里插入图片描述](https://img-blog.csdnimg.cn/20210521153332735.png)

![在这里插入图片描述](https://img-blog.csdnimg.cn/20210521153352543.png?x-oss-process=image/watermark,type_ZmFuZ3poZW5naGVpdGk,shadow_10,text_aHR0cHM6Ly9ibG9nLmNzZG4ubmV0L0lrYXJvc181MjE=,size_16,color_FFFFFF,t_70)
![在这里插入图片描述](https://img-blog.csdnimg.cn/20210521153417206.png?x-oss-process=image/watermark,type_ZmFuZ3poZW5naGVpdGk,shadow_10,text_aHR0cHM6Ly9ibG9nLmNzZG4ubmV0L0lrYXJvc181MjE=,size_16,color_FFFFFF,t_70)
![在这里插入图片描述](https://img-blog.csdnimg.cn/20210521153431647.png?x-oss-process=image/watermark,type_ZmFuZ3poZW5naGVpdGk,shadow_10,text_aHR0cHM6Ly9ibG9nLmNzZG4ubmV0L0lrYXJvc181MjE=,size_16,color_FFFFFF,t_70)
![在这里插入图片描述](https://img-blog.csdnimg.cn/20210521153528619.png?x-oss-process=image/watermark,type_ZmFuZ3poZW5naGVpdGk,shadow_10,text_aHR0cHM6Ly9ibG9nLmNzZG4ubmV0L0lrYXJvc181MjE=,size_16,color_FFFFFF,t_70)
程序路径
![在这里插入图片描述](https://img-blog.csdnimg.cn/20210521154025988.png)

## 程序烧写
ST-LINK -> SWIM -> STM8S103F3
![在这里插入图片描述](https://img-blog.csdnimg.cn/20210521153745472.png?x-oss-process=image/watermark,type_ZmFuZ3poZW5naGVpdGk,shadow_10,text_aHR0cHM6Ly9ibG9nLmNzZG4ubmV0L0lrYXJvc181MjE=,size_16,color_FFFFFF,t_70)

选择hex程序，进行烧写

![在这里插入图片描述](https://img-blog.csdnimg.cn/20210521153920797.png?x-oss-process=image/watermark,type_ZmFuZ3poZW5naGVpdGk,shadow_10,text_aHR0cHM6Ly9ibG9nLmNzZG4ubmV0L0lrYXJvc181MjE=,size_16,color_FFFFFF,t_70)

# 核心代码
## main.c

```c
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

```
## step.c

```c
#include "step.h"
#include "delay.h"

//IN4: PF4  d
//IN3: PF3  c
//IN2: PF2  b
//IN1: PF1  a

u8 forward[4] = {0x03,0x06,0x0c,0x09}; // 正转
u8 reverse[4]= {0x03,0x09,0x0c,0x06}; // 反转

//引脚初始化
void Step_Motor_GPIO_Init(void)
{
  GPIO_Init(GPIOC, GPIO_PIN_7, GPIO_MODE_OUT_PP_LOW_FAST);
  GPIO_Init(GPIOC, GPIO_PIN_6, GPIO_MODE_OUT_PP_LOW_FAST);
  GPIO_Init(GPIOC, GPIO_PIN_5, GPIO_MODE_OUT_PP_LOW_FAST);
  GPIO_Init(GPIOC, GPIO_PIN_4, GPIO_MODE_OUT_PP_LOW_FAST);
}

//引脚映射
void SetMotor(unsigned char InputData)
{
  if(InputData == 0x03)
  {
    GPIO_WriteHigh(GPIOC,GPIO_PIN_7);
    GPIO_WriteHigh(GPIOC,GPIO_PIN_6);
    GPIO_WriteLow(GPIOC,GPIO_PIN_5);
    GPIO_WriteLow(GPIOC,GPIO_PIN_4);
  }
  else if(InputData == 0x06)
  {
    GPIO_WriteLow(GPIOC,GPIO_PIN_7);
    GPIO_WriteHigh(GPIOC,GPIO_PIN_6);
    GPIO_WriteHigh(GPIOC,GPIO_PIN_5);
    GPIO_WriteLow(GPIOC,GPIO_PIN_4);
  }
  else if(InputData == 0x09)
  {
    GPIO_WriteHigh(GPIOC,GPIO_PIN_7);
    GPIO_WriteLow(GPIOC,GPIO_PIN_6);
    GPIO_WriteLow(GPIOC,GPIO_PIN_5);
    GPIO_WriteHigh(GPIOC,GPIO_PIN_4);
  }
  else if(InputData == 0x0c)
  {	
    GPIO_WriteLow(GPIOC,GPIO_PIN_7);
    GPIO_WriteLow(GPIOC,GPIO_PIN_6);
    GPIO_WriteHigh(GPIOC,GPIO_PIN_5);
    GPIO_WriteHigh(GPIOC,GPIO_PIN_4);
  }
  else if(InputData == 0x00)
  {
    GPIO_WriteLow(GPIOC,GPIO_PIN_7);
    GPIO_WriteLow(GPIOC,GPIO_PIN_6);
    GPIO_WriteLow(GPIOC,GPIO_PIN_5);
    GPIO_WriteLow(GPIOC,GPIO_PIN_4);
  }
}

/*
功能：转1/64圈
步距角5.625 360/5.625=64 减速比1/64
故64*64个脉冲转一圈
n 圈数
direction 方向 1正转 非1反转
delay delay时长 >= 2
*/
void motor_circle(int n, int direction, int delay)
{
  int i, j;
  for(i = 0; i < n * 8; i++)
  {
    for(j = 0; j < 4; j++)
    {
      if(1 == direction)
      {
        SetMotor(0x00);
        SetMotor(forward[j]);
      }
      else
      {
        SetMotor(0x00);
        SetMotor(reverse[j]);
      }

      Delay_ms(delay > 2 ? delay : 2);
      
      /*for(k = 0; k < (delay > 2 ? delay : 2); k++)
      {
        Delay(150 * delay);
      }*/
    }
  }
}

```
## step.h

```c
#ifndef __STEP_H
#define __STEP_H
#include "stm8s_gpio.h"
                        
void Step_Motor_GPIO_Init(void);
/*
	功能：转1/64圈
	步距角5.625 360/5.625=64 减速比1/64
	故64*64个脉冲转一圈
	n 圈数
	direction 方向 1正转 非1反转
	delay delay时长ms >= 2
*/
void motor_circle(int n, int direction, int delay);

#endif

```
## led.c

```c
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
```
## led.h

```c
#ifndef __LED_H
#define __LED_H

#include "stm8s.h"

#define ON 0
#define OFF 1

void LED_Init(void);
void LED_Reverse(void);
void LED_ON_OFF(u8 ON_OFF);

#endif

```

## delay.c

```c
#include "delay.h"

__IO uint32_t TimingDelay = 0;
__IO uint16_t g_Timing_tim4;
/***************************************
*               TIMER1初始化
*           检测外部频率和脉冲
***************************************/
void timer4_init(void)
{
//  disableInterrupts();
  /* TIM4 configuration:
   - TIM4CLK is set to 16 MHz, the TIM4 Prescaler is equal to 128 so the TIM4 counter
   clock used is 16 MHz / 128 = 125 000 Hz
  - With 125 000 Hz we can generate time base:
      max time base is 2.048 ms if TIM4_PERIOD = 255 --> (255 + 1) / 125000 = 2.048 ms
      min time base is 0.016 ms if TIM4_PERIOD = 1   --> (  1 + 1) / 125000 = 0.016 ms
  - In this example we need to generate a time base equal to 1 ms
   so TIM4_PERIOD = (0.001 * 125000 - 1) = 124 */

  /* Time base configuration */
  TIM4_TimeBaseInit(TIM4_PRESCALER_128, 124);
  /* Clear TIM4 update flag */
  TIM4_ClearFlag(TIM4_FLAG_UPDATE);
  /* Enable update interrupt */
  TIM4_ITConfig(TIM4_IT_UPDATE, ENABLE);
  
  /* enable interrupts */
  enableInterrupts();

  /* Enable TIM4 */
  TIM4_Cmd(ENABLE);
  
//  enableInterrupts();
}
void Tim4_Clean(void)
{
    g_Timing_tim4 = 0;
}
/**
  * @brief  Inserts a delay time.
  * @param  nTime: specifies the delay time length, in milliseconds.
  * @retval None
  */
void Delay_ms(__IO uint16_t nTime)
{
  TimingDelay = nTime;

  while (TimingDelay != 0);
}

/**
  * @brief  Decrements the TimingDelay variable.
  * @param  None
  * @retval None
  */
void TimingDelay_Decrement(void)
{
  if (TimingDelay != 0x00)
  {
    TimingDelay--;
  }
}

void Delay(uint16_t nCount)
{
  /* Decrement nCount value */
  while (nCount != 0)
  {   
    nCount--;
  }
}
```
## dealy.h

```c
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

```
## 中断服务函数 stm8s_it.c 中追加
头文件 导入 delay.h

```c
#else /*STM8S208, STM8S207, STM8S105 or STM8S103 or STM8AF52Ax or STM8AF62Ax or STM8AF626x */
/**
  * @brief Timer4 Update/Overflow Interrupt routine.
  * @param  None
  * @retval None
  */
 INTERRUPT_HANDLER(TIM4_UPD_OVF_IRQHandler, 23)
 {
  /* In order to detect unexpected events during development,
     it is recommended to set a breakpoint on the following instruction.
  */
  TimingDelay_Decrement();
  /* Cleat Interrupt Pending bit */
  TIM4_ClearITPendingBit(TIM4_IT_UPDATE);
  g_Timing_tim4++;
  if(g_Timing_tim4>=65535)
  {
    g_Timing_tim4 = 0;
  }
 }
#endif /*STM8S903*/
```

