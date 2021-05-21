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
