#ifndef __STEP_H
#define __STEP_H
#include "stm8s_gpio.h"
                        
void Step_Motor_GPIO_Init(void);
/*
	���ܣ�ת1/64Ȧ
	�����5.625 360/5.625=64 ���ٱ�1/64
	��64*64������תһȦ
	n Ȧ��
	direction ���� 1��ת ��1��ת
	delay delayʱ��ms >= 2
*/
void motor_circle(int n, int direction, int delay);

#endif
