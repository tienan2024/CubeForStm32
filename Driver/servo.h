#ifndef __SERVO_CONTROL_H
#define __SERVO_CONTROL_H

#include "stm32f10x.h"
#include <stdio.h>

// USART2 ��ʼ������
void USART2_Init(void);

// ����һ���ַ�
void USART2_SendChar(char ch);

// �����ַ���
void USART2_SendString(const unsigned char* str);

// �����ʱ�����
void servo_send(int angle);
void servo_init_shun(void);
void servo_init_ni(void);
void servo_ni(int speed,int time_s);
void servo_shun(int speed,int time_s);

int motor_shou(void);
int motor_chu(void);
#endif /* __SERVO_CONTROL_H */
