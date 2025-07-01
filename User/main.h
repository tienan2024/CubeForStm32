/********************************************************************************************************
**                                 Copyright (c)          MEI15
**                                                 All Rights Reserved
**  
**                                 Email:                           QQ:
**-------------------------------------------------------------------------------------------------------
**  MCU        : STM32F103RBT6 (STMicroelectronics)
**  Compiler   : Keil uVision 5.10
**  Module Name: 
**  Module Date: 2016-
**  Module Auth: 
**  Description: 
**  Version    : V1.0
**  Notes      : 
**-------------------------------------------------------------------------------------------------------
**  Version    : 
**  Modified By: 
**  Date       : 
**  Content    : 
**  Notes      : 
********************************************************************************************************/
#ifndef __MAIN_H
#define __MAIN_H

/*-----------------------------------------  I N C L U D E S  -----------------------------------------*/
#include "Global.h"
#include "delay.h"
/*---------------------------------------  D E F I N I T I O N  ---------------------------------------*/
//����
//*//����̵����Ǹߵ�ƽ��������CLOSE=1 <-��ʾ����   OPEN=0��
//�����������
#define R_PUL_UP            GPIO_SetBits(GPIOB, GPIO_Pin_5)
#define R_PUL_DOWN          GPIO_ResetBits(GPIOB, GPIO_Pin_5)
#define L_PUL_UP            GPIO_SetBits(GPIOB, GPIO_Pin_7)
#define L_PUL_DOWN          GPIO_ResetBits(GPIOB, GPIO_Pin_7)

//�����������
#define R_DIR_N             GPIO_SetBits(GPIOB, GPIO_Pin_6)
#define R_DIR_P             GPIO_ResetBits(GPIOB, GPIO_Pin_6)
#define L_DIR_N             GPIO_SetBits(GPIOB, GPIO_Pin_8)
#define L_DIR_P             GPIO_ResetBits(GPIOB, GPIO_Pin_8)

#define     R_AIR_OPEN       GPIO_SetBits(GPIOA, GPIO_Pin_0)          //di��ƽ����
#define     R_AIR_CLOSE      GPIO_ResetBits(GPIOA, GPIO_Pin_0)        //R���
//�̵���23

#define     L_AIR_OPEN       GPIO_SetBits(GPIOA, GPIO_Pin_1)                  //1               ������պ�
#define     L_AIR_CLOSE      GPIO_ResetBits(GPIOA, GPIO_Pin_1)  //�͵�ƽ����  //0             �������/L���
//�̵���13
#define     L_CAP_IN_OPEN    GPIO_SetBits(GPIOA, GPIO_Pin_3)       //L���      �ߵ�ƽ����
#define     L_CAP_IN_CLOSE   GPIO_ResetBits(GPIOA, GPIO_Pin_3)
//�̵���11
#define     L_CAP_OUT_CLOSE  GPIO_SetBits(GPIOB, GPIO_Pin_13);GPIO_ResetBits(GPIOB, GPIO_Pin_10);      //�ߵ�ƽ����
#define     L_CAP_OUT_OPEN   GPIO_SetBits(GPIOB, GPIO_Pin_10);GPIO_ResetBits(GPIOB, GPIO_Pin_13);       //L24
//�̵���12

#define     R_CAP_IN_OPEN    //GPIO_SetBits(GPIOB, GPIO_Pin_0)       //�͵�ƽ����
#define     R_CAP_IN_CLOSE   //GPIO_ResetBits(GPIOB, GPIO_Pin_0)     //R���
//�̵���21
#define     R_CAP_OUT_CLOSE  GPIO_SetBits(GPIOB, GPIO_Pin_1);GPIO_ResetBits(GPIOB, GPIO_Pin_11);     //�͵�ƽ����
#define     R_CAP_OUT_OPEN   GPIO_ResetBits(GPIOB, GPIO_Pin_1);GPIO_SetBits(GPIOB, GPIO_Pin_11);     //R24
//�̵���22

// ��̨й�����������ɵ���״̬
#define LOOP_EXHAUST	    GPIO_ResetBits(GPIOB, GPIO_Pin_1);GPIO_ResetBits(GPIOB, GPIO_Pin_11);GPIO_ResetBits(GPIOB, GPIO_Pin_13);GPIO_ResetBits(GPIOB, GPIO_Pin_10);

#define LED0          GPIO_ResetBits(GPIOC, GPIO_Pin_13)      //�͵�ƽ����
#define LED1          GPIO_SetBits(GPIOC, GPIO_Pin_13)        //R�ŵ�

#define KEY0  GPIO_ReadInputDataBit(GPIOB,GPIO_Pin_4)//��ȡ����0
#define KEY1  GPIO_ReadInputDataBit(GPIOB,GPIO_Pin_3)//��ȡ����1
#define KEY2  GPIO_ReadInputDataBit(GPIOB,GPIO_Pin_2)//��ȡ����2 

/*-------------------------------------------  M A C R O S  -------------------------------------------*/
//==================================˫�۵�ǰ������Ϣ
#define CLOSE               0
#define OPEN                1 

typedef struct
{
	u8 LeftHand;
	u8 RightHand;
	s16 LeftDir; // ����ͳһ����ת���ۼƽǶȱ�־λ 1=90�� 2=180�� 3=270�㣨��ʱ���������߻��Ƹ�λ���׼λ�ã� -1=-90�� -2=-180�� -3=-270��
	s16 RightDir; // ����ͳһ����ת���ۼƽǶȱ�־λ 1=90�� 2=180�� 3=270�㣨��ʱ���������߻��Ƹ�λ���׼λ�ã� -1=-90�� -2=-180�� -3=-270��
}Hand;
/*--------------------------------------  D E C L A R A T I O N  --------------------------------------*/
/* Internal Variable */

/* External Variable */

/* Internal Function */

/* External Function */
extern Hand g_RobotHand;
#endif /*__MAIN_H*/
//===========================================  End Of File  ===========================================//
