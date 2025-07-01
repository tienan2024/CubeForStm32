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
//气缸
//*//如果继电器是高电平触发，则CLOSE=1 <-表示工作   OPEN=0，
//步进电机脉冲
#define R_PUL_UP            GPIO_SetBits(GPIOB, GPIO_Pin_5)
#define R_PUL_DOWN          GPIO_ResetBits(GPIOB, GPIO_Pin_5)
#define L_PUL_UP            GPIO_SetBits(GPIOB, GPIO_Pin_7)
#define L_PUL_DOWN          GPIO_ResetBits(GPIOB, GPIO_Pin_7)

//步进电机方向
#define R_DIR_N             GPIO_SetBits(GPIOB, GPIO_Pin_6)
#define R_DIR_P             GPIO_ResetBits(GPIOB, GPIO_Pin_6)
#define L_DIR_N             GPIO_SetBits(GPIOB, GPIO_Pin_8)
#define L_DIR_P             GPIO_ResetBits(GPIOB, GPIO_Pin_8)

#define     R_AIR_OPEN       GPIO_SetBits(GPIOA, GPIO_Pin_0)          //di电平触发
#define     R_AIR_CLOSE      GPIO_ResetBits(GPIOA, GPIO_Pin_0)        //R充电
//继电器23

#define     L_AIR_OPEN       GPIO_SetBits(GPIOA, GPIO_Pin_1)                  //1               电磁铁闭合
#define     L_AIR_CLOSE      GPIO_ResetBits(GPIOA, GPIO_Pin_1)  //低电平触发  //0             电磁铁打开/L充电
//继电器13
#define     L_CAP_IN_OPEN    GPIO_SetBits(GPIOA, GPIO_Pin_3)       //L充电      高电平触发
#define     L_CAP_IN_CLOSE   GPIO_ResetBits(GPIOA, GPIO_Pin_3)
//继电器11
#define     L_CAP_OUT_CLOSE  GPIO_SetBits(GPIOB, GPIO_Pin_13);GPIO_ResetBits(GPIOB, GPIO_Pin_10);      //高电平触发
#define     L_CAP_OUT_OPEN   GPIO_SetBits(GPIOB, GPIO_Pin_10);GPIO_ResetBits(GPIOB, GPIO_Pin_13);       //L24
//继电器12

#define     R_CAP_IN_OPEN    //GPIO_SetBits(GPIOB, GPIO_Pin_0)       //低电平触发
#define     R_CAP_IN_CLOSE   //GPIO_ResetBits(GPIOB, GPIO_Pin_0)     //R充电
//继电器21
#define     R_CAP_OUT_CLOSE  GPIO_SetBits(GPIOB, GPIO_Pin_1);GPIO_ResetBits(GPIOB, GPIO_Pin_11);     //低电平触发
#define     R_CAP_OUT_OPEN   GPIO_ResetBits(GPIOB, GPIO_Pin_1);GPIO_SetBits(GPIOB, GPIO_Pin_11);     //R24
//继电器22

// 滑台泄气，进入自由调节状态
#define LOOP_EXHAUST	    GPIO_ResetBits(GPIOB, GPIO_Pin_1);GPIO_ResetBits(GPIOB, GPIO_Pin_11);GPIO_ResetBits(GPIOB, GPIO_Pin_13);GPIO_ResetBits(GPIOB, GPIO_Pin_10);

#define LED0          GPIO_ResetBits(GPIOC, GPIO_Pin_13)      //低电平触发
#define LED1          GPIO_SetBits(GPIOC, GPIO_Pin_13)        //R放电

#define KEY0  GPIO_ReadInputDataBit(GPIOB,GPIO_Pin_4)//读取按键0
#define KEY1  GPIO_ReadInputDataBit(GPIOB,GPIO_Pin_3)//读取按键1
#define KEY2  GPIO_ReadInputDataBit(GPIOB,GPIO_Pin_2)//读取按键2 

/*-------------------------------------------  M A C R O S  -------------------------------------------*/
//==================================双臂当前开合信息
#define CLOSE               0
#define OPEN                1 

typedef struct
{
	u8 LeftHand;
	u8 RightHand;
	s16 LeftDir; // 左手统一方向转动累计角度标志位 1=90° 2=180° 3=270°（此时触发放绕线机制复位会标准位置） -1=-90° -2=-180° -3=-270°
	s16 RightDir; // 右手统一方向转动累计角度标志位 1=90° 2=180° 3=270°（此时触发放绕线机制复位会标准位置） -1=-90° -2=-180° -3=-270°
}Hand;
/*--------------------------------------  D E C L A R A T I O N  --------------------------------------*/
/* Internal Variable */

/* External Variable */

/* Internal Function */

/* External Function */
extern Hand g_RobotHand;
#endif /*__MAIN_H*/
//===========================================  End Of File  ===========================================//
