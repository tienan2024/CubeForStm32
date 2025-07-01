/********************************************************************************************************
**                                 Copyright (c)          MEI15
**                                                 All Rights Reserved
**  
**                                 Email:                           QQ:
**-------------------------------------------------------------------------------------------------------
**  MCU        : STM32F407VG (STMicroelectronics)
**  Compiler   : Keil uVision 5.10
**  Module Name: 
**  Module Date: 2016-2
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
#ifndef __MOTOR_CONTROL_H
#define __MOTOR_CONTROL_H

/*-----------------------------------------  I N C L U D E S  -----------------------------------------*/
#include "Global.h"
//L_1 = 0
//L_2 = 1
//L_3 = 2
//L_O = 3
//L_C = 4

//R_1 = 5
//R_2 = 6
//R_3 = 7
//R_O = 8
//R_C = 9
/*---------------------------------------  D E F I N I T I O N  ---------------------------------------*/
 	/*
		L_C: 0		R_C: 5
		L_O: 1		R_O: 6
		L_1: 2		L_1: 7
		L_2: 3		L_2: 8
		L_3: 4		L_3: 9
	*/
//===================
#define  M_LC   0
#define  M_LO   1
#define  M_L1   2
#define  M_L2   3
#define  M_L3   4

#define  M_RC   5
#define  M_RO   6
#define  M_R1   7
#define  M_R2   8
#define  M_R3   9
//====================
#define  L_C    0
#define  L_O    1
#define  L_1    2
#define  L_2    3
#define  L_3    4
                
#define  R_C    5
#define  R_O    6
#define  R_1    7
#define  R_2    8
#define  R_3    9
//====================

/*-------------------------------------------  M A C R O S  -------------------------------------------*/


/*--------------------------------------  D E C L A R A T I O N  --------------------------------------*/
/* Internal Variable */

/* External Variable */
extern u16   OpenHand_TIM;//气缸打开
extern u16   CloseHand_TIM;//气缸关闭
/* Internal Function */

/* External Function */
extern void MoveL1(int AcDelay[],int DecDelay[],u16 AcNum,u16 DecNum);
extern void MoveL2(int AcDelay[],int DecDelay[],u16 AcNum,u16 DecNum);
extern void MoveL3(int AcDelay[],int DecDelay[],u16 AcNum,u16 DecNum);
extern void MoveR1(int AcDelay[],int DecDelay[],u16 AcNum,u16 DecNum);
extern void MoveR2(int AcDelay[],int DecDelay[],u16 AcNum,u16 DecNum);
extern void MoveR3(int AcDelay[],int DecDelay[],u16 AcNum,u16 DecNum);
extern void MotorMove(u8 MechStep);
extern void GetTimeCost(void);

/*void CAP_L_OUT();
void CAP_L_IN();
void CAP_R_OUT();
void CAP_R_IN();*/
#endif /*__DELAY_H */
//===========================================  End Of File  ===========================================//

