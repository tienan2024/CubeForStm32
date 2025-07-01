/********************************************************************************************************
**                                 Copyright (c)          MEI15
**                                                 All Rights Reserved
**  
**                                 Email:                     QQ:729527658
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

********************************************************************************************************/
/*-----------------------------------------  I N C L U D E S  -----------------------------------------*/
#include "MotorControl.h"
#include "main.h"
/*---------------------------------------  D E F I N I T I O N  ---------------------------------------*/
const int MotorSub = 1600;  //�������ϸ����
#define KZ90FY   402 // ����
#define KZ180FY  820 // ����
#define KZ270FY  1220 // ����
#define ND90FY   402 // ����
#define ND180FY  802 // ����
#define ND270FY  1220 // ����
#define DD90FY   420 // ����
#define DD180FY  820 // ����
#define DD270FY  1220 // ����
extern char close_after_turn_flag;
extern char open_after_turn_flag;
extern char open_after_close_flag;
//====================================================�Ӽ������߲���
////��ת90��
//#define KZ90AcNum  98
//int KZAc90[98]={
//93,75,63,55,50,45,42,39,36,34,33,31,30,28,27,26,25,25,24,23,22,22,21,
//21,20,20,19,19,19,18,18,18,17,17,17,16,16,16,16,15,15,15,15,15,14,14,14,14,
//14,14,13,13,13,13,13,13,13,12,12,12,12,12,12,12,12,12,12,11,11,11,11,11,11,
//11,11,11,11,11,11,10,10,10,10,10,10,10,10,10,10,10,10,10,10,9,9,8,8,8
//};

#define KZ90AcNum  98
int KZAc90[98] = {
88,49,38,31,27,25,24,23,22,21,
20,19,18,17,17,16,16,15,15,14,
14,13,13,13,12,12,12,11,11,11,
11,10,10,10,10,9,9,9,9,9,
9,9,8,8,8,8,8,8,8,8,
8,8,8,8,7,7,7,7,7,7,
7,7,7,7,7,7,7,7,7,7,
7,7,7,6,6,6,6,6,6,6,
6,6,6,6,6,6,6,6,6,6,
6,6,6,6,6,6,6,6
};

//#define KZ90DecNum  98
//int KZDec90[98]={
//93,75,63,55,50,45,42,39,36,34,33,31,30,28,27,26,25,25,24,23,22,22,21,
//21,20,20,19,19,19,18,18,18,17,17,17,16,16,16,16,15,15,15,15,15,14,14,14,14,
//14,14,13,13,13,13,13,13,13,12,12,12,12,12,12,12,12,12,12,11,11,11,11,11,11,
//11,11,11,11,11,11,10,10,10,10,10,10,10,10,10,10,10,10,10,10,9,9,8,8,8
//};

#define KZ90DecNum  98  
int KZDec90[98] = {
88,49,38,31,27,25,24,23,22,21,
20,19,18,17,17,16,16,15,15,14,
14,13,13,13,12,12,12,11,11,11,
11,10,10,10,10,9,9,9,9,9,
9,9,8,8,8,8,8,8,8,8,
8,8,8,8,7,7,7,7,7,7,
7,7,7,7,7,7,7,7,7,7,
7,7,7,6,6,6,6,6,6,6,
6,6,6,6,6,6,6,6,6,6,
6,6,6,6,6,6,6,6
};

//====================================================

////��ת180��
//#define KZ180AcNum  98
//int KZAc180[98]={
//93,75,63,55,50,45,42,39,36,34,33,31,30,28,27,26,25,25,24,23,22,22,21,
//21,20,20,19,19,19,18,18,18,17,17,17,16,16,16,16,15,15,15,15,15,14,14,14,14,
//14,14,13,13,13,13,13,13,13,12,12,12,12,12,12,12,12,12,12,11,11,11,11,11,11,
//11,11,11,11,11,11,10,10,10,10,10,10,10,10,10,10,10,10,10,10,9,9,8,8,8
//};

//��ת180��
#define KZ180AcNum  98
int KZAc180[98]={
88,49,38,31,27,25,24,23,22,21,
20,19,18,17,17,16,16,15,15,14,
14,13,13,13,12,12,12,11,11,11,
11,10,10,10,10,9,9,9,9,9,
9,9,8,8,8,8,8,8,8,8,
8,8,8,8,7,7,7,7,7,7,
7,7,7,7,7,7,7,7,7,7,
7,7,7,6,6,6,6,6,6,6,
6,6,6,6,6,6,6,6,6,6,
6,6,6,6,6,6,6,6
};

//#define KZ180DecNum  98
//int KZDec180[98]={
//93,75,63,55,50,45,42,39,36,34,33,31,30,28,27,26,25,25,24,23,22,22,21,
//21,20,20,19,19,19,18,18,18,17,17,17,16,16,16,16,15,15,15,15,15,14,14,14,14,
//14,14,13,13,13,13,13,13,13,12,12,12,12,12,12,12,12,12,12,11,11,11,11,11,11,
//11,11,11,11,11,11,10,10,10,10,10,10,10,10,10,10,10,10,10,10,9,9,8,8,8
//};

#define KZ180DecNum  98
int KZDec180[98]={
88,49,38,31,27,25,24,23,22,21,
20,19,18,17,17,16,16,15,15,14,
14,13,13,13,12,12,12,11,11,11,
11,10,10,10,10,9,9,9,9,9,
9,9,8,8,8,8,8,8,8,8,
8,8,8,8,7,7,7,7,7,7,
7,7,7,7,7,7,7,7,7,7,
7,7,7,6,6,6,6,6,6,6,
6,6,6,6,6,6,6,6,6,6,
6,6,6,6,6,6,6,6
};

//====================================================

////š��90��
//#define ND90AcNum  108
//int NDAc90[108]={
//91,73,62,54,48,44,41,38,35,33,32,30,29,28,27,26,25,24,23,22,22,21,21,
//20,20,19,19,18,18,18,17,17,17,16,16,16,16,15,15,15,15,15,14,14,14,14,14,13,
//13,13,13,13,13,13,12,12,12,12,12,12,12,12,12,11,11,11,11,11,11,11,11,11,11,
//11,10,10,10,10,10,10,10,10,10,10,10,10,10,10,10,10,10,10,10,10,10,10,10,10,
//9,9,9,9,9,9,9,9,9,9
//};

//š��90��
#define ND90AcNum  108
int NDAc90[108]={
95,85,77,68,61,55,50,44,38,33,
26,24,24,23,22,21,20,19,18,16,
14,14,13,13,12,12,11,11,10,10,
9,9,8,8,8,7,7,7,6,6,
6,6,6,6,6,6,6,6,6,6,
6,6,6,6,6,6,6,6,6,6,
6,6,6,6,6,6,6,6,6,6,
6,6,6,6,6,6,6,6,6,6,
6,6,6,6,6,6,6,6,6,6,
6,6,6,6,6,6,6,6,6,6,
6,6,6,6,6,6,6,6
};

//#define ND90DecNum  108
//int NDDec90[108]={
//91,73,62,54,48,44,41,38,35,33,32,30,29,28,27,26,25,24,23,22,22,21,21,
//20,20,19,19,18,18,18,17,17,17,16,16,16,16,15,15,15,15,15,14,14,14,14,14,13,
//13,13,13,13,13,13,12,12,12,12,12,12,12,12,12,11,11,11,11,11,11,11,11,11,11,
//11,10,10,10,10,10,10,10,10,10,10,10,10,10,10,10,10,10,10,10,10,10,10,10,10,
//9,9,9,9,9,9,9,9,9,9
//};

#define ND90DecNum  108
int NDDec90[108]={
95,85,77,68,61,55,50,44,38,33,
26,24,24,23,22,21,20,19,18,16,
14,14,13,13,12,12,11,11,10,10,
9,9,8,8,8,7,7,7,6,6,
6,6,6,6,6,6,6,6,6,6,
6,6,6,6,6,6,6,6,6,6,
6,6,6,6,6,6,6,6,6,6,
6,6,6,6,6,6,6,6,6,6,
6,6,6,6,6,6,6,6,6,6,
6,6,6,6,6,6,6,6,6,6,
6,6,6,6,6,6,6,6

};

//====================================================

////š��180��
//#define ND180AcNum  108
//int NDAc180[108]={
//91,73,62,54,48,44,41,38,35,33,32,30,29,28,27,26,25,24,23,22,22,21,21,
//20,20,19,19,18,18,18,17,17,17,16,16,16,16,15,15,15,15,15,14,14,14,14,14,13,
//13,13,13,13,13,13,12,12,12,12,12,12,12,12,12,11,11,11,11,11,11,11,11,11,11,
//11,10,10,10,10,10,10,10,10,10,10,10,10,10,10,10,10,10,10,10,10,10,10,10,10,
//9,9,9,9,9,9,9,9,9,9
//};

//š��180��
#define ND180AcNum  108
int NDAc180[108]={
93,33,29,26,24,23,22,21,20,19,
18,17,17,16,15,15,14,14,13,13,
13,12,12,12,11,11,11,10,10,10,
9,9,9,9,8,8,8,8,7,7,
7,7,7,6,6,6,6,6,6,6,
6,6,6,6,6,6,6,6,6,6,
6,6,6,6,6,6,6,6,6,6,
6,6,6,6,6,6,6,6,6,6,
6,6,6,6,6,6,6,6,6,6,
6,6,6,6,6,6,6,6,6,6,
6,6,6,6,6,6,6,6
};

//#define ND180DecNum  108
//int NDDec180[108]={
//91,73,62,54,48,44,41,38,35,33,32,30,29,28,27,26,25,24,23,22,22,21,21,
//20,20,19,19,18,18,18,17,17,17,16,16,16,16,15,15,15,15,15,14,14,14,14,14,13,
//13,13,13,13,13,13,12,12,12,12,12,12,12,12,12,11,11,11,11,11,11,11,11,11,11,
//11,10,10,10,10,10,10,10,10,10,10,10,10,10,10,10,10,10,10,10,10,10,10,10,10,
//9,9,9,9,9,9,9,9,9,9
//};

#define ND180DecNum  108
int NDDec180[108]={
93,33,29,26,24,23,22,21,20,19,
18,17,17,16,15,15,14,14,13,13,
13,12,12,12,11,11,11,10,10,10,
9,9,9,9,8,8,8,8,7,7,
7,7,7,6,6,6,6,6,6,6,
6,6,6,6,6,6,6,6,6,6,
6,6,6,6,6,6,6,6,6,6,
6,6,6,6,6,6,6,6,6,6,
6,6,6,6,6,6,6,6,6,6,
6,6,6,6,6,6,6,6,6,6,
6,6,6,6,6,6,6,6
};

//====================================================

//����90��
#define DD90AcNum  183
int DDAc90[183]={
96,91,87,83,80,77,74,72,70,68,66,64,62,
61,59,58,56,55,54,53,52,51,50,49,48,47,47,46,45,45,44,43,43,42,41,41,40,40,
39,39,38,38,38,37,37,36,36,36,35,35,35,34,34,34,33,33,33,32,32,32,32,31,31,
31,31,30,30,30,30,29,29,29,29,29,28,28,28,28,28,27,27,27,27,27,27,26,26,26,
26,26,26,26,25,25,25,25,25,25,25,24,24,24,24,24,24,24,24,24,23,23,23,23,23,
23,23,23,23,22,22,22,22,22,22,22,22,22,22,22,21,21,21,21,21,21,21,21,21,21,
21,21,20,20,20,20,20,20,20,20,20,20,20,20,20,20,20,19,19,19,19,19,19,19,19,
19,19,19,19,19,19,19,19,19,18,18,18,18,18,18,18,18,18,18,18
};

////����90��
//#define DD90AcNum  183
//int DDAc90[183]={
//93,90,88,86,84,83,81,79,77,75,
//74,72,70,69,67,66,64,63,62,60,
//59,57,56,55,54,52,51,50,49,48,
//47,46,45,44,43,42,41,40,39,38,
//37,36,36,35,34,33,32,32,31,30,
//30,29,28,28,27,26,26,25,25,24,
//24,23,23,22,21,21,21,20,20,19,
//19,18,18,17,17,17,16,16,16,15,
//15,15,15,15,15,15,15,15,15,15,
//15,15,15,15,15,15,15,15,15,15,
//15,15,15,15,15,15,15,15,15,15,
//15,15,15,15,15,15,15,15,15,15,
//15,15,15,15,15,15,15,15,15,15,
//15,15,15,15,15,15,15,15,15,15,
//15,15,15,15,15,15,15,15,15,15,
//15,15,15,15,15,15,15,15,15,15,
//15,15,15,15,15,15,15,15,15,15,
//15,15,15,15,15,15,15,15,15,15,
//15,15,15
//};

#define DD90DecNum  183
int DDDec90[183]={
96,91,87,83,80,77,74,72,70,68,66,64,62,
61,59,58,56,55,54,53,52,51,50,49,48,47,47,46,45,45,44,43,43,42,41,41,40,40,
39,39,38,38,38,37,37,36,36,36,35,35,35,34,34,34,33,33,33,32,32,32,32,31,31,
31,31,30,30,30,30,29,29,29,29,29,28,28,28,28,28,27,27,27,27,27,27,26,26,26,
26,26,26,26,25,25,25,25,25,25,25,24,24,24,24,24,24,24,24,24,23,23,23,23,23,
23,23,23,23,22,22,22,22,22,22,22,22,22,22,22,21,21,21,21,21,21,21,21,21,21,
21,21,20,20,20,20,20,20,20,20,20,20,20,20,20,20,20,19,19,19,19,19,19,19,19,
19,19,19,19,19,19,19,19,19,18,18,18,18,18,18,18,18,18,18,18
};

//#define DD90DecNum  183
//int DDDec90[183]={
//93,90,88,86,84,83,81,79,77,75,
//74,72,70,69,67,66,64,63,62,60,
//59,57,56,55,54,52,51,50,49,48,
//47,46,45,44,43,42,41,40,39,38,
//37,36,36,35,34,33,32,32,31,30,
//30,29,28,28,27,26,26,25,25,24,
//24,23,23,22,21,21,21,20,20,19,
//19,18,18,17,17,17,16,16,16,15,
//15,15,15,15,15,15,15,15,15,15,
//15,15,15,15,15,15,15,15,15,15,
//15,15,15,15,15,15,15,15,15,15,
//15,15,15,15,15,15,15,15,15,15,
//15,15,15,15,15,15,15,15,15,15,
//15,15,15,15,15,15,15,15,15,15,
//15,15,15,15,15,15,15,15,15,15,
//15,15,15,15,15,15,15,15,15,15,
//15,15,15,15,15,15,15,15,15,15,
//15,15,15,15,15,15,15,15,15,15,
//15,15,15
//};

//====================================================

//����180��
#define DD180AcNum  183
int DDAc180[183]={
96,91,87,83,80,77,74,72,70,68,66,64,62,
61,59,58,56,55,54,53,52,51,50,49,48,47,47,46,45,45,44,43,43,42,41,41,40,40,
39,39,38,38,38,37,37,36,36,36,35,35,35,34,34,34,33,33,33,32,32,32,32,31,31,
31,31,30,30,30,30,29,29,29,29,29,28,28,28,28,28,27,27,27,27,27,27,26,26,26,
26,26,26,26,25,25,25,25,25,25,25,24,24,24,24,24,24,24,24,24,23,23,23,23,23,
23,23,23,23,22,22,22,22,22,22,22,22,22,22,22,21,21,21,21,21,21,21,21,21,21,
21,21,20,20,20,20,20,20,20,20,20,20,20,20,20,20,20,19,19,19,19,19,19,19,19,
19,19,19,19,19,19,19,19,19,18,18,18,18,18,18,18,18,18,18,18
};

////����180��
//#define DD180AcNum  183
//int DDAc180[183]={
//95,90,86,82,79,76,73,71,69,67,
//65,63,61,60,58,57,55,54,53,52,
//51,50,49,48,47,46,46,45,44,44,
//43,42,42,41,40,40,39,39,38,38,
//37,37,37,36,36,35,35,35,34,34,
//34,33,33,33,32,32,32,31,31,31,
//31,30,30,30,30,29,29,29,29,28,
//28,28,28,28,27,27,27,27,27,26,
//26,26,26,26,26,25,25,25,25,25,
//25,25,24,24,24,24,24,24,24,23,
//23,23,23,23,23,23,23,23,22,22,
//22,22,22,22,22,22,22,21,21,21,
//21,21,21,21,21,21,21,21,20,20,
//20,20,20,20,20,20,20,20,20,20,
//19,19,19,19,19,19,19,19,19,19,
//19,19,19,19,19,18,18,18,18,18,
//18,18,18,18,18,18,18,18,18,18,
//18,18,17,17,17,17,17,17,17,17,
//17,17,15
//};

#define DD180DecNum  183
int DDDec180[183]={
96,91,87,83,80,77,74,72,70,68,66,64,62,
61,59,58,56,55,54,53,52,51,50,49,48,47,47,46,45,45,44,43,43,42,41,41,40,40,
39,39,38,38,38,37,37,36,36,36,35,35,35,34,34,34,33,33,33,32,32,32,32,31,31,
31,31,30,30,30,30,29,29,29,29,29,28,28,28,28,28,27,27,27,27,27,27,26,26,26,
26,26,26,26,25,25,25,25,25,25,25,24,24,24,24,24,24,24,24,24,23,23,23,23,23,
23,23,23,23,22,22,22,22,22,22,22,22,22,22,22,21,21,21,21,21,21,21,21,21,21,
21,21,20,20,20,20,20,20,20,20,20,20,20,20,20,20,20,19,19,19,19,19,19,19,19,
19,19,19,19,19,19,19,19,19,18,18,18,18,18,18,18,18,18,18,18
};

//#define DD180DecNum  183
//int DDDec180[183]={
//95,90,86,82,79,76,73,71,69,67,
//65,63,61,60,58,57,55,54,53,52,
//51,50,49,48,47,46,46,45,44,44,
//43,42,42,41,40,40,39,39,38,38,
//37,37,37,36,36,35,35,35,34,34,
//34,33,33,33,32,32,32,31,31,31,
//31,30,30,30,30,29,29,29,29,28,
//28,28,28,28,27,27,27,27,27,26,
//26,26,26,26,26,25,25,25,25,25,
//25,25,24,24,24,24,24,24,24,23,
//23,23,23,23,23,23,23,23,22,22,
//22,22,22,22,22,22,22,21,21,21,
//21,21,21,21,21,21,21,21,20,20,
//20,20,20,20,20,20,20,20,20,20,
//19,19,19,19,19,19,19,19,19,19,
//19,19,19,19,19,18,18,18,18,18,
//18,18,18,18,18,18,18,18,18,18,
//18,18,17,17,17,17,17,17,17,17,
//17,17,15
//};

//�Ӽ��ٹ���
 void MoveL1(int AcDelay[],int DecDelay[],u16 AcNum,u16 DecNum)
{
	  int i;
	  int totalPulse;
	  //int FY_Num;
	
	
		if(g_RobotHand.LeftDir<3)//������
		{
			g_RobotHand.LeftDir+=1; 
			L_DIR_N;
			totalPulse=MotorSub/4;
		}
	  else
		{	
			// ����������У׼��ı�׼λ���ۼ�����ת�ﵽ270������һ��ת��ǰ
			//��Left/right Dir =3/=-3������ת270��ص���׼λ��g_RobotHand.LeftDir-=3;
			L_DIR_P;				
			totalPulse=MotorSub*3/4;
		}
	  delay_5us(20);
	  
		for(i=0;i<AcNum;i++)                     //���ٽ׶�
		{
			L_PUL_UP;
			delay_5us((int)(AcDelay[i]));
			L_PUL_DOWN;
			delay_5us((int)(AcDelay[i]));
		}
		
		for(i=AcNum;i<totalPulse-DecNum;i++)           //���ٽ׶�
		{
			L_PUL_UP;
			delay_5us(AcDelay[AcNum-1]);
			L_PUL_DOWN;
			delay_5us(AcDelay[AcNum-1]);
		}

		for(i=totalPulse-DecNum;i<totalPulse;i++)  //���ٽ׶�
		{
			L_PUL_UP;
			delay_5us((int)(DecDelay[totalPulse-i-1]));
			L_PUL_DOWN;
			delay_5us((int)(DecDelay[totalPulse-i-1]));
		}
}
void MoveL2(int AcDelay[],int DecDelay[],u16 AcNum,u16 DecNum)
{
	  int i;
	  int totalPulse;
	  //int FY_Num=800;

		if(g_RobotHand.LeftDir<0)//������
		{
			g_RobotHand.LeftDir+=2; 
			L_DIR_N;
			totalPulse=MotorSub/2;
		}
	  else
		{	
			g_RobotHand.LeftDir-=2; 
			L_DIR_P;
			totalPulse=MotorSub/2;
		}
	  delay_5us(20);
	

		for(i=0;i<AcNum;i++)                     //���ٽ׶�
		{
			L_PUL_UP;
			delay_5us((int)(AcDelay[i]));
			L_PUL_DOWN;
			delay_5us((int)(AcDelay[i]));
		}
		
		for(i=AcNum;i<totalPulse-DecNum;i++)           //���ٽ׶�
		{
			L_PUL_UP;
			delay_5us(AcDelay[AcNum-1]);
			L_PUL_DOWN;
			delay_5us(AcDelay[AcNum-1]);
		}

		for(i=totalPulse-DecNum;i<totalPulse;i++)  //���ٽ׶�
		{
			L_PUL_UP;
			delay_5us((int)(DecDelay[totalPulse-i-1]));
			L_PUL_DOWN;
			delay_5us((int)(DecDelay[totalPulse-i-1]));
		}
		
}
void MoveL3(int AcDelay[],int DecDelay[],u16 AcNum,u16 DecNum)
{
	  int i;
	  int totalPulse;
	  //int FY_Num;
	
		if(g_RobotHand.LeftDir >-3)//������
		{
			g_RobotHand.LeftDir-=1; 
			L_DIR_P;
			totalPulse=MotorSub/4;
		}
	  else
		{	
			g_RobotHand.LeftDir+=3; 
			L_DIR_N;
			totalPulse=MotorSub/4*3;
		}
	  delay_5us(20);
		
	  		for(i=0;i<AcNum;i++)                     //���ٽ׶�
		{
			L_PUL_UP;
			delay_5us((int)(AcDelay[i]));
			L_PUL_DOWN;
			delay_5us((int)(AcDelay[i]));
		}
		
		for(i=AcNum;i<totalPulse-DecNum;i++)           //���ٽ׶�
		{
			L_PUL_UP;
			delay_5us(AcDelay[AcNum-1]);
			L_PUL_DOWN;
			delay_5us(AcDelay[AcNum-1]);
		}

		for(i=totalPulse-DecNum;i<totalPulse;i++)  //���ٽ׶�
		{
			L_PUL_UP;
			delay_5us((int)(DecDelay[totalPulse-i-1]));
			L_PUL_DOWN;
			delay_5us((int)(DecDelay[totalPulse-i-1]));
		}	
}
void MoveR1(int AcDelay[],int DecDelay[],u16 AcNum,u16 DecNum)
{
	  int i;
	  int totalPulse;
	  //int FY_Num;
		if(g_RobotHand.RightDir<3)//������
		{
			g_RobotHand.RightDir+=1; 
			R_DIR_N;
			totalPulse=MotorSub/4;
		}
	  else
		{	
			g_RobotHand.RightDir-=3; 
		    R_DIR_P;
			totalPulse=MotorSub*3/4;
		}
	  delay_5us(40);
		for(i=0;i<AcNum;i++)                     //���ٽ׶�
		{
			R_PUL_UP;
			delay_5us((int)(AcDelay[i]));
			R_PUL_DOWN;
			delay_5us((int)(AcDelay[i]));
		}
		
		for(i=AcNum;i<totalPulse-DecNum;i++)           //���ٽ׶�
		{
			R_PUL_UP;
			delay_5us(AcDelay[AcNum-1]);
			R_PUL_DOWN;
			delay_5us(AcDelay[AcNum-1]);
		}

		for(i=totalPulse-DecNum;i<totalPulse;i++)  //���ٽ׶�
		{
			R_PUL_UP;
			delay_5us((int)(DecDelay[totalPulse-i-1]));
			R_PUL_DOWN;
			delay_5us((int)(DecDelay[totalPulse-i-1]));
		}
}
void MoveR2(int AcDelay[],int DecDelay[],u16 AcNum,u16 DecNum)
{
	  int i;
	  int totalPulse;
	  //int FY_Num=800;
	 
		if(g_RobotHand.RightDir<0)//������
		{
			g_RobotHand.RightDir+=2; 
			R_DIR_N;
			totalPulse=MotorSub/2;
		}
	  else
		{	
			g_RobotHand.RightDir-=2; 
		    R_DIR_P;
			totalPulse=MotorSub/2;
		}
	  delay_5us(20);
     for(i=0;i<AcNum;i++)                     //���ٽ׶�
		{
			R_PUL_UP;
			delay_5us((int)(AcDelay[i]));
			R_PUL_DOWN;
			delay_5us((int)(AcDelay[i]));
		}
		
		for(i=AcNum;i<totalPulse-DecNum;i++)           //���ٽ׶�
		{
			R_PUL_UP;
			delay_5us(AcDelay[AcNum-1]);
			R_PUL_DOWN;
			delay_5us(AcDelay[AcNum-1]);
		}

		for(i=totalPulse-DecNum;i<totalPulse;i++)  //���ٽ׶�
		{
			R_PUL_UP;
			delay_5us((int)(DecDelay[totalPulse-i-1]));
			R_PUL_DOWN;
			delay_5us((int)(DecDelay[totalPulse-i-1]));
		}
}
void MoveR3(int AcDelay[],int DecDelay[],u16 AcNum,u16 DecNum)
{
	  int i;
	  int totalPulse;
	  //int FY_Num;
		if(g_RobotHand.RightDir>-3)//������
		{
			g_RobotHand.RightDir-=1; 
			R_DIR_P;
			totalPulse=MotorSub/4;
		}
	  else
		{	
			g_RobotHand.RightDir+=3; 
		  R_DIR_N;
			totalPulse=MotorSub*3/4;
		}
	  delay_5us(20);
	 for(i=0;i<AcNum;i++)                     //���ٽ׶�
		{
			R_PUL_UP;
			delay_5us((int)(AcDelay[i]));
			R_PUL_DOWN;
			delay_5us((int)(AcDelay[i]));
		}
		
		for(i=AcNum;i<totalPulse-DecNum;i++)           //���ٽ׶�
		{
			R_PUL_UP;
			delay_5us(AcDelay[AcNum-1]);
			R_PUL_DOWN;
			delay_5us(AcDelay[AcNum-1]);
		}

		for(i=totalPulse-DecNum;i<totalPulse;i++)  //���ٽ׶�
		{
			R_PUL_UP;
			delay_5us((int)(DecDelay[totalPulse-i-1]));
			R_PUL_DOWN;
			delay_5us((int)(DecDelay[totalPulse-i-1]));
		}
}

void MotorMove(u8 MechStep) // ��ָ����˶�
{
	/*
		L_C: 0		R_C: 5
		L_O: 1		R_O: 6
		L_1: 2		R_1: 7
		L_2: 3		R_2: 8
		L_3: 4		R_3: 9
	*/
	u16 AcNum=0;
	u16 DecNum=0;
	int *AcDelay;
	int *DecDelay;
  //��ת90
	if(
		(((MechStep==L_1)||(MechStep==L_3)) && (g_RobotHand.LeftHand==OPEN))||
	    (((MechStep==R_1)||(MechStep==R_3)) && (g_RobotHand.RightHand==OPEN))
    )
	{
		AcNum = KZ90AcNum;
		DecNum = KZ90DecNum;
		AcDelay=KZAc90;
		DecDelay=KZDec90;
	}
  //��ת180
	if(
		((MechStep==L_2) && (g_RobotHand.LeftHand==OPEN))||
	    ((MechStep==R_2) && (g_RobotHand.RightHand==OPEN))
    )
	{
  		AcNum = KZ180AcNum;
		DecNum = KZ180DecNum;
		AcDelay=KZAc180;
		DecDelay=KZDec180;
	}
	//š��90
	else if(
         ((MechStep==L_1)||(MechStep==L_3)||(MechStep==R_1)||(MechStep==R_3))&&
         ((g_RobotHand.RightHand==CLOSE)&&(g_RobotHand.LeftHand==CLOSE))
         )
	{
		AcNum = ND90AcNum;
		DecNum = ND90DecNum;
		AcDelay=NDAc90;
		DecDelay=NDDec90;
	}
	//š��180
	else if(
         ((MechStep==L_2)||(MechStep==R_2))&&
         ((g_RobotHand.RightHand==CLOSE)&&(g_RobotHand.LeftHand==CLOSE))
         )
	{
		AcNum = ND180AcNum;
		DecNum = ND180DecNum;
		AcDelay=NDAc180;
		DecDelay=NDDec180;
	}
	//����90
	else if(
		     (((MechStep==L_1)||(MechStep==L_3)) && ((g_RobotHand.LeftHand==CLOSE)&&(g_RobotHand.RightHand==OPEN)))||
	         (((MechStep==R_1)||(MechStep==R_3)) && ((g_RobotHand.RightHand==CLOSE)&&(g_RobotHand.LeftHand==OPEN)))
         )
	{
		AcNum = DD90AcNum;
		DecNum = DD90DecNum;
		AcDelay=DDAc90;
		DecDelay=DDDec90;
	}
	//����180
	else if(
		     ((MechStep==L_2) && ((g_RobotHand.LeftHand==CLOSE)&&(g_RobotHand.RightHand==OPEN)))||
	         ((MechStep==R_2) && ((g_RobotHand.RightHand==CLOSE)&&(g_RobotHand.LeftHand==OPEN)))
         )
	{
		AcNum = DD180AcNum;
		DecNum = DD180DecNum;
		AcDelay=DDAc180;
		DecDelay=DDDec180;
	}  

	switch(MechStep)
	{
		case L_C:
		{
			L_CAP_OUT_CLOSE;     
			delay_5us2(12000);  //��̨��£����Ҫ��ʱ��t2
			g_RobotHand.LeftHand=CLOSE;		
			break;
		}
		case L_O:
		{ 
		    L_CAP_OUT_OPEN;   
		    delay_5us2(12000);  //��е��ץ��ʱ��t2   			
		    g_RobotHand.LeftHand=OPEN;
		    break;
		}
		case L_1:MoveL1(AcDelay,DecDelay,AcNum,DecNum);break;
		case L_2:MoveL2(AcDelay,DecDelay,AcNum,DecNum);break;
		case L_3:MoveL3(AcDelay,DecDelay,AcNum,DecNum);break;
		
		case R_C:
		{
			R_CAP_OUT_CLOSE;  
			delay_5us2(12000);  //��е��ץ��ʱ��t2   
			g_RobotHand.RightHand=CLOSE;		
			break;
		}
		case R_O:
		{		
			R_CAP_OUT_OPEN; 
			delay_5us2(12000);  //��е��ץ��ʱ��t2   			
		    g_RobotHand.RightHand=OPEN;
			break;
		}
		case R_1:MoveR1(AcDelay,DecDelay,AcNum,DecNum);break;
		case R_2:MoveR2(AcDelay,DecDelay,AcNum,DecNum);break;
		case R_3:MoveR3(AcDelay,DecDelay,AcNum,DecNum);break;
		default:break;
	}
}

//====================================================���ڲⶨÿ�����ڵĺ�ʱ
int TimeCost[5]={0,0,0,0,0};
void GetTimeCost(void)
{
	 int i;
	 int totalPulse;
	 int AcNum;
	 int DecNum;
	 int* AcDelay;
	 int* DecDelay;
     int j;
	//�����ת90��ʱ
	totalPulse=MotorSub/4;
	AcNum=KZ90AcNum;
	DecNum=KZ90DecNum;
	AcDelay=KZAc90;
	DecDelay=KZDec90;
	j=0;
		for(i=0;i<AcNum;i++)                     //���ٽ׶�
		{
			TimeCost[j]+=AcDelay[i];
		}
		for(i=AcNum;i<totalPulse-DecNum;i++)           //���ٽ׶�
		{
			TimeCost[j]+=AcDelay[AcNum-1];
		}
		for(i=totalPulse-DecNum;i<totalPulse;i++)  //���ٽ׶�
		{
			TimeCost[j]+=DecDelay[totalPulse-i-1];
		}
	//����š��90��ʱ
	totalPulse=MotorSub/4;
	AcNum=ND90AcNum;
	DecNum=ND90DecNum;
	AcDelay=NDAc90;
	DecDelay=NDDec90;
	j=1;
		for(i=0;i<AcNum;i++)                     //���ٽ׶�
		{
			TimeCost[j]+=AcDelay[i];
		}
		for(i=AcNum;i<totalPulse-DecNum;i++)           //���ٽ׶�
		{
			TimeCost[j]+=AcDelay[AcNum-1];
		}
		for(i=totalPulse-DecNum;i<totalPulse;i++)  //���ٽ׶�
		{
			TimeCost[j]+=DecDelay[totalPulse-i-1];
		}
			//����š��180��ʱ
	totalPulse=MotorSub/2;
	AcNum=ND180AcNum;
	DecNum=ND180DecNum;
	AcDelay=NDAc180;
	DecDelay=NDDec180;
	j=2;
		for(i=0;i<AcNum;i++)                     //���ٽ׶�
		{
			TimeCost[j]+=AcDelay[i];
		}
		for(i=AcNum;i<totalPulse-DecNum;i++)           //���ٽ׶�
		{
			TimeCost[j]+=AcDelay[AcNum-1];
		}
		for(i=totalPulse-DecNum;i<totalPulse;i++)  //���ٽ׶�
		{
			TimeCost[j]+=DecDelay[totalPulse-i-1];
		}
					//�������90��ʱ
	totalPulse=MotorSub/4;
	AcNum=DD90AcNum;
	DecNum=DD90DecNum;
	AcDelay=DDAc90;
	DecDelay=DDDec90;
	j=3;
		for(i=0;i<AcNum;i++)                     //���ٽ׶�
		{
			TimeCost[j]+=AcDelay[i];
		}
		for(i=AcNum;i<totalPulse-DecNum;i++)           //���ٽ׶�
		{
			TimeCost[j]+=AcDelay[AcNum-1];
		}
		for(i=totalPulse-DecNum;i<totalPulse;i++)  //���ٽ׶�
		{
			TimeCost[j]+=DecDelay[totalPulse-i-1];
		}
							//�������180��ʱ
	totalPulse=MotorSub/2;
	AcNum=DD180AcNum;
	DecNum=DD180DecNum;
	AcDelay=DDAc180;
	DecDelay=DDDec180;
	j=4;
		for(i=0;i<AcNum;i++)                     //���ٽ׶�
		{
			TimeCost[j]+=AcDelay[i];
		}
		for(i=AcNum;i<totalPulse-DecNum;i++)           //���ٽ׶�
		{
			TimeCost[j]+=AcDelay[AcNum-1];
		}
		for(i=totalPulse-DecNum;i<totalPulse;i++)  //���ٽ׶�
		{
			TimeCost[j]+=DecDelay[totalPulse-i-1];
		}
		
		//���
		for(i=0;i<5;i++)
		{
			TimeCost[i]/=100;
		}
		
}
//===========================================  End Of File  ===========================================//
