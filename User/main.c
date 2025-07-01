/*******************************************************************************************************
*һЩ���飺
*1����λ�����ٽ��������Ż� ���� ���ߡ�
*2�����ڷ����Ƶ��Ż���ͨ�����Է������е���λ����ħ���㷨���ÿ��չ��40�����ͻᴥ��һ�η����ƻ��ƣ�
*   �������ǿ���ͨ���Ż����ܲ��������Ӵ��������Ƶ���ֵ�Ӷ��������岽�衣����������2%����
*3������δ���ķ�չ������Ϊħ�������������ᣬ���ԶԵ����Ť��Ҫ�󲻸ߣ����������ڵĽṹ�����صĽ�����̨����ζ�ŵ����Ť�ز���̫�ͣ�
*   Ҳ����˵�����ת�ٲ���̫�죬��Ȼ������ɵ��ۼ�����ʹ������Ļ�ԭħ����ò�̫���ܡ�������Ӧ����С�ͻ�����
*******************************************************************************************************/
#include "main.h"
#include "oled.h"
#include "servo.h"
#include "bsp.h"
#include "stm32f10x_tim.h"

volatile uint32_t time = 0;  // ��������
extern int servo_flag;
int	need_looplock_flag = 0;
extern int KZDec90[]; // Ѱ���ⲿ�˶���ʱ���� ʵ�ִ������ߵ��Թ���
extern int DDDec90[];
extern int NDDec90[];
extern int KZDec180[];
extern int DDDec180[];
extern int NDDec180[];
extern int KZAc90[];
extern int DDAc90[];
extern int NDAc90[];
extern int KZAc180[];
extern int DDAc180[];
extern int NDAc180[];
	
//#define NI 0	// ����
//#define shun 1	// ����

#define calibration 1000
#define calibration1 10000
/*---------------------------------------  D E F I N I T I O N  ---------------------------------------*/
Hand g_RobotHand; // ˫��״̬�ṹ��

char close_after_turn_flag=0;
char open_after_turn_flag=0;
char open_after_close_flag=0;
char key_flag=0; // ��������
char calibration_flag=0; // У׼��־λ

void OtherInitial(void) // ��ʼ��˫��״̬��־λ
{
	g_RobotHand.LeftHand=CLOSE;
	g_RobotHand.RightHand=CLOSE;
	g_RobotHand.LeftDir=0;
	g_RobotHand.RightDir=0;
}

s8 MechSteps[150];

void MechStepsInit(void)
{
	int i=0;
	for(i=0;i<150;i++)
	{
		MechSteps[i]=-1;
	}
}
void DMAdataInit(void)
{
	int i;
	for(i=0;i<1500;i++)
	{
		DMAbuffer[i]=0;
	}
}
//��һ�ν�ħ�������󣬸�λ��ץλ��
void ResetHandDir(void)
{
	//���Ƚ�����1����-1���ָ�λ��0 ��׼λ�����Ҿ�ʮ�Ȼص���׼λ��
	if(g_RobotHand.LeftDir==1||g_RobotHand.LeftDir==-3)
	{
		MotorMove(L_3);
	}
	if(g_RobotHand.LeftDir==-1||g_RobotHand.LeftDir==3)
	{
		MotorMove(L_1);
	}
	if(g_RobotHand.RightDir==1||g_RobotHand.RightDir==-3)
	{
		MotorMove(R_3);
	}
	if(g_RobotHand.RightDir==-1||g_RobotHand.RightDir==3)
	{
		MotorMove(R_1);
	}
	//Ȼ��2��λ��0λ�� ��׼λ������180�Ȼص���׼λ��
	if(g_RobotHand.LeftDir==2||g_RobotHand.LeftDir==-2)
	{
		MotorMove(L_2);
	}
	if(g_RobotHand.RightDir==2||g_RobotHand.RightDir==-2)
	{
		MotorMove(R_2);
	}
}
//================================================================DMA���ݰ�

#define NO_DATA      0 // ��ǽ������Ϊ��Ч����
#define SECOND_CAP   1 // ��ǽ������Ϊ��Ҫ�������㣨���ã�
#define MOVE         2 // ��ǽ������Ϊ��Ч���ݣ���Ҫִ��

u8 ReleaseDMApack(void)
{
//DMA���
/*
		��ͷ: 0XAA
		����ת����
		L_C: 0		R_C: 5
		L_O: 1		R_O: 6
		L_1: 2		R_1: 7
		L_2: 3		R_2: 8
		L_3: 4		R_3: 9
		��β: 0xBB
*/
	u16 i=0;
	u16 jj=0;
	
	for(i=0;i<=1500;i++)
	{
      delay_5us(3);
		if((DMAbuffer[i]==0xAA) && (DMAbuffer[i+149]==0xBB))   //ͨѶ��ͷ 
		{
			if(DMAbuffer[i+1]==0XFF)  //�ڶ���ɨ��
			{
				return SECOND_CAP;
			}
			else
			{
				for(jj=0;jj<148;jj++)
				{
					if(DMAbuffer[i+jj+1]==0XBB)
					{
						break;
					}
					else
					{
						MechSteps[jj]=DMAbuffer[i+jj+1];
						delay_5us(1);
					}
                    
				}
				return MOVE;
			}
        
		}
	}
	return NO_DATA;
}

//***************************************ʵ�ִ�DMA������������ʱ������˶�����********************************************
#define ERR0 0 // δ���ܵ�����
#define ERR1 1 // ������������Ч
#define ERR2 2 // ��ʱ���������־����ʧ��
#define ERR3 3 // �˶��ǶȽ�������
#define ERR4 4 // �˶����ʹ���
#define ERR5 5 // 
#define SUCCESS 6 // ִ�гɹ�

u8 ReleaseDMAcmd(void)
{
    u16 i, j, k;
    char typeStr[3] = {0};
    int delayValues[200] = {0}; 
    int valueCount = 0; 
    int angle = 0;	// �Ƕ�90/180
    int type = 0;  // 1=KZ, 2=DD, 3=ND
    int arraySize = 0; // ��ʱ�����С
    char sizeBuffer[10] = {0}; // 
    int sizeIndex = 0; // 
    int bracketFound = 0; // "["
    
    for(i=0; i<1500; i++)
    {
        if(DMAbuffer[i] == 'i' && DMAbuffer[i+1] == 'n' && DMAbuffer[i+2] == 't') // ��Ѱ�ҡ�int�����������ݿ�ʼλ��
			{
            // ͨ��i���4��5��ȷ���˶�����
            typeStr[0] = DMAbuffer[i+4];
            typeStr[1] = DMAbuffer[i+5];
            
            if(typeStr[0] == 'K' && typeStr[1] == 'Z') // �Ƿ�ΪKZ
            {
                type = 1;
            }
            else if(typeStr[0] == 'D' && typeStr[1] == 'D') // �Ƿ�ΪDD
            {
                type = 2;
            }
            else if(typeStr[0] == 'N' && typeStr[1] == 'D') // �Ƿ�ΪND
            {
                type = 3;
            }
            else
            {
                delay_5us(20);
                USARTSend(USART1, 'E');
                USARTSend(USART1, 'R');
                USARTSend(USART1, 'R');
                USARTSend(USART1, '4');
				
				delay_5us(1000);
				DMAdataInit(); // ��ʼ��������
                return ERR4; // �˶����ʹ���
            }
            
            // �ж�90����180
            for(j=i+6; j<i+12; j++)
            {
                if(DMAbuffer[j] >= '0' && DMAbuffer[j] <= '9')
                {
                    if(DMAbuffer[j] == '9' && DMAbuffer[j+1] == '0')
                    {
                        angle = 90;
                        break;
                    }
                    else if(DMAbuffer[j] == '1' && DMAbuffer[j+1] == '8' && DMAbuffer[j+2] == '0')
                    {
                        angle = 180;
                        break;
                    }
                }
            }
            
            if(angle != 90 && angle != 180)
            {
                delay_5us(20);
                USARTSend(USART1, 'E');
                USARTSend(USART1, 'R');
                USARTSend(USART1, 'R');
                USARTSend(USART1, '3');
				
				delay_5us(1000);
				//DMAdataInit(); // ��ʼ��������
                return ERR3; // �˶��ǶȽ�������
            }
            
            // ͨ��������[��������ʱ�����С
            for(j=i+6; j<i+30; j++)
            {
                if(DMAbuffer[j] == '[') // ������ʱ�����С�Ŀ�ʼλ��
                {
                    bracketFound = 1;
                    j++;
                    while(DMAbuffer[j] >= '0' && DMAbuffer[j] <= '9' && sizeIndex < 9)
                    {
                        sizeBuffer[sizeIndex++] = DMAbuffer[j];
                        j++;
                    }
                    if(sizeIndex > 0)
                    {
                        sizeBuffer[sizeIndex] = '\0';
                        arraySize = atoi(sizeBuffer);
                    }
                    break;
                }
            }
            
            // ��ʼ������ʱ�����Ա
            for(j=i; j<i+300; j++)
            {
                if(DMAbuffer[j] == '{') // ������ʱ���鿪ʼλ��
                {
                    char numBuffer[10] = {0};
                    int numIndex = 0;
                    j++;

                    // ������ʱ���������־��ȷ����������
                    while(j<i+1000 && DMAbuffer[j] != '}')
                    {
                        if(DMAbuffer[j] >= '0' && DMAbuffer[j] <= '9')
                        {
                            if(numIndex <= 9) // 1λ����ֱ�Ӵ���
                            {
                                numBuffer[numIndex++] = DMAbuffer[j];
                            }
                        }
                        else if((DMAbuffer[j] == ',' || DMAbuffer[j] == ' ' || DMAbuffer[j] == '\r' || DMAbuffer[j] == '\n') && numIndex > 0)
                        {
                            numBuffer[numIndex] = '\0';
                            if(valueCount < 200)
                            {
                                delayValues[valueCount++] = atoi(numBuffer);
                            }
                            numIndex = 0;
                        }
                        j++;
                    }
					
					// û�м�⵽��}����ʱ���������־ʱ
					if(DMAbuffer[j] != '}')
					{
						delay_5us(20);
                        USARTSend(USART1, 'E');
                        USARTSend(USART1, 'R');
                        USARTSend(USART1, 'R');
                        USARTSend(USART1, '2');
						
						delay_5us(1000);
						//DMAdataInit(); // ��ʼ��������
						return ERR2; // ��ʱ���������־����ʧ��
					}
						
                    // ��Ч����
                    if(numIndex > 0)
                    {
                        numBuffer[numIndex] = '\0';
                        if(valueCount < 200)
                        {
                            delayValues[valueCount++] = atoi(numBuffer);
                        }
                    }
                    
                    // ����Чֵ
                    if(valueCount < 1)
                    {
                        delay_5us(20);
                        USARTSend(USART1, 'N');
                        USARTSend(USART1, 'O');
                        USARTSend(USART1, 'V');
                        USARTSend(USART1, 'A');
                        USARTSend(USART1, 'L');
						
						delay_5us(1000);
						//DMAdataInit(); // ��ʼ��������
                        return ERR5;
                    }
                    
                    // ʵ�ʽ��յ���ʱ�����С�������е���ֵ����Ӧʱ
                    if(arraySize > 0 && valueCount != arraySize)
                    {
                        delay_5us(20);
                        USARTSend(USART1, 'S');
                        USARTSend(USART1, 'Z');
                        USARTSend(USART1, ':');
                        USART_SendNumber(USART1, valueCount);
                        USARTSend(USART1, '/');
                        USART_SendNumber(USART1, arraySize);
                        // ����ʵ�ʽ��յ�����ʱ�����С
                    }
                    
                    break;
                }
            }
            
            // ��0ʱ�䣬��ʼ��ʱ
            time = 0;
            
            // ���ս��������ͣ��Ƕȣ���ʱ�����С��ȷ��ִ�������˶�
            if(type > 0 && angle > 0 && valueCount > 0)
            {         
                if(type == 1)  // KZ
                {
                    if(angle == 90)
                    {
                        int copySize = (valueCount < 98) ? valueCount : 98;
                        for(k=0; k<copySize; k++)
                        {
                            KZDec90[k] = delayValues[k];
                        }
                        g_RobotHand.LeftHand=OPEN; // �����˶�״̬��ȷ��ΪKZ
                        MotorMove(L_1);
                    }
                    else if(angle == 180)
                    {
                        int copySize = (valueCount < 98) ? valueCount : 98;
                        for(k=0; k<copySize; k++)
                        {
                            KZDec180[k] = delayValues[k];
                        }
                        g_RobotHand.LeftHand=OPEN;
                        MotorMove(L_2);
                    }
                }
                else if(type == 2)  // DD
                {
                    if(angle == 90)
                    {
                        int copySize = (valueCount < 183) ? valueCount : 183;
                        for(k=0; k<copySize; k++)
                        {
                            DDDec90[k] = delayValues[k];
                        }
                        g_RobotHand.LeftHand=CLOSE; // �����˶�״̬��ȷ��ΪDD
                        MotorMove(L_1);
                    }
                    else if(angle == 180)
                    {
                        int copySize = (valueCount < 183) ? valueCount : 183;
                        for(k=0; k<copySize; k++)
                        {
                            DDDec180[k] = delayValues[k];
                        }
                        g_RobotHand.LeftHand=CLOSE;
                        MotorMove(L_2);
                    }
                }
                else if(type == 3)  // ND
                {
                    if(angle == 90)
                    {
                        int copySize = (valueCount < 108) ? valueCount : 108;
                        for(k=0; k<copySize; k++)
                        {
                            NDDec90[k] = delayValues[k];
                        }
                        g_RobotHand.LeftHand=CLOSE; // �����˶�״̬��ȷ��ΪND
                        g_RobotHand.RightHand=CLOSE;
                        MotorMove(L_1);
                    }
                    else if(angle == 180)
                    {
                        int copySize = (valueCount < 108) ? valueCount : 108;
                        for(k=0; k<copySize; k++)
                        {
                            NDDec180[k] = delayValues[k];
                        }
                        g_RobotHand.LeftHand=CLOSE;
                        g_RobotHand.RightHand=CLOSE;
                        MotorMove(L_2);
                    }
                }
                
                delay_5us(5);
                USARTSend(USART1, 'T');
                USARTSend(USART1, 'i');
                USARTSend(USART1, 'm');
                USARTSend(USART1, 'e');
                USARTSend(USART1, ':');
                USART_SendNumber(USART1, time); // �����˶�����ʱ��
                USARTSend(USART1, 'm');
                USARTSend(USART1, 's');
                USARTSend(USART1, '\r');
                USARTSend(USART1, '\n');
                
                delay_5us(1000);
                ResetHandDir(); // ˫�ָ�λ
                
                delay_5us(1000);
                DMAdataInit(); // ��ʼ��������
				
                return SUCCESS; // �����ɹ���������
            }

            delay_5us(20);
            USARTSend(USART1, 'E');
            USARTSend(USART1, 'R');
            USARTSend(USART1, 'R');
            USARTSend(USART1, '1');
			
			delay_5us(1000);
            //DMAdataInit(); // ��ʼ��������
            return ERR1; // ������������Ч
        }
    }
    
    delay_5us(20);
	
    //DMAdataInit(); // ��ʼ��������
    return ERR0; // δ���ܵ�����
}


/********************************************************************************************************
Description  : 
Inputs       : None   
Outputs      : None
********************************************************************************************************/
u8 moveflag=10;
u8 steps[]={
	M_L2,M_RO,M_L2,M_RC,M_R3,M_LO,M_R1,M_L1,M_LC,M_RO,
	M_L3,M_RC,M_R2,M_RO,M_L3,M_RC,M_L3,M_R1,M_LO,M_R1,
	M_LC,M_L2,M_LO,M_R3,M_LC,M_RO,M_R1,M_L2,M_RC,M_L2,
	M_R1,M_LO,M_R1,M_LC,M_L3,M_RO,M_L2,M_RC,M_LO,M_L1,
	M_R2,M_LC,M_R2,M_RO,M_L2,M_R1,M_RC,M_R1,M_L2,M_RO,
	M_L2,M_R1,M_RC,M_R3,M_L2,M_LO,M_R1,M_LC,M_RO,M_R1,
	M_L2,M_RC,M_L2,M_R2,M_L2,M_LO,M_L1,M_LC,M_RO,M_L3,
	M_RC,M_R1,M_RO,M_R1,M_L2,M_RC,M_R2
};  //77��

u8 steps2[]={
	M_L2,M_RO,M_L2,M_RC,M_R3,M_LO,M_R1,M_L1,M_LC,M_RO,
	M_L3,M_RC
};  //12��

u8 steps3[]={
    M_R2,M_RO,M_L2,M_R3,M_RC,M_R3,M_RO,M_L1,M_RC,M_LO,
	M_L3,M_LC,M_L2,M_R2,M_L2,M_RO,M_L2,M_R3,M_RC,M_LO,
	M_R3,M_LC,M_L2,M_R1,M_RO,M_R3,M_L2,M_RC,M_L2,M_R3,
	M_RO,M_R3,M_L2,M_RC,M_R2,M_LO,M_R2,M_L3,M_LC,M_RO,
	M_L2,M_RC,M_L1,M_LO,M_R3,M_LC,M_R3,M_L2,M_RO,M_L2,
	M_R3,M_RC,M_LO,M_R1,M_LC,M_L2,M_LO,M_R3,M_LC,M_R3,
	M_L1,M_RO,M_L1,M_RC,M_R2,M_RO,M_L1,M_RC,M_LO,M_L3,
	M_R3,M_LC,M_R1,M_RO,M_L2,M_RC,M_L2    
};	//77��


void pi()
{
    GPIO_InitTypeDef  GPIO_InitStructure;
 	
	RCC_APB2PeriphClockCmd(RCC_APB2Periph_GPIOC, ENABLE);	
    GPIO_InitStructure.GPIO_Pin = GPIO_Pin_13;	    		 // LED1-->PE.5 �˿�����, �������
	GPIO_InitStructure.GPIO_Mode = GPIO_Mode_Out_PP; 		 // �������
	GPIO_InitStructure.GPIO_Speed = GPIO_Speed_50MHz;		 // IO���ٶ�Ϊ50MHz				 // �����趨������ʼ��GPIOB.5
    GPIO_Init(GPIOC, &GPIO_InitStructure);	  				 // ������� ��IO���ٶ�Ϊ50MHz
	GPIO_SetBits(GPIOC,GPIO_Pin_13); 
}

void KEY_Init(void) // ����IO��ʼ��
{ 
 	GPIO_InitTypeDef GPIO_InitStructure;
 
 	RCC_APB2PeriphClockCmd(RCC_APB2Periph_GPIOB,ENABLE); // ʹ��PORTA,PORTEʱ��

	GPIO_InitStructure.GPIO_Pin  = GPIO_Pin_9|GPIO_Pin_3|GPIO_Pin_4; // KEY0-KEY2
	GPIO_InitStructure.GPIO_Mode = GPIO_Mode_IPD; // ���ó���������
 	GPIO_Init(GPIOB, &GPIO_InitStructure); // ��ʼ��GPIOE2,3,4

}

void calibrate(void){ // У׼���
	// ��������̽У׼
	int i;
	delay_5us(20000);
	L_DIR_P;
	delay_5us(30);	
	for(i=0;i<=66;i++)
	{
		L_PUL_UP;
		delay_5us(calibration);
		L_PUL_DOWN;
		delay_5us(calibration);
	}
		L_DIR_N;
		delay_5us(30);	
	while(GPIO_ReadInputDataBit(GPIOA,GPIO_Pin_0) == 0 )	
	{

		L_PUL_UP;
		delay_5us(calibration);
		L_PUL_DOWN;
		delay_5us(calibration);
	}
		L_DIR_P;
		delay_5us(30);	
	for(i=0;i<=66;i++)
	{
		L_PUL_UP;
		delay_5us(calibration);
		L_PUL_DOWN;
		delay_5us(calibration);
	}
		L_DIR_N;
		delay_5us(30);	
	while(GPIO_ReadInputDataBit(GPIOA,GPIO_Pin_0) == 0 )	
	{

		L_PUL_UP;
		delay_5us(calibration);
		L_PUL_DOWN;
		delay_5us(calibration);
	}

	L_DIR_N;
	delay_5us(30);	
	for(i=0;i<=32;i++)
	{
		L_PUL_UP;
		delay_5us(calibration);
		L_PUL_DOWN;
		delay_5us(calibration);
	}

	delay_5us(30000);

		R_DIR_N;
		delay_5us(30);	
	for(i=0;i<=66;i++)
	{
		R_PUL_UP;
		delay_5us(calibration);
		R_PUL_DOWN;
		delay_5us(calibration);
	}

	R_DIR_P;
	delay_5us(30);	
	while(GPIO_ReadInputDataBit(GPIOA,GPIO_Pin_1) == 0 )	
	{

		R_PUL_UP;
		delay_5us(calibration);
		R_PUL_DOWN;
		delay_5us(calibration);
	}
		R_DIR_N;
		delay_5us(30);	
	for(i=0;i<=66;i++)
	{
		R_PUL_UP;
		delay_5us(calibration);
		R_PUL_DOWN;
		delay_5us(calibration);
	}
	R_DIR_P;
	delay_5us(30);	
	while(GPIO_ReadInputDataBit(GPIOA,GPIO_Pin_1) == 0 )	
	{
		R_PUL_UP;
		delay_5us(calibration);
		R_PUL_DOWN;
		delay_5us(calibration);
	}

		R_DIR_P;
		delay_5us(30);	
	for(i=0;i<21;i++)
	{
		R_PUL_UP;
		delay_5us(calibration);
		R_PUL_DOWN;
		delay_5us(calibration);
	}
}

void TIM4_Configuration(void) // ����ʱ���ʱ���õļ�ʱ��
{
    TIM_TimeBaseInitTypeDef TIM_TimeBaseStructure;
    NVIC_InitTypeDef NVIC_InitStructure;
    
    // 1. ����TIM4ʱ��
    RCC_APB1PeriphClockCmd(RCC_APB1Periph_TIM4, ENABLE);
    
    // 2. ����TIM4��������
    // ����ϵͳʱ��Ϊ72MHz��APB1��Ƶϵ��Ϊ2������TIM4ʱ��Ϊ72MHz
    // Ԥ��Ƶֵ7200-1��������ʱ��Ϊ10kHz (72MHz/7200=10kHz)
    // �Զ���װ��ֵ10-1����ʱ��ÿ1ms���һ�� (10kHz/10=1ms)
    TIM_TimeBaseStructure.TIM_Period = 10 - 1;
    TIM_TimeBaseStructure.TIM_Prescaler = 7200 - 1;
    TIM_TimeBaseStructure.TIM_ClockDivision = 0;
    TIM_TimeBaseStructure.TIM_CounterMode = TIM_CounterMode_Up;
    TIM_TimeBaseInit(TIM4, &TIM_TimeBaseStructure);
    
    // 3. ʹ��TIM4�����ж�
    TIM_ITConfig(TIM4, TIM_IT_Update, ENABLE);
    
    // 4. ����NVIC
    NVIC_InitStructure.NVIC_IRQChannel = TIM4_IRQn;
    NVIC_InitStructure.NVIC_IRQChannelPreemptionPriority = 0;
    NVIC_InitStructure.NVIC_IRQChannelSubPriority = 1;
    NVIC_InitStructure.NVIC_IRQChannelCmd = ENABLE;
    NVIC_Init(&NVIC_InitStructure);
    
    // 5. ����TIM4
    TIM_Cmd(TIM4, ENABLE);
}

void TIM4_IRQHandler(void)
{
    if (TIM_GetITStatus(TIM4, TIM_IT_Update) != RESET)
    {
        TIM_ClearITPendingBit(TIM4, TIM_IT_Update);
        time++; // ÿ1ms����һ��
    }
}

void tdelay_ms(uint32_t ms) {
    volatile uint32_t i, j;
    for (i = 0; i < ms; i++) {
        for (j = 0; j < 7200; j++); // ����1ms��ʱ
    }
}

void check_loop_lock(void){ // ����Ƿ���Ҫ��·����
	if(need_looplock_flag == 1){
		delay_5us(2000);
		LOOP_EXHAUST;
		need_looplock_flag = 0;
		delay_5us(2000);
	}
}

int main(void)
{
	int i=0;
	int test_key2=0;
	int next_flag=0;
	u8 ReleaseDMApackReturn; // ��λ������Ƿ�Ϊ��Ч���ݵ��жϱ�־��
    Bsp_Init();
    pi();
	OtherInitial();
	DMAdataInit();
	USART2_Init();
	TIM4_Configuration();
	LOOP_EXHAUST;           // ����ʱ��̨й�����������ɵ���״̬
	need_looplock_flag = 1; // �Ƿ���Ҫ��·�����������ɵ���״̬�ı�־λ����Ҫ���ɵ���Ϊ1
	
	USARTSend(USART1,' ');
	USARTSend(USART1,'o');
	USARTSend(USART1,'k');

	GPIO_SetBits(GPIOC, GPIO_Pin_13);
	GPIO_SetBits(GPIOC, GPIO_Pin_14);
	GPIO_SetBits(GPIOB, GPIO_Pin_3);
	GPIO_SetBits(GPIOB, GPIO_Pin_4);		
	GPIO_SetBits(GPIOB, GPIO_Pin_9);	
	
	calibrate(); // У׼����
	
	while(1) // ����ѭ��
	{
		check_loop_lock();
		
		ReleaseDMAcmd();
		
		if(GPIO_ReadInputDataBit(GPIOB,GPIO_Pin_9) == 0 )
		{
			test_key2++;
			key_flag=1;
			next_flag=0;
			delay_5us(1000);
		}

		if(GPIO_ReadInputDataBit(GPIOB,GPIO_Pin_4) == 0 )
		{
			key_flag=2;
			next_flag++;
//			ReleaseDMApackReturn = ReleaseDMApack();
//			
//			for(i=0;i<150;i++){
//				USART_SendNumber(USART1,MechSteps[i]);
//			}
			if(next_flag>3)goto running; // ��������ѭ��
			delay_5us(1000);
		}
		switch(key_flag){
			case 1:
				MotorMove(R_C);
				MotorMove(L_C);
				key_flag=0;
				break;
			case 2:
				test_key2 = 0;
				MotorMove(R_O);
				MotorMove(L_O);
				need_looplock_flag = 1; // ��·��������̨�������ɵ���״̬
				key_flag = 0;
				delay_5us2(10000);
				ResetHandDir();
				break;
		}
		if(test_key2==3)
		{
			
			delay_5us(1000);
			time=0;
			test_key2=0;
			for(i=0;i<77;i++)MotorMove(steps3[i]);
			delay_5us(1000);
			//MotorMove(L_O);
			key_flag=0;
			GPIO_SetBits(GPIOB, GPIO_Pin_9);
			MotorMove(L_O);
			MotorMove(R_O);
			
			USARTSend(USART1, 'T');
			USARTSend(USART1, 'i');
			USARTSend(USART1, 'm');
			USARTSend(USART1, 'e');
			USARTSend(USART1, ':');
			USART_SendNumber(USART1,time); // ���Ͳ����ʱ��ms��
			USARTSend(USART1, 'm');
			USARTSend(USART1, 's');
			USARTSend(USART1, '\r');
			USARTSend(USART1, '\n');
	   }
	}
	
	running:
	
	need_looplock_flag=1; // ȷ����·й�����������ɵ���״̬
	
	while(1) // ��ʽѭ�� �ȴ���λ������
	{	
		check_loop_lock();
		
		if(GPIO_ReadInputDataBit(GPIOB,GPIO_Pin_9) == 0 ){		
			while(GPIO_ReadInputDataBit(GPIOB,GPIO_Pin_9) == 0);
			delay_5us(200);
		  //MotorMove(L_C);
		  //MotorMove(R_C);
			L_CAP_OUT_CLOSE;
			delay_5us2(10000);
			R_CAP_OUT_CLOSE;
			delay_5us2(10000);
			GPIO_SetBits(GPIOB, GPIO_Pin_9);	
		}
		if(GPIO_ReadInputDataBit(GPIOB,GPIO_Pin_4) == 0 ){		
			while(GPIO_ReadInputDataBit(GPIOB,GPIO_Pin_4) == 0);
			delay_5us(200);
			need_looplock_flag = 1; // ��·��������̨�������ɵ���״̬
			MotorMove(L_O);
			MotorMove(R_O);
			delay_5us2(10000);
			ResetHandDir();
			GPIO_SetBits(GPIOB, GPIO_Pin_4);
		}
		
		//��ʼ����е�����б�
		MechStepsInit();
		delay_5us(8000);
		
		//DMA���
		ReleaseDMApackReturn = ReleaseDMApack();
		if(ReleaseDMApackReturn==NO_DATA)
		{
			//do nothing
		}
		else if(ReleaseDMApackReturn==SECOND_CAP)   // �����ڶ������գ����嶯����������ɺ���0xCC
		{											// �˴����� ������ͷ1��������ɽ��������������
			MotorMove(L_O);
			MotorMove(R_2);
			MotorMove(L_C);
			MotorMove(R_O);
			MotorMove(L_2);
			MotorMove(R_C);
			delay_5us(8000);
			USARTSend(USART1,'1');
			USARTSend(USART1,'1');
			USARTSend(USART1,'1');//
			DMAdataInit();     //�����һ�η�������
		}
		else if(ReleaseDMApackReturn==MOVE)
		{
			need_looplock_flag = 1;

			MotorMove(R_C);
			MotorMove(L_C);
			//��ħ��
			for(i=0;i<150;i++)
			{
				if(MechSteps[i]==-1)
				{
					break;
				}
				else
				{
					MotorMove(MechSteps[i]);
					//delay_5us(400);
				}					
			}
			//��������ץλ��
			USARTSend(USART1,'E');              //���ж����������END��ʾ���
			USARTSend(USART1,'N'); 
			USARTSend(USART1,'D'); 
			USARTSend(USART1,'\r');  
			USARTSend(USART1,'\n');
			
			MotorMove(L_O);   // �����������ץ��
			MotorMove(R_O);
            
			DMAdataInit(); // ��ս��ջ�����
		}  
	}
}
		
	

//===========================================  End Of File  ===========================================//
