/*******************************************************************************************************
*一些建议：
*1：下位机调速建议着重优化 带动 曲线。
*2：关于防缠绕的优化，通过测试发现现有的上位机解魔方算法大概每扩展出40步，就会触发一次防缠绕机制，
*   所以我们可以通过优化气管布局来增加触发防缠绕的阈值从而减少整体步骤。提升幅度在2%左右
*3：关于未来的发展方向，因为魔方本身质量较轻，所以对电机的扭矩要求不高，而我们现在的结构，沉重的金属滑台就意味着电机的扭矩不能太低，
*   也就是说电机的转速不能太快，不然丢步造成的累计误差会使长步骤的还原魔方变得不太可能。故我们应该做小型话处理
*******************************************************************************************************/
#include "main.h"
#include "oled.h"
#include "servo.h"
#include "bsp.h"
#include "stm32f10x_tim.h"

volatile uint32_t time = 0;  // 自增变量
extern int servo_flag;
int	need_looplock_flag = 0;
extern int KZDec90[]; // 寻找外部运动延时数组 实现串口在线调试功能
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
	
//#define NI 0	// 弃用
//#define shun 1	// 弃用

#define calibration 1000
#define calibration1 10000
/*---------------------------------------  D E F I N I T I O N  ---------------------------------------*/
Hand g_RobotHand; // 双手状态结构体

char close_after_turn_flag=0;
char open_after_turn_flag=0;
char open_after_close_flag=0;
char key_flag=0; // 按键功能
char calibration_flag=0; // 校准标志位

void OtherInitial(void) // 初始化双手状态标志位
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
//在一次解魔方结束后，复位手抓位置
void ResetHandDir(void)
{
	//首先将所有1或者-1的手复位到0 标准位置左右九十度回到标准位置
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
	//然后将2复位到0位置 标准位置左右180度回到标准位置
	if(g_RobotHand.LeftDir==2||g_RobotHand.LeftDir==-2)
	{
		MotorMove(L_2);
	}
	if(g_RobotHand.RightDir==2||g_RobotHand.RightDir==-2)
	{
		MotorMove(R_2);
	}
}
//================================================================DMA数据包

#define NO_DATA      0 // 标记解包所得为无效数据
#define SECOND_CAP   1 // 标记解包所得为需要二次拍摄（弃用）
#define MOVE         2 // 标记解包所得为有效数据，需要执行

u8 ReleaseDMApack(void)
{
//DMA解包
/*
		包头: 0XAA
		步骤转换表：
		L_C: 0		R_C: 5
		L_O: 1		R_O: 6
		L_1: 2		R_1: 7
		L_2: 3		R_2: 8
		L_3: 4		R_3: 9
		包尾: 0xBB
*/
	u16 i=0;
	u16 jj=0;
	
	for(i=0;i<=1500;i++)
	{
      delay_5us(3);
		if((DMAbuffer[i]==0xAA) && (DMAbuffer[i+149]==0xBB))   //通讯桢头 
		{
			if(DMAbuffer[i+1]==0XFF)  //第二次扫描
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

//***************************************实现从DMA缓冲区解析延时数组和运动命令********************************************
#define ERR0 0 // 未接受到命令
#define ERR1 1 // 解析的数据无效
#define ERR2 2 // 延时数组结束标志解析失败
#define ERR3 3 // 运动角度解析错误
#define ERR4 4 // 运动类型错误
#define ERR5 5 // 
#define SUCCESS 6 // 执行成功

u8 ReleaseDMAcmd(void)
{
    u16 i, j, k;
    char typeStr[3] = {0};
    int delayValues[200] = {0}; 
    int valueCount = 0; 
    int angle = 0;	// 角度90/180
    int type = 0;  // 1=KZ, 2=DD, 3=ND
    int arraySize = 0; // 延时数组大小
    char sizeBuffer[10] = {0}; // 
    int sizeIndex = 0; // 
    int bracketFound = 0; // "["
    
    for(i=0; i<1500; i++)
    {
        if(DMAbuffer[i] == 'i' && DMAbuffer[i+1] == 'n' && DMAbuffer[i+2] == 't') // 先寻找‘int’，锁定数据开始位置
			{
            // 通过i后第4，5来确定运动类型
            typeStr[0] = DMAbuffer[i+4];
            typeStr[1] = DMAbuffer[i+5];
            
            if(typeStr[0] == 'K' && typeStr[1] == 'Z') // 是否为KZ
            {
                type = 1;
            }
            else if(typeStr[0] == 'D' && typeStr[1] == 'D') // 是否为DD
            {
                type = 2;
            }
            else if(typeStr[0] == 'N' && typeStr[1] == 'D') // 是否为ND
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
				DMAdataInit(); // 初始化缓冲区
                return ERR4; // 运动类型错误
            }
            
            // 判断90还是180
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
				//DMAdataInit(); // 初始化缓冲区
                return ERR3; // 运动角度解析错误
            }
            
            // 通过锁定‘[’解析延时数组大小
            for(j=i+6; j<i+30; j++)
            {
                if(DMAbuffer[j] == '[') // 锁定延时数组大小的开始位置
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
            
            // 开始解析延时数组成员
            for(j=i; j<i+300; j++)
            {
                if(DMAbuffer[j] == '{') // 锁定延时数组开始位置
                {
                    char numBuffer[10] = {0};
                    int numIndex = 0;
                    j++;

                    // 锁定延时数组结束标志来确保解析完整
                    while(j<i+1000 && DMAbuffer[j] != '}')
                    {
                        if(DMAbuffer[j] >= '0' && DMAbuffer[j] <= '9')
                        {
                            if(numIndex <= 9) // 1位数字直接存入
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
					
					// 没有检测到‘}’延时数组结束标志时
					if(DMAbuffer[j] != '}')
					{
						delay_5us(20);
                        USARTSend(USART1, 'E');
                        USARTSend(USART1, 'R');
                        USARTSend(USART1, 'R');
                        USARTSend(USART1, '2');
						
						delay_5us(1000);
						//DMAdataInit(); // 初始化缓冲区
						return ERR2; // 延时数组结束标志解析失败
					}
						
                    // 有效数据
                    if(numIndex > 0)
                    {
                        numBuffer[numIndex] = '\0';
                        if(valueCount < 200)
                        {
                            delayValues[valueCount++] = atoi(numBuffer);
                        }
                    }
                    
                    // 非有效值
                    if(valueCount < 1)
                    {
                        delay_5us(20);
                        USARTSend(USART1, 'N');
                        USARTSend(USART1, 'O');
                        USARTSend(USART1, 'V');
                        USARTSend(USART1, 'A');
                        USARTSend(USART1, 'L');
						
						delay_5us(1000);
						//DMAdataInit(); // 初始化缓冲区
                        return ERR5;
                    }
                    
                    // 实际接收的延时数组大小与命令中的数值不对应时
                    if(arraySize > 0 && valueCount != arraySize)
                    {
                        delay_5us(20);
                        USARTSend(USART1, 'S');
                        USARTSend(USART1, 'Z');
                        USARTSend(USART1, ':');
                        USART_SendNumber(USART1, valueCount);
                        USARTSend(USART1, '/');
                        USART_SendNumber(USART1, arraySize);
                        // 发送实际接收到的延时数组大小
                    }
                    
                    break;
                }
            }
            
            // 清0时间，开始计时
            time = 0;
            
            // 按照解析的类型，角度，延时数组大小来确定执行那种运动
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
                        g_RobotHand.LeftHand=OPEN; // 设置运动状态，确保为KZ
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
                        g_RobotHand.LeftHand=CLOSE; // 设置运动状态，确保为DD
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
                        g_RobotHand.LeftHand=CLOSE; // 设置运动状态，确保为ND
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
                USART_SendNumber(USART1, time); // 发送运动消耗时间
                USARTSend(USART1, 'm');
                USARTSend(USART1, 's');
                USARTSend(USART1, '\r');
                USARTSend(USART1, '\n');
                
                delay_5us(1000);
                ResetHandDir(); // 双手复位
                
                delay_5us(1000);
                DMAdataInit(); // 初始化缓冲区
				
                return SUCCESS; // 解析成功并且运行
            }

            delay_5us(20);
            USARTSend(USART1, 'E');
            USARTSend(USART1, 'R');
            USARTSend(USART1, 'R');
            USARTSend(USART1, '1');
			
			delay_5us(1000);
            //DMAdataInit(); // 初始化缓冲区
            return ERR1; // 解析的数据无效
        }
    }
    
    delay_5us(20);
	
    //DMAdataInit(); // 初始化缓冲区
    return ERR0; // 未接受到命令
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
};  //77步

u8 steps2[]={
	M_L2,M_RO,M_L2,M_RC,M_R3,M_LO,M_R1,M_L1,M_LC,M_RO,
	M_L3,M_RC
};  //12步

u8 steps3[]={
    M_R2,M_RO,M_L2,M_R3,M_RC,M_R3,M_RO,M_L1,M_RC,M_LO,
	M_L3,M_LC,M_L2,M_R2,M_L2,M_RO,M_L2,M_R3,M_RC,M_LO,
	M_R3,M_LC,M_L2,M_R1,M_RO,M_R3,M_L2,M_RC,M_L2,M_R3,
	M_RO,M_R3,M_L2,M_RC,M_R2,M_LO,M_R2,M_L3,M_LC,M_RO,
	M_L2,M_RC,M_L1,M_LO,M_R3,M_LC,M_R3,M_L2,M_RO,M_L2,
	M_R3,M_RC,M_LO,M_R1,M_LC,M_L2,M_LO,M_R3,M_LC,M_R3,
	M_L1,M_RO,M_L1,M_RC,M_R2,M_RO,M_L1,M_RC,M_LO,M_L3,
	M_R3,M_LC,M_R1,M_RO,M_L2,M_RC,M_L2    
};	//77步


void pi()
{
    GPIO_InitTypeDef  GPIO_InitStructure;
 	
	RCC_APB2PeriphClockCmd(RCC_APB2Periph_GPIOC, ENABLE);	
    GPIO_InitStructure.GPIO_Pin = GPIO_Pin_13;	    		 // LED1-->PE.5 端口配置, 推挽输出
	GPIO_InitStructure.GPIO_Mode = GPIO_Mode_Out_PP; 		 // 推挽输出
	GPIO_InitStructure.GPIO_Speed = GPIO_Speed_50MHz;		 // IO口速度为50MHz				 // 根据设定参数初始化GPIOB.5
    GPIO_Init(GPIOC, &GPIO_InitStructure);	  				 // 推挽输出 ，IO口速度为50MHz
	GPIO_SetBits(GPIOC,GPIO_Pin_13); 
}

void KEY_Init(void) // 按键IO初始化
{ 
 	GPIO_InitTypeDef GPIO_InitStructure;
 
 	RCC_APB2PeriphClockCmd(RCC_APB2Periph_GPIOB,ENABLE); // 使能PORTA,PORTE时钟

	GPIO_InitStructure.GPIO_Pin  = GPIO_Pin_9|GPIO_Pin_3|GPIO_Pin_4; // KEY0-KEY2
	GPIO_InitStructure.GPIO_Mode = GPIO_Mode_IPD; // 设置成上拉输入
 	GPIO_Init(GPIOB, &GPIO_InitStructure); // 初始化GPIOE2,3,4

}

void calibrate(void){ // 校准电机
	// 先左电机上探校准
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

void TIM4_Configuration(void) // 运行时间计时所用的计时器
{
    TIM_TimeBaseInitTypeDef TIM_TimeBaseStructure;
    NVIC_InitTypeDef NVIC_InitStructure;
    
    // 1. 开启TIM4时钟
    RCC_APB1PeriphClockCmd(RCC_APB1Periph_TIM4, ENABLE);
    
    // 2. 配置TIM4基础设置
    // 假设系统时钟为72MHz，APB1分频系数为2，所以TIM4时钟为72MHz
    // 预分频值7200-1，计数器时钟为10kHz (72MHz/7200=10kHz)
    // 自动重装载值10-1，定时器每1ms溢出一次 (10kHz/10=1ms)
    TIM_TimeBaseStructure.TIM_Period = 10 - 1;
    TIM_TimeBaseStructure.TIM_Prescaler = 7200 - 1;
    TIM_TimeBaseStructure.TIM_ClockDivision = 0;
    TIM_TimeBaseStructure.TIM_CounterMode = TIM_CounterMode_Up;
    TIM_TimeBaseInit(TIM4, &TIM_TimeBaseStructure);
    
    // 3. 使能TIM4更新中断
    TIM_ITConfig(TIM4, TIM_IT_Update, ENABLE);
    
    // 4. 配置NVIC
    NVIC_InitStructure.NVIC_IRQChannel = TIM4_IRQn;
    NVIC_InitStructure.NVIC_IRQChannelPreemptionPriority = 0;
    NVIC_InitStructure.NVIC_IRQChannelSubPriority = 1;
    NVIC_InitStructure.NVIC_IRQChannelCmd = ENABLE;
    NVIC_Init(&NVIC_InitStructure);
    
    // 5. 启动TIM4
    TIM_Cmd(TIM4, ENABLE);
}

void TIM4_IRQHandler(void)
{
    if (TIM_GetITStatus(TIM4, TIM_IT_Update) != RESET)
    {
        TIM_ClearITPendingBit(TIM4, TIM_IT_Update);
        time++; // 每1ms自增一次
    }
}

void tdelay_ms(uint32_t ms) {
    volatile uint32_t i, j;
    for (i = 0; i < ms; i++) {
        for (j = 0; j < 7200; j++); // 粗略1ms延时
    }
}

void check_loop_lock(void){ // 检查是否需要环路排气
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
	u8 ReleaseDMApackReturn; // 上位机命令（是否为有效数据的判断标志）
    Bsp_Init();
    pi();
	OtherInitial();
	DMAdataInit();
	USART2_Init();
	TIM4_Configuration();
	LOOP_EXHAUST;           // 开机时滑台泄气，进入自由调节状态
	need_looplock_flag = 1; // 是否需要环路排气进入自由调节状态的标志位，需要自由调节为1
	
	USARTSend(USART1,' ');
	USARTSend(USART1,'o');
	USARTSend(USART1,'k');

	GPIO_SetBits(GPIOC, GPIO_Pin_13);
	GPIO_SetBits(GPIOC, GPIO_Pin_14);
	GPIO_SetBits(GPIOB, GPIO_Pin_3);
	GPIO_SetBits(GPIOB, GPIO_Pin_4);		
	GPIO_SetBits(GPIOB, GPIO_Pin_9);	
	
	calibrate(); // 校准函数
	
	while(1) // 测试循环
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
			if(next_flag>3)goto running; // 结束测试循环
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
				need_looplock_flag = 1; // 环路排气，滑台进入自由调整状态
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
			USART_SendNumber(USART1,time); // 发送步骤耗时（ms）
			USARTSend(USART1, 'm');
			USARTSend(USART1, 's');
			USARTSend(USART1, '\r');
			USARTSend(USART1, '\n');
	   }
	}
	
	running:
	
	need_looplock_flag=1; // 确保环路泄气，进入自由调节状态
	
	while(1) // 正式循环 等待上位机数据
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
			need_looplock_flag = 1; // 环路排气，滑台进入自由调整状态
			MotorMove(L_O);
			MotorMove(R_O);
			delay_5us2(10000);
			ResetHandDir();
			GPIO_SetBits(GPIOB, GPIO_Pin_4);
		}
		
		//初始化机械步骤列表
		MechStepsInit();
		delay_5us(8000);
		
		//DMA解包
		ReleaseDMApackReturn = ReleaseDMApack();
		if(ReleaseDMApackReturn==NO_DATA)
		{
			//do nothing
		}
		else if(ReleaseDMApackReturn==SECOND_CAP)   // 触发第二次拍照，具体动作看需求，完成后发送0xCC
		{											// 此处无用 四摄像头1次拍照完成解算无需二次拍照
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
			DMAdataInit();     //清除第一次发送命令
		}
		else if(ReleaseDMApackReturn==MOVE)
		{
			need_looplock_flag = 1;

			MotorMove(R_C);
			MotorMove(L_C);
			//解魔方
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
			//调整好手抓位置
			USARTSend(USART1,'E');              //所有动作做完后发送END表示完成
			USARTSend(USART1,'N'); 
			USARTSend(USART1,'D'); 
			USARTSend(USART1,'\r');  
			USARTSend(USART1,'\n');
			
			MotorMove(L_O);   // 动作结束后打开抓手
			MotorMove(R_O);
            
			DMAdataInit(); // 清空接收缓冲区
		}  
	}
}
		
	

//===========================================  End Of File  ===========================================//
