#include "stm32f10x_tim.h" 
#include "servo.h"
#include "delay.h"
#include <string.h>
/*
 串口：
 USART2_RX  ->  PA3
 USART2_TX  ->  PA2
 
 舵机总线：
 VCC -> 5v
 GND -> GND
 RX  -> PA3
*/
// USART2 初始化函数 用于调试状态语句输出 与解魔方流程无关
void USART2_Init(void) {
    GPIO_InitTypeDef GPIO_InitStructure;
    USART_InitTypeDef USART_InitStructure;
    NVIC_InitTypeDef NVIC_InitStructure;

    // 使能GPIOA和USART2时钟
    RCC_APB2PeriphClockCmd(RCC_APB2Periph_GPIOA, ENABLE);
    RCC_APB1PeriphClockCmd(RCC_APB1Periph_USART2, ENABLE);

    // 配置USART2 Tx (PA.2)为复用推挽输出
    GPIO_InitStructure.GPIO_Pin = GPIO_Pin_2;
    GPIO_InitStructure.GPIO_Mode = GPIO_Mode_AF_PP;
    GPIO_InitStructure.GPIO_Speed = GPIO_Speed_50MHz;
    GPIO_Init(GPIOA, &GPIO_InitStructure);

    // 配置USART2 Rx (PA.3)为浮空输入
    GPIO_InitStructure.GPIO_Pin = GPIO_Pin_3;
    GPIO_InitStructure.GPIO_Mode = GPIO_Mode_IN_FLOATING;
    GPIO_Init(GPIOA, &GPIO_InitStructure);

    // USART2 配置
    USART_InitStructure.USART_BaudRate = 115200;
    USART_InitStructure.USART_WordLength = USART_WordLength_8b;
    USART_InitStructure.USART_StopBits = USART_StopBits_1;
    USART_InitStructure.USART_Parity = USART_Parity_No;
    USART_InitStructure.USART_HardwareFlowControl = USART_HardwareFlowControl_None;
    USART_InitStructure.USART_Mode = USART_Mode_Rx | USART_Mode_Tx;
    USART_Init(USART2, &USART_InitStructure);

    // 使能接收中断
    USART_ITConfig(USART1, USART_IT_RXNE, ENABLE);

    // 使能USART2
    USART_Cmd(USART2, ENABLE);

    // 配置NVIC
    NVIC_InitStructure.NVIC_IRQChannel = USART2_IRQn;
    NVIC_InitStructure.NVIC_IRQChannelPreemptionPriority = 0;
    NVIC_InitStructure.NVIC_IRQChannelSubPriority = 0;
    NVIC_InitStructure.NVIC_IRQChannelCmd = ENABLE;
    NVIC_Init(&NVIC_InitStructure);
}

// 发送一个字符
void USART2_SendChar(char ch) {
    while (USART_GetFlagStatus(USART2, USART_FLAG_TXE) == RESET);
    USART_SendData(USART2, (uint8_t)ch);
}

// 发送字符串
void USART2_SendString(const unsigned char* str) {
    while (*str) {
        USART2_SendChar(*str++);
    }
}

//-------------------------------------------------------------------------------------------//
//------------------------------------------总线舵机-----------------------------------------//
//-------------------------------------------------------------------------------------------//

volatile int servo_flag=0; // 舵机当前状态标志位 伸出：0  收回 ：1

// 单舵机角度控制
void servo_send(int angle){
	char cmd_buf[32];
	angle=angle/270*2000+500;
	snprintf(cmd_buf, sizeof(cmd_buf), "#%03dP%04dT1000!", 0, angle);
	USART2_SendString((unsigned char *)cmd_buf);
}

void servo_init_shun(void){
	char cmd_buf[32]="#000PMOD7!";//设置为模式7，马达模式，角度 360 度，定时旋转，方向顺时针
	USART2_SendString((unsigned char *)cmd_buf);
}

void servo_init_ni(void){
	char cmd_buf[32]="#000PMOD8!";//设置为模式8，马达模式，角度 360 度，定时旋转，方向逆时针
	USART2_SendString((unsigned char *)cmd_buf);
}

void servo_shun(int speed,int time_s){
	char cmd_buf[32];
	servo_flag=0;
	snprintf(cmd_buf, sizeof(cmd_buf), "#%03dP%04dT%04d!", 0, speed+1500, time_s);//表示以 speed 的速度，运行 time_s 秒
	USART2_SendString((unsigned char *)cmd_buf);
}

void servo_ni(int speed,int time_s){
	char cmd_buf[32];
	//servo_flag=1;
	snprintf(cmd_buf, sizeof(cmd_buf), "#%03dP%04dT%04d!", 0, speed+1500, time_s);//表示以 speed 的速度，运行 time_s 秒
	USART2_SendString((unsigned char *)cmd_buf);
}

// 实物链接 https://item.taobao.com/item.htm?detail_redpacket_pop=true&id=549063298947&ltk2=1748425160463xsfknhsx6ia05agp4a66z2k&ns=1&priceTId=213e04cf17484251560406139e1cb2&query=%E6%80%BB%E7%BA%BF%E8%88%B5%E6%9C%BA&skuId=5864385883768&spm=a21n57.1.hoverItem.1&utparam=%7B%22aplus_abtest%22%3A%22ce0427b078a6df74aa08d031912a2a66%22%7D&xxc=ad_ztc

//-------------------------------------------------------------------------------------------//
//------------------------------------------AB相TT减速电机-----------------------------------//
//-------------------------------------------------------------------------------------------//

extern int time;			// 执行时间计时变量	
int time_out=2000;			// 执行超时时间
#define RX_BUFFER_SIZE 64   // 发送命令数组大小
#define CMD_OK "Extending"  // 执行成功回应
static uint8_t RxBuffer[RX_BUFFER_SIZE]; // 命令数组
static uint16_t RxIndex = 0;

//#chu!表示支撑成功伸出
//#shou！表示支撑成功收入

// USART1中断服务函数
void USART1_IRQHandler(void) {
	int i = 0;
    if (USART_GetITStatus(USART1, USART_IT_RXNE) != RESET) {
		u8 data = USART_ReceiveData(USART1);	
        // 存入缓冲区
        if(RxIndex < RX_BUFFER_SIZE - 1)
        {
            RxBuffer[RxIndex++] = data;
            
            // 检查是否接收到命令结束符'!'
            if(data == '!')
            {
                RxBuffer[RxIndex] = '\0';  // 字符串结束符
                //RxFlag = 1;  // 设置接收完成标志
            }
		}
		else
        {
			for(i=0; i < RX_BUFFER_SIZE; i++)
			{
				RxBuffer[i] = 0;
			}
			RxIndex = 0;
        }
    }
}

int motor_chu(void){
	char cmd_buf[32]="#chu!";
	USART2_SendString((unsigned char *)cmd_buf);
//	time=0;
//	while(time>time_out){
//		if (strncmp((char *)RxBuffer, CMD_OK, strlen(CMD_OK)) == 0)
//		{
//			return 1;
//		}
//	}
	return 0;
};

int motor_shou(void){
	char cmd_buf[32]="#shou!";
	USART2_SendString((unsigned char *)cmd_buf);
//	time=0;
//	while(time>time_out){
//		if (strncmp((char *)RxBuffer, CMD_OK, strlen(CMD_OK)) == 0)
//		{
//			return 1;
//		}
//	}
	return 0;
};

// 实物链接 https://item.taobao.com/item.htm?spm=a1z09.2.0.0.62d42e8dSM9sG8&id=593413957261&_u=s206qavsr49173&pisk=gV_uO5_7Rg-5tzyvDZT7bhFyAZrvNUTC893ppepU0KJbO0n8NHRFt9bd26Jp-9XhKLKUFUQhn11LNwFWzyAFBtYdwLpdnWXRO2e7AU3EKT1aF_pLNwvenTWHA79pLpXdTgFYWPC5NeTeKRUTWzBK9NWoU0-zTKRW1SdzGgzFbeTUBJmx83aMRToEGODr0IJXtBke898VgQde8QoyLSv2tCiEU9WUinRM12Rr845q3CJq4bJyLEJ2wCDyTB-zisJX_08e8984iBMXYbJN8M74HE6M6zI0pZRkqd5yU_CRuA-xV_9cSwb2ZsxVakgE8ZAkqwQgq9Xw2M56faQz-qTRsi826_aooF5ysTpAQPuDusCVphsbdVKfnwxFz3lUgK7lnaTPekuGIUXHba-mT2CGgL8DgNPKfKQDe97k053MvE79bUSYc8KprCXFPTcn8O5A6atf-RD2dgd6uCC7rqYNaBSrymoZam3BgWQqADtyGIvTfrOYGIXUtgN0icsB4IOMBSVmADtyGIvTiSm_d3RXsdC..

//AB相TT减速电机END--------------------------------------------------------------------------------
