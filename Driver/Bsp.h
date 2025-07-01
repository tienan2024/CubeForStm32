#ifndef __BSP_H
#define __BSP_H

/*-----------------------------------------  I N C L U D E S  -----------------------------------------*/
#include "Global.h"
/*---------------------------------------  D E F I N I T I O N  ---------------------------------------*/
#define RX_BUF_SIZE 1500 // ԭΪ300         //USART1_DMA���������С
/*-------------------------------------------  M A C R O S  -------------------------------------------*/


/*--------------------------------------  D E C L A R A T I O N  --------------------------------------*/
/* Internal Variable */
#define KEY1_INT_EXTI_PORTSOURCE   GPIO_PortSourceGPIOB
#define KEY1_INT_EXTI_PINSOURCE    GPIO_PinSource10
#define KEY1_INT_EXTI_LINE         EXTI_Line10
#define KEY1_INT_EXTI_IRQ          EXTI15_10_IRQn

#define KEY1_IRQHandler            EXTI15_10_IRQHandler

#define LEFT_HAND_GPIO    GPIO_Pin_10
#define RIGHT_HAND_GPIO   GPIO_Pin_11

/* External Variable */
extern u32 SysTime_5us;
extern u8 DMAbuffer[RX_BUF_SIZE];
/* Internal Function */
void Bsp_GPIOInit(void);     
void Bsp_USARTInit(void);    //���ڳ�ʼ��
void USART1_IRQHandler(void);
void Bsp_TimerInit(void);
void Bsp_DMAInit(void);    
/* External Function */
extern void Bsp_Init(void);     
extern void USARTSend(USART_TypeDef *com,char Data); //����8λ����
void USART_SendNumber(USART_TypeDef* USARTx, int num);
#endif /*__BSP_H*/
//===========================================  End Of File  ===========================================//
