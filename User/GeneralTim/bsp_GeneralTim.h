#ifndef __BSP_GENERALTIME_H
#define __BSP_GENERALTIME_H


#include "stm32f10x.h"
#include "stm32f10x_tim.h"


/************ͨ�ö�ʱ��TIM�������壬ֻ��TIM2��3��4��5************/
// ��ʹ�ò�ͬ�Ķ�ʱ����ʱ�򣬶�Ӧ��GPIO�ǲ�һ���ģ����Ҫע��
// ��������Ĭ��ʹ��TIM1
#define BRE_TIM_IRQn								TIM3_IRQn

#define            GENERAL_TIM                   TIM1
#define            GENERAL_TIM_APBxClock_FUN     RCC_APB2PeriphClockCmd
#define            GENERAL_TIM_CLK               RCC_APB2Periph_TIM1
#define            GENERAL_TIM_Period            2500
#define            GENERAL_TIM_Prescaler         350
// TIM1 ����Ƚ�ͨ��1
#define            GENERAL_TIM_CH1_GPIO_CLK      RCC_APB2Periph_GPIOA
#define            GENERAL_TIM_CH1_PORT          GPIOA
#define            GENERAL_TIM_CH1_PIN           GPIO_Pin_8

// TIM1 ����Ƚ�ͨ��4
#define            GENERAL_TIM_CH4_GPIO_CLK      RCC_APB2Periph_GPIOA
#define            GENERAL_TIM_CH4_PORT          GPIOA
#define            GENERAL_TIM_CH4_PIN           GPIO_Pin_11
/**************************��������********************************/

void GENERAL_TIM_Init(void);


#endif	/* __BSP_GENERALTIME_H */


