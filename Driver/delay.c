
#include "delay.h"

u32 g_total_delay_time;
void delay_5us(u32 num)    //整体速度通过改变延迟函数计数来改变，从而达到不改变正弦模型（改变电机）
{
    u32 endTime=num*1.4+SysTime_5us;
	  g_total_delay_time += num;
	  while(SysTime_5us<endTime);
}
void delay_5us2(u32 num)  //整体速度通过改变延迟函数计数来改变，从而达到不改变正弦模型（改变电磁铁）
{
    u32 endTime=num*1.6+SysTime_5us;
	  g_total_delay_time += num;
	  while(SysTime_5us<endTime);
}
//===========================================  End Of File  ===========================================//
