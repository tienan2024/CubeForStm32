
#include "delay.h"

u32 g_total_delay_time;
void delay_5us(u32 num)    //�����ٶ�ͨ���ı��ӳٺ����������ı䣬�Ӷ��ﵽ���ı�����ģ�ͣ��ı�����
{
    u32 endTime=num*1.4+SysTime_5us;
	  g_total_delay_time += num;
	  while(SysTime_5us<endTime);
}
void delay_5us2(u32 num)  //�����ٶ�ͨ���ı��ӳٺ����������ı䣬�Ӷ��ﵽ���ı�����ģ�ͣ��ı�������
{
    u32 endTime=num*1.6+SysTime_5us;
	  g_total_delay_time += num;
	  while(SysTime_5us<endTime);
}
//===========================================  End Of File  ===========================================//
