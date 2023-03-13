#include "drv_RC_PPM.h"
#include "drv_tim13.h"
#include "stm32f4xx_hal.h"
#include <rtthread.h>
#include "board.h"
#include "drv_thread.h"

static int channel_duty[8]={0};//换算得到的各个通道的占空比
static int	pulse_width_us;//用于计算脉冲宽度
static int Now_Tick;
static int Last_Tick;
static int pulse_flag = 0;

RC_PPM_data copter_rec_data;

struct rt_semaphore RC_PPM_rec;

rt_err_t RC_PPM_Init(void)
{
	MX_TIM13_Init();
	
	rt_sem_init(&RC_PPM_rec, "RC_PPM_rec", 0, RT_IPC_FLAG_FIFO);
	
	rt_thread_t RC_PPM = RT_NULL;
	
	RC_PPM = rt_thread_create(
        "RC_PPM_receive",                     //线程名
        RC_PPM_REC_Thread,       //线程入口
        RT_NULL,                      //入口参数无
        1024,                         //线程栈
        THREAD_PRIO_IMU_DATA_COLLECT, //线程优先级
        1);                           //线程时间片大小
	
	rt_pin_mode(PPM_PIN,PIN_MODE_INPUT_PULLDOWN);
	rt_pin_attach_irq(PPM_PIN,PIN_IRQ_MODE_RISING,pulse_process,RT_NULL);
	rt_pin_irq_enable(PPM_PIN,ENABLE);

   return RT_EOK;
}

static void pulse_process(void *args)
{
	int Delt_Tick = 0;
	
	Now_Tick = TIM13_GetCnt();//获取当前时刻信息
	
	if(pulse_flag == 0)
	{
		Last_Tick = Now_Tick;//第一次捕获
		pulse_flag++;
	}else
	{
		Delt_Tick = Now_Tick - Last_Tick; 
		Last_Tick = Now_Tick;
		
		if(Delt_Tick <= 0)
		{
			Delt_Tick += 65536;//跨圈解算
		}
		
		pulse_width_us = Delt_Tick/2;//脉冲的时间长度（us）


/*	在未知遥控器通道数的情况下可使用该方法
		if(pulse_width_us > 2000.0)//长间隔脉冲视为下一信号的起始位
			pulse_flag = 0;
		else if(pulse_width_us > 0 && pulse_width_us < 2000)
		{
			channel_duty[pulse_flag-1] = pulse_width_us/2000.0;//换算为占空比形式
		}
*/
		
//在已知通道数的情况下可以直接根据通道数量解析信号
		if(pulse_width_us > 0 && pulse_width_us <= 2000 )
		{	
			channel_duty[pulse_flag-1] = pulse_width_us;//换算为占空比形式
		}
		pulse_flag++;
		
		if(pulse_flag == 8)//长间隔脉冲视为下一信号的起始位
		{
			pulse_flag = 0;
			rt_sem_release(&RC_PPM_rec);
		}
	}
}

static void (*Remote_Routine)(void);

void Remote_Routine_Set(void (*Func)(void))
{
    Remote_Routine = Func;
}

static int RC_S1_now = 0 ,RC_S2_now = 0 ;

void RC_PPM_REC_Thread(void *Para)
{
	rt_sem_take(&RC_PPM_rec,RT_WAITING_FOREVER);

	//以下为无级变化通道，实时更新
	copter_rec_data.RC_roll 					= channel_duty[0];
	copter_rec_data.RC_pitch 					= channel_duty[1];
	copter_rec_data.RC_yaw 						= channel_duty[2];
	copter_rec_data.RC_throttle 			= channel_duty[3];
	copter_rec_data.RC_roller 				= channel_duty[6];
	
	//以下为开关通道，开关改变触发事件
	RC_S1_now 	= channel_duty[4];
	RC_S2_now 	= channel_duty[5];
		
	//三档开关
	if(RC_S1_now <= s1_low )
			copter_rec_data.RC_switch_left = 500;
	else if (RC_S1_now > s1_low && RC_S1_now < s1_high )
			copter_rec_data.RC_switch_left = 1000;
	else if (RC_S1_now > s1_high )
			copter_rec_data.RC_switch_left = 1500;

//		if( RC_S1_now != copter_rec_data.RC_switch_left )
//    	copter_rec_data.RC_switch_left = RC_S1_now;
//	if( RC_S1_now != copter_rec_data.RC_switch_left )
//	{
//		copter_rec_data.RC_switch_left = RC_S1_now;
//		/*以下为开关对应事件代码*/
//		switch(copter_rec_data.RC_switch_left)
//			{
//				case 500:   break;
//				case 1000:  break;
//				case 1500:  break;
//			}
//	}
	if(RC_S2_now <= s2_low )
			copter_rec_data.RC_switch_right = 500;
	else if (RC_S2_now > s2_low && RC_S2_now < s2_high )
			copter_rec_data.RC_switch_right = 1000;
	else if (RC_S2_now > s2_high )
			copter_rec_data.RC_switch_right = 1500;
	
//	if( RC_S2_now != copter_rec_data.RC_switch_right )
//		copter_rec_data.RC_switch_right = RC_S2_now;
//		/*以下为开关对应事件代码*/
//		switch(copter_rec_data.RC_switch_right)
//			{
//				case 500:   break;
//				case 1000:  break;
//				case 1500:  break;
//			}
//	}

	Remote_Routine();//遥控器数据处理

}

