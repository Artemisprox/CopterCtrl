#include "drv_RGB.h"
#include "drv_thread.h"
#include "drv_rgblight.h"

static RGB_types RGB_status;
static RGB_light RGB_con;
static rt_int32_t times = 0;

void red_threetimes(void)
{
//    times = 0;
}

void red_blink_slowly(void)
{
    RGB_status = red_slow;
//    times = 0;
}

void red_keepon(void)
{
    RGB_status = red_keep;    
//    times = 0;
}

void blue_quickly(void)
{
    RGB_status = blue_quick;
//    times = 0;
}

void blue_slowly(void)
{
    RGB_status = blue_slow;
//    times = 0;
}

void blue_keepon(void)
{
    RGB_status = blue_keep;
//    times = 0;
}

void green_keepon(void)
{
    RGB_status = green_keep;
//    times = 0;
}

void green_quickly(void)
{
    RGB_status = green_quick;
//    times = 0;
}

struct rt_semaphore RGB_10ms_sem; /* 定时信号量 */
static struct rt_timer RGB_tim;
/* 定时器 */static void RGB_10ms_IRQHandler(void *parameter)
{
     while (rt_sem_trytake(&RGB_10ms_sem) == RT_EOK)
        continue; //取完多余的信号量
    rt_sem_release(&RGB_10ms_sem);
}

static void RGB_thread(void *para)
{
	RGB_init();
	RGB_status = green_quick;
	while(1)
	{
		switch(RGB_status)
		{
			case red_slow : 
            {
                if(times < SLOW_TIMES )
                {
                    RGB_con.blue = 1.0f;
                    RGB_con.green = 1.0f;
                    RGB_con.red = 0.5f;
                    times++;
                }else if(times < 2*SLOW_TIMES )
                {
                    RGB_con.blue = 1.0f;
                    RGB_con.green = 1.0f;
                    RGB_con.red = 1.0f;
                    times++;
                }else
                    times = 0;
            }
                break;
	        case red_keep : 
                RGB_con.blue = 1.0f;
                RGB_con.green = 1.0f;
                RGB_con.red = 0.5f;
                break;
	        case blue_quick : 
            {
                if(times < QUICK_TIMES )
                {
                    RGB_con.blue = 0.8f;
                    RGB_con.green = 1.0f;
                    RGB_con.red = 1.0f;
                    times++;
                }else if(times < 2*QUICK_TIMES )
                {
                    RGB_con.blue = 1.0f;
                    RGB_con.green = 1.0f;
                    RGB_con.red = 1.0f;
                    times++;
                }else
                    times = 0;
            }
                break;
	        case blue_slow : 
            {
                if(times < SLOW_TIMES )
                {
                    RGB_con.blue = 0.5f;
                    RGB_con.green = 1.0f;
                    RGB_con.red = 1.0f;
                    times++;
                }else if(times < 2*SLOW_TIMES )
                {
                    RGB_con.blue = 1.0f;
                    RGB_con.green = 1.0f;
                    RGB_con.red = 1.0f;
                    times++;
                }else
                    times = 0;
            }
                break;
            case blue_keep:
                RGB_con.blue = 0.8f;
                RGB_con.green = 1.0f;
                RGB_con.red = 1.0f;
                break;
	        case green_keep : 
                RGB_con.blue = 1.0f;
                RGB_con.green = 0.8f;
                RGB_con.red = 1.0f;
                break;
	        case green_quick : 
            {
                if(times < QUICK_TIMES )
                {
                    RGB_con.blue = 1.0f;
                    RGB_con.green = 0.5f;
                    RGB_con.red = 1.0f;
                    times++;
                }else if(times < 2*QUICK_TIMES )
                {
                    RGB_con.blue = 1.0f;
                    RGB_con.green = 1.0f;
                    RGB_con.red = 1.0f;
                    times++;
                }else
                    times = 0;
            }
            break;
		}
        set_RGB(RGB_con.red , RGB_con.green , RGB_con.blue);
        rt_sem_take(&RGB_10ms_sem,RT_WAITING_FOREVER);
	}
	
}

void MainRGB_init()
{
    rt_thread_t thread;
    rt_sem_init(&RGB_10ms_sem, "State_sem", 0, RT_IPC_FLAG_FIFO);
    thread = rt_thread_create("State_message", RGB_thread, RT_NULL, 2048, THREAD_PRIO_STATE, 1);
    if (thread != RT_NULL)
        rt_thread_startup(thread);

    /*定时器线程*/
    rt_timer_init(&RGB_tim, "State_decide_tim", RGB_10ms_IRQHandler, RT_NULL, 10,
                  RT_TIMER_FLAG_PERIODIC | RT_TIMER_FLAG_SOFT_TIMER);
    /* 定时器开始 */
    rt_timer_start(&RGB_tim);
		
}

    
