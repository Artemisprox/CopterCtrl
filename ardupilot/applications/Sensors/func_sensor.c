#include "drv_thread.h"
#include <rtthread.h>
#include "func_sensor.h"
#include "drv_NimingFlow.h"
#include "drv_TF_mini.h"

acc_sensor copter_acc;
pos_sensor copter_pos = {0};
static uint8_t first_flag = 1;

struct rt_semaphore Pos_20ms_sem; /* 用于接收消息的信号量 */
static struct rt_timer Pos_sensor_tim;/* 闭环线程定时器 */

static float Compensate_filter(float* pos , float pos_sensor , float acc , float t)
{
    *pos = k*(*pos + acc*period) + (1-k)*pos_sensor;
}

//使用互补滤波处理位置信息（加速度计估计速度+光流估计速度）
static void Pos_sensor_thread_entry(void *parameter)
{
    while(1)
    {
        if(first_flag)
        {
            copter_pos.height = NiMingFlow_data.distance;
            copter_pos.V_height = 0;
            copter_pos.V_pos_x = 0;
            copter_pos.V_pos_y = 0;
            copter_pos.pos_y = 0;
            copter_pos.pos_x = 0;

            first_flag = 0;
        }else
        {
            /*速度估计*/
            Compensate_filter(&copter_pos.V_pos_x,NiMingFlow_data.Vx_Flow, copter_acc.acc_x,period);
            Compensate_filter(&copter_pos.V_pos_y,NiMingFlow_data.Vy_Flow, copter_acc.acc_y,period);
            Compensate_filter(&copter_pos.V_height,NiMingFlow_data.distance_v, copter_acc.acc_z,period);

            /*位置估计*/
            copter_pos.pos_x += copter_pos.V_pos_x*period;
            copter_pos.pos_y += copter_pos.V_pos_y*period;
            Compensate_filter(&copter_pos.height,NiMingFlow_data.distance, copter_pos.V_height,period);
        }

        rt_sem_take(&Pos_20ms_sem, RT_WAITING_FOREVER);

    }

}

static void Pos_sensor_20ms_IRQHandler(void *parameter)
{
     while (rt_sem_trytake(&Pos_20ms_sem) == RT_EOK)
        continue; // 清空多余的信号量
    rt_sem_release(&Pos_20ms_sem);
}

rt_err_t NiMingFlow_Init(void)
{
	/*定时器处理线程*/
    rt_thread_t thread;
    rt_sem_init(&Pos_20ms_sem, "Position_sem", 0, RT_IPC_FLAG_FIFO);
    thread = rt_thread_create("Pos_message", Pos_sensor_thread_entry, RT_NULL, 2048, THREAD_PRIO_STRIKEPID, 1);
    if (thread != RT_NULL)
        rt_thread_startup(thread);

    /*定时器中断*/
    rt_timer_init(&Pos_sensor_tim, "Pos_Tim", Pos_sensor_20ms_IRQHandler, RT_NULL, 2,
                  RT_TIMER_FLAG_PERIODIC | RT_TIMER_FLAG_SOFT_TIMER);
    /* 启动定时器 */
    rt_timer_start(&Pos_sensor_tim);

   return RT_EOK;
}
