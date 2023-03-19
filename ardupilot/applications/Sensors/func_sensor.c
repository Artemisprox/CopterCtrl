#include "drv_thread.h"
#include "func_sensor.h"
#include "drv_NimingFlow.h"
#include "drv_TF_mini.h"
#include "drv_dataserve.h"

data_fresh_time fresh_time_last;
acc_sensor copter_acc;
pos_sensor copter_pos = {0};
static uint8_t first_flag = 1;
rt_int8_t Package_ID;

struct rt_semaphore Pos_20ms_sem; /* 用于接收消息的信号量 */
static struct rt_timer Pos_sensor_tim;/* 闭环线程定时器 */

static void Compensate_filter(float* pos , float pos_sensor , float acc , float t)
{
    *pos = k*(*pos + acc*period) + (1-k)*pos_sensor;
}

//使用互补滤波处理位置信息（加速度计估计速度+光流估计速度）
static void Pos_sensor_thread_entry(void *parameter)
{
    while(1)
    {
        uint32_t height_data_del_time = 0;/*高度数据更新时间*/
        uint32_t pos_data_del_time = 0;/*水平位置数据更新时间*/

        if(NiMingFlow_data.height_data_Valid || TF_mini_data.Data_fresh_time)//高度数据可用
        {
            if(first_flag)
            {
                /*第一次进行参数初始化*/
                copter_pos.distance = NiMingFlow_data.distance;
                copter_pos.V_height = 0;
                copter_pos.V_pos_x = 0;
                copter_pos.V_pos_y = 0;
                copter_pos.pos_y = 0;
                copter_pos.pos_x = 0;
                first_flag = 0;
            }else
            {   
                if (USING_FLOW)
                {
                     /*如果高度数据未更新，则认为数据不准确*/
                    height_data_del_time = NiMingFlow_data.height_data_fresh_time - fresh_time_last.height_time;
                    
                    if( height_data_del_time != 0)
                    {
                        if(DATA_FUSE)//启用板载加速度计融合
                        {
                            /*高度估计*/
                            Compensate_filter(&copter_pos.V_height,NiMingFlow_data.distance_v, copter_acc.acc_z,period);
                            Compensate_filter(&copter_pos.distance,NiMingFlow_data.distance, copter_pos.V_height,period);
                        }else
                        {
                            copter_pos.V_height = NiMingFlow_data.distance_v;
                            copter_pos.distance = NiMingFlow_data.distance;
                        }
                        
                        copter_pos.height_valid = 1;//更新时间校验和数据校验都通过，认为数据可用
                        fresh_time_last.height_time = NiMingFlow_data.height_data_fresh_time;
                    }else
                    {
                        copter_pos.height_valid = 0;
                    }
                   
                }else
                {
                    /*如果高度数据未更新，则认为数据不准确*/
                    height_data_del_time = TF_mini_data.Data_fresh_time - fresh_time_last.height_time;
                    
                    if( height_data_del_time != 0)
                    {
                        if(DATA_FUSE)//启用板载加速度计融合
                        {
                            /*高度估计*/
                            Compensate_filter(&copter_pos.V_height,TF_mini_data.distance, copter_acc.acc_z,period);
                            Compensate_filter(&copter_pos.distance, TF_mini_data.distance_v, copter_acc.acc_z, period);
                        }else
                        {
                            copter_pos.V_height = TF_mini_data.distance_v;
                            copter_pos.distance = TF_mini_data.distance;
                        }
                        
                        copter_pos.height_valid = 1;//更新时间校验和数据校验都通过，认为数据可用
                        fresh_time_last.height_time = TF_mini_data.Data_fresh_time;
                    }else
                    {
                        copter_pos.height_valid = 0;
                    }
                }
                
                if(NiMingFlow_data.pos_data_Valid)
                {
                    /*如果位置数据未更新，则认为数据不准确*/
                    pos_data_del_time = NiMingFlow_data.pos_data_fresh_time - fresh_time_last.pos_time;

                    if( pos_data_del_time != 0)
                    {
                        if(DATA_FUSE)//启用板载加速度计融合
                        {
                             /*速度估计*/
                            Compensate_filter(&copter_pos.V_pos_x,NiMingFlow_data.Vx_Flow, copter_acc.acc_x,period);
                            Compensate_filter(&copter_pos.V_pos_y,NiMingFlow_data.Vy_Flow, copter_acc.acc_y,period);
                        }else
                        {
                            copter_pos.V_pos_x = NiMingFlow_data.Vx_Flow;
                            copter_pos.V_pos_y = NiMingFlow_data.Vy_Flow;
                        }

                        /*位置估计*/
                        copter_pos.pos_x += copter_pos.V_pos_x*period;
                        copter_pos.pos_y += copter_pos.V_pos_y*period;
                    
                        copter_pos.pos_valid = 1;//更新时间校验和数据校验都通过，认为数据可用
                        fresh_time_last.pos_time = NiMingFlow_data.pos_data_fresh_time;

                    }else
                    {
                        copter_pos.pos_valid = 0;
                    }

                }else
                {
                    copter_pos.pos_valid = 0;
                }
            }
        }else
        {
            copter_pos.height_valid = 0;
        }

        /*数据服务器进行数据更新*/
        pos_sensor *p = Package_Pionter_Single(Package_ID,pos_sensor);
        *p = copter_pos;
        Package_Write_Pionter_End(Package_ID,pos_sensor);

        rt_sem_take(&Pos_20ms_sem, RT_WAITING_FOREVER);

    }

}

static void Pos_sensor_20ms_IRQHandler(void *parameter)
{
     while (rt_sem_trytake(&Pos_20ms_sem) == RT_EOK)
        continue; // 清空多余的信号量
    rt_sem_release(&Pos_20ms_sem);
}

rt_err_t Sensor_Init(void)
{
    if(USING_FLOW)
        NiMingFlow_Init();
    else
        TF_mini_Init();
    
    Package_Pionter_Add("pos_sensor", pos_sensor);
		Package_ID = Package_Find_Num("pos_sensor");

	/*定时器处理线程*/
    rt_thread_t thread;
    rt_sem_init(&Pos_20ms_sem, "Position_sem", 0, RT_IPC_FLAG_FIFO);
    thread = rt_thread_create("Pos_message", Pos_sensor_thread_entry, RT_NULL, 2048, THREAD_PRIO_STRIKEPID, 1);
    if (thread != RT_NULL)
        rt_thread_startup(thread);

    /*定时器中断*/
    rt_timer_init(&Pos_sensor_tim, "Pos_Tim", Pos_sensor_20ms_IRQHandler, RT_NULL, 20,
                  RT_TIMER_FLAG_PERIODIC | RT_TIMER_FLAG_SOFT_TIMER);
    /* 启动定时器 */
    rt_timer_start(&Pos_sensor_tim);

   return RT_EOK;
}
