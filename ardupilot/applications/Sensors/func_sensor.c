#include "drv_thread.h"
#include "func_sensor.h"
#include "drv_NimingFlow.h"
#include "drv_TF_mini.h"
#include "drv_dataserve.h"
#include "drv_IMU.h"
#include "velocity_estimator.h"
#include "arm_math.h"

data_fresh_time fresh_time_last;
//static acc_sensor copter_acc;
static pos_sensor copter_pos = {0};
static uint8_t first_flag = 1;
static rt_int8_t Package_ID , IMU_ID ;
static IMU_t Copter_IMU;

struct rt_semaphore Pos_20ms_sem; /* 用于接收消息的信号量 */
static struct rt_timer Pos_sensor_tim;/* 闭环线程定时器 */

static float average_filter(float value , float sample)
{
	static int num = 0 ;
	static float data[3] ;
	float out = 0;
	if(num < 3)
	{
		data[num] = sample;
		num++;
		return (value*(num/1.0f/(num+1)) + sample/num) ;
	}else
	{
		data[num%3] = sample;
		num ++;
		int i = 0;
		out = 0;
		for( i = 0; i < 3; i++ )
			out += data[i]/3.0f;
	}
	return out;
}


float jscope_test1 , jscope_test2;
float temp1[2] , temp2[2] ;
//使用互补滤波处理位置信息（加速度计估计速度+光流估计速度）
static void Pos_sensor_thread_entry(void *parameter)
{
    while(1)
    {
        /*从数据服务器更新数据*/
	    IMU_t *p_2 =  Package_Pionter_Single(IMU_ID,IMU_t);
	    Copter_IMU = *p_2 ;
	    Package_Write_Pionter_End(IMU_ID,IMU_t);
        Copter_IMU.yaw = 0;

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
                            //Compensate_filter(&copter_pos.V_height,NiMingFlow_data.distance_v, copter_acc.acc_z);
                            //Compensate_filter(&copter_pos.distance,NiMingFlow_data.distance, copter_pos.V_height);
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
             
					temp1[0] =  arm_cos_f32(Copter_IMU.roll/360.0f*2.0f*3.1415926f)*arm_cos_f32(Copter_IMU.pitch/360.0f*2.0f*3.1415926f)*TF_mini_data.distance;
					temp1[1] = arm_cos_f32(Copter_IMU.roll/360.0f*2.0f*3.1415926f)*arm_cos_f32(Copter_IMU.pitch/360.0f*2.0f*3.1415926f)*TF_mini_data.distance_v;
									
                    if( height_data_del_time != 0)
                    {
                        if(DATA_FUSE)//启用板载加速度计融合
                        {
                            /*高度估计*/
														jscope_test2 = temp1[1];
														temp1[1] = average_filter(copter_pos.V_height , temp1[1] );
														jscope_test1 = temp1[1];
                            Velocity_estimate(temp1[0] , temp1[1] , temp2 );
														copter_pos.distance = temp2[0];
														copter_pos.V_height = temp2[1];
                            //Compensate_filter(&copter_pos.distance, TF_mini_data.distance, copter_acc.acc_z, PERIOD);
                        }else
                        {
														copter_pos.V_height = temp1[1];
							//average_filter(copter_pos.V_height , TF_mini_data.distance_v );
                            //copter_pos.V_height = HERO_IMU.height_estimator;
                            copter_pos.distance = temp1[0];
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
                           // Compensate_filter(&copter_pos.V_pos_x,NiMingFlow_data.Vx_Flow, copter_acc.acc_x);
                            //Compensate_filter(&copter_pos.V_pos_y,NiMingFlow_data.Vy_Flow, copter_acc.acc_y);
                        }else
                        {
                            copter_pos.V_pos_x = NiMingFlow_data.Vx_Flow;
                            copter_pos.V_pos_y = NiMingFlow_data.Vy_Flow;
                        }

                        /*位置估计*/
                        copter_pos.pos_x += copter_pos.V_pos_x*PERIOD;
                        copter_pos.pos_y += copter_pos.V_pos_y*PERIOD;
                    
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
    IMU_ID = Package_Find_Num("IMU");
	
		Velocity_estimator_init();
	/*定时器处理线程*/
    rt_thread_t thread;
    rt_sem_init(&Pos_20ms_sem, "Position_sem", 0, RT_IPC_FLAG_FIFO);
    thread = rt_thread_create("Pos_message", Pos_sensor_thread_entry, RT_NULL, 2048, THREAD_PRIO_SENSOR_DATA, 1);
    if (thread != RT_NULL)
        rt_thread_startup(thread);

    /*定时器中断*/
    rt_timer_init(&Pos_sensor_tim, "Pos_Tim", Pos_sensor_20ms_IRQHandler, RT_NULL, 20,
                  RT_TIMER_FLAG_PERIODIC | RT_TIMER_FLAG_SOFT_TIMER);
    /* 启动定时器 */
    rt_timer_start(&Pos_sensor_tim);

   return RT_EOK;
}
