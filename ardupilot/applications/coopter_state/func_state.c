#include "func_state.h"
#include "drv_thread.h"
#include "func_sensor.h"
#include "func_remote.h"
#include "drv_IMU.h"

remote_data copter_remote;
pos_sensor copter_pos;
IMU_t copter_atti;

struct rt_semaphore State_20ms_sem; /* 用于接收消息的信号量 */
static struct rt_timer State_decide_tim;/* 闭环线程定时器 */
data_check copter_data_valid = {0};
status copter_status = {0};

static void State_decide_20ms_IRQHandler(void *parameter)
{
     while (rt_sem_trytake(&State_20ms_sem) == RT_EOK)
        continue; // 清空多余的信号量
    rt_sem_release(&State_20ms_sem);
}

static void copter_mode_change(int16_t remote_mode)
{
    
}
    
static void State_decide_thread_entry(void *parameter)
{
   
    while(1)
    {
        /*数据服务器进行数据更新*/


        copter_data_valid.pos_valid = copter_pos.pos_valid;
        copter_data_valid.height_valid = copter_pos.height_valid;
        copter_data_valid.atti_valid = copter_atti.atti_ready;
        copter_data_valid.battery_OK = 1;

        

    }

}

rt_err_t StateDecide_Init(void)
{
	/*定时器处理线程*/
    rt_thread_t thread;
    rt_sem_init(&State_20ms_sem, "Position_sem", 0, RT_IPC_FLAG_FIFO);
    thread = rt_thread_create("Pos_message", State_decide_thread_entry, RT_NULL, 2048, THREAD_PRIO_STRIKEPID, 1);
    if (thread != RT_NULL)
        rt_thread_startup(thread);

    /*定时器中断*/
    rt_timer_init(&State_decide_tim, "State_decide_tim", State_decide_20ms_IRQHandler, RT_NULL, 20,
                  RT_TIMER_FLAG_PERIODIC | RT_TIMER_FLAG_SOFT_TIMER);
    /* 启动定时器 */
    rt_timer_start(&State_decide_tim);

   return RT_EOK;
}
