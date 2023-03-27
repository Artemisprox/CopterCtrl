#include "drv_thread.h"
#include "func_IMUCali.h"
#include <rtthread.h>

IMU_redun copter_IMU_redun;
Sensor_RAW_t copter_IMU_RAW;

struct rt_semaphore IMU_1ms_sem; /* 定时信号量 */
struct rt_semaphore IMU_redun_sem; /* 定时信号量 */
static struct rt_timer IMU_redundancy_tim;/* 定时器 */

static void IMU_redundancy_1ms_IRQHandler(void *parameter)
{
    while (rt_sem_trytake(&IMU_1ms_sem) == RT_EOK)
        continue; //取完多余的信号量
    rt_sem_release(&IMU_1ms_sem);
}

void IMU_redundancy_Thread(void *para)
{
    copter_IMU_redun.IMU1_fresh_last = Sensor_RAW_IMU1.DataFreshtime;
    copter_IMU_redun.IMU2_fresh_last = Sensor_RAW_IMU2.DataFreshtime;
    copter_IMU_redun.IMU_using = IMU1_set;
    while(1)
    {
        rt_sem_take(&IMU_1ms_sem,RT_WAITING_FOREVER);
        if(copter_IMU_redun.IMU1_fresh_last == Sensor_RAW_IMU1.DataFreshtime)
        {
            copter_IMU_redun.IMU1_state = 0;
        }else
        {
            copter_IMU_redun.IMU1_state = 1;
        }

        if(copter_IMU_redun.IMU2_fresh_last == Sensor_RAW_IMU2.DataFreshtime)
        {
            copter_IMU_redun.IMU2_state = 0;
        }else
        {
            copter_IMU_redun.IMU2_state = 1;
        }

        if((copter_IMU_redun.IMU1_state == 0) && (copter_IMU_redun.IMU2_state == 0))
            copter_IMU_RAW.RawDataReady = 0;
        else if((copter_IMU_redun.IMU_using == IMU1_set) && (copter_IMU_redun.IMU1_state == 0) && (copter_IMU_redun.IMU2_state == 1))
            copter_IMU_redun.IMU_using = IMU2_set;
        else if ((copter_IMU_redun.IMU_using == IMU2_set) && (copter_IMU_redun.IMU2_state == 0) && (copter_IMU_redun.IMU1_state == 1))
            copter_IMU_redun.IMU_using = IMU1_set;

        switch (copter_IMU_redun.IMU_using)
        {
        case IMU1_set:
            copter_IMU_RAW.Gyro_Raw.x = Sensor_RAW_IMU1.Gyro_Raw.x - IMU1_OffSet.x;
            copter_IMU_RAW.Gyro_Raw.y = Sensor_RAW_IMU1.Gyro_Raw.x - IMU1_OffSet.y;
            copter_IMU_RAW.Gyro_Raw.z = Sensor_RAW_IMU1.Gyro_Raw.x - IMU1_OffSet.z;
            break;
        case IMU2_set:
            copter_IMU_RAW.Gyro_Raw.x = Sensor_RAW_IMU2.Gyro_Raw.x - IMU2_OffSet.x;
            copter_IMU_RAW.Gyro_Raw.y = Sensor_RAW_IMU2.Gyro_Raw.x - IMU2_OffSet.y;
            copter_IMU_RAW.Gyro_Raw.z = Sensor_RAW_IMU2.Gyro_Raw.x - IMU2_OffSet.z;
            break;
        }
        while (rt_sem_trytake(&IMU_redun_sem) == RT_EOK)
            continue; //取完多余的信号量
        rt_sem_release(&IMU_redun_sem);
    }
}

void IMU_WaitForRawData(void)
{
    rt_sem_take(&IMU_redun_sem,RT_WAITING_FOREVER);
}

void IMU_redundancy_init(void)
{
    rt_thread_t thread;
    rt_sem_init(&IMU_1ms_sem, "IMU_tim_sem", 0, RT_IPC_FLAG_FIFO);
    rt_sem_init(&IMU_redun_sem, "IMU_redundancy_sem", 0, RT_IPC_FLAG_FIFO);
    thread = rt_thread_create("State_message", IMU_redundancy_Thread, RT_NULL, 2048, THREAD_PRIO_STRIKEPID, 1);
    if (thread != RT_NULL)
        rt_thread_startup(thread);

    /*定时器线程*/
    rt_timer_init(&IMU_redundancy_tim, "State_decide_tim", IMU_redundancy_1ms_IRQHandler, RT_NULL, 20,
                  RT_TIMER_FLAG_PERIODIC | RT_TIMER_FLAG_SOFT_TIMER);
    /* 定时器开始 */
    rt_timer_start(&IMU_redundancy_tim);


}
