#include "drv_thread.h"
//#include "func_IMU_redundancy.h"
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
    //记录陀螺仪数据更新时间
    copter_IMU_redun.IMU1_fresh_last = Sensor_RAW_IMU1.DataFreshtime;
    copter_IMU_redun.IMU2_fresh_last = Sensor_RAW_IMU2.DataFreshtime;
    //默认使用IMU1
    copter_IMU_redun.IMU_using = IMU1_set;
    while(1)
    {
        rt_sem_take(&IMU_1ms_sem,RT_WAITING_FOREVER);//等待定时中断产生

        if(copter_IMU_redun.IMU1_fresh_last == Sensor_RAW_IMU1.DataFreshtime)//若该时间段内陀螺仪未更新，则认为通信断开
        {
            copter_IMU_redun.IMU1_state = 0;
        }else
        {
            copter_IMU_redun.IMU1_state = 1;
        }

        if(copter_IMU_redun.IMU2_fresh_last == Sensor_RAW_IMU2.DataFreshtime)//若该时间段内陀螺仪未更新，则认为通信断开
        {
            copter_IMU_redun.IMU2_state = 0;
        }else
        {
            copter_IMU_redun.IMU2_state = 1;
        }

        //两个陀螺仪都寄了，判定没救了
        if((copter_IMU_redun.IMU1_state == 0) && (copter_IMU_redun.IMU2_state == 0))
            copter_IMU_RAW.RawDataReady = 0;
        //IMU1失效改用IMU2
        else if((copter_IMU_redun.IMU_using == IMU1_set) && (copter_IMU_redun.IMU1_state == 0) && (copter_IMU_redun.IMU2_state == 1))
            copter_IMU_redun.IMU_using = IMU2_set;
        else if ((copter_IMU_redun.IMU_using == IMU2_set) && (copter_IMU_redun.IMU2_state == 0) && (copter_IMU_redun.IMU1_state == 1))
            copter_IMU_redun.IMU_using = IMU1_set;

        switch (copter_IMU_redun.IMU_using)//根据使用的陀螺仪将数据写入结构体，进入姿态解算部分
        {
        case IMU1_set:
            copter_IMU_RAW.Gyro_Raw.x = Sensor_RAW_IMU1.Gyro_Raw.x - IMU1_OffSet.x;
            copter_IMU_RAW.Gyro_Raw.y = Sensor_RAW_IMU1.Gyro_Raw.x - IMU1_OffSet.y;
            copter_IMU_RAW.Gyro_Raw.z = Sensor_RAW_IMU1.Gyro_Raw.x - IMU1_OffSet.z;
						copter_IMU_RAW.Accl_Raw.x = Sensor_RAW_IMU1.Accl_Raw.x;
            copter_IMU_RAW.Accl_Raw.y = Sensor_RAW_IMU1.Accl_Raw.y;
						copter_IMU_RAW.Accl_Raw.z = Sensor_RAW_IMU1.Accl_Raw.z;
						copter_IMU_RAW.DataRate = Sensor_RAW_IMU1.DataRate;
						copter_IMU_RAW.Temperature = Sensor_RAW_IMU1.Temperature;
						copter_IMU_RAW.DataFreshtime = Sensor_RAW_IMU1.DataFreshtime;
						copter_IMU_RAW.RawDataReady = 1;
            break;
        case IMU2_set:
            copter_IMU_RAW.Gyro_Raw.x = Sensor_RAW_IMU2.Gyro_Raw.x - IMU2_OffSet.x;
            copter_IMU_RAW.Gyro_Raw.y = Sensor_RAW_IMU2.Gyro_Raw.x - IMU2_OffSet.y;
            copter_IMU_RAW.Gyro_Raw.z = Sensor_RAW_IMU2.Gyro_Raw.x - IMU2_OffSet.z;
						copter_IMU_RAW.Accl_Raw.x = Sensor_RAW_IMU2.Accl_Raw.x;
            copter_IMU_RAW.Accl_Raw.y = Sensor_RAW_IMU2.Accl_Raw.y;
						copter_IMU_RAW.Accl_Raw.z = Sensor_RAW_IMU2.Accl_Raw.z;
						copter_IMU_RAW.DataRate = Sensor_RAW_IMU2.DataRate;
						copter_IMU_RAW.Temperature = Sensor_RAW_IMU2.Temperature;
						copter_IMU_RAW.DataFreshtime = Sensor_RAW_IMU2.DataFreshtime;
						copter_IMU_RAW.RawDataReady = 1;
            break;
        }
        while (rt_sem_trytake(&IMU_redun_sem) == RT_EOK)
            continue; //取完多余的信号量
        rt_sem_release(&IMU_redun_sem);
        //释放信号量触发姿态解算线程
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
    thread = rt_thread_create("State_message", IMU_redundancy_Thread, RT_NULL, 2048, THREAD_PRIO_ATTICALCU, 1);
    if (thread != RT_NULL)
        rt_thread_startup(thread);

    /*定时器线程*/
    rt_timer_init(&IMU_redundancy_tim, "IMU_redundancy_tim", IMU_redundancy_1ms_IRQHandler, RT_NULL, 20,
                  RT_TIMER_FLAG_PERIODIC | RT_TIMER_FLAG_SOFT_TIMER);
    /* 定时器开始 */
    rt_timer_start(&IMU_redundancy_tim);


}
