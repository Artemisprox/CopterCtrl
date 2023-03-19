#include "drv_battery.h"
#include "drv_dataserve.h"
#include "drv_thread.h"
#include "drv_dataserve.h"

battery copter_power = {0};
struct rt_semaphore battery_100ms_sem; /* 用于定时的信号量 */
struct rt_semaphore battery_rec_sem; /* 用于接收信息的信号量 */
static struct rt_timer battery_tim;/* 闭环线程定时器 */
static rt_int8_t Package_ID;
/**
 * @brief  读取can中的电池数据
 * @param  rxmsg：反馈报文数据
 * @param  power：电池数据结构
 * @retval None
 */
void battery_readmsg(rt_uint8_t rxmsg[])
{
    copter_power.voltage = (float)( rxmsg[3] << 8*3 | rxmsg[2] << 8*2 | rxmsg[1] << 8 | rxmsg[0]);
    copter_power.current = (float)( rxmsg[7] << 8*3 | rxmsg[6] << 8*2 | rxmsg[5] << 8 | rxmsg[4]);
    copter_power.fresh_time = rt_tick_get();

    rt_sem_release(&battery_rec_sem);//接收到数据则告诉电池数据处理线程
}

static void battery_100ms_IRQHandler(void *parameter)
{
     while (rt_sem_trytake(&battery_100ms_sem) == RT_EOK)
        continue; // 清空多余的信号量
    rt_sem_release(&battery_100ms_sem);
}
    
static void battery_thread_entry(void *parameter)
{
   
    while(1)
    {
        if(rt_sem_trytake(&battery_rec_sem) == RT_EOK)
        {
            copter_power.Battery_data_rec = 1;//接收到电池数据
        }else
        {
            copter_power.Battery_data_rec = 0;//未收到电池数据
        }
        
        if( copter_power.voltage >= BATTERY_LOW )//检查电池电量是否过低，过低则不允许起飞/提醒尽快降落
            copter_power.Battery_status = 1;
        else copter_power.Battery_status = 0;

        /*数据服务器写入*/
        /*数据服务器写入*/
        battery *p =  Package_Pionter_Single(Package_ID,battery);
        *p = copter_power;
        Package_Write_Pionter_End(Package_ID,battery);

        rt_sem_take(&battery_100ms_sem,RT_WAITING_FOREVER);
    }

}

rt_err_t Battery_Init(void)
{
    /*数据服务器初始化*/
    Package_Pionter_Add("battery", battery);
	  Package_ID = Package_Find_Num("battery");
	/*定时器处理线程*/
    rt_thread_t thread;
    rt_sem_init(&battery_100ms_sem, "battery_sem", 0, RT_IPC_FLAG_FIFO);
    rt_sem_init(&battery_rec_sem, "battery_rec_sem", 0, RT_IPC_FLAG_FIFO);
    thread = rt_thread_create("battery_message", battery_thread_entry, RT_NULL, 2048, THREAD_PRIO_STRIKEPID, 1);
    if (thread != RT_NULL)
        rt_thread_startup(thread);

    /*定时器中断*/
    rt_timer_init(&battery_tim, "battery_tim", battery_100ms_IRQHandler, RT_NULL, 100,
                  RT_TIMER_FLAG_PERIODIC | RT_TIMER_FLAG_SOFT_TIMER);
    /* 启动定时器 */
    rt_timer_start(&battery_tim);

   return RT_EOK;
}
