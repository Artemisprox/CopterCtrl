/************************************** Copyright ******************************
 *                 (C) Copyright 2020,China, HITwh.
 *                            All Rights Reserved
 *
 *                     HITwh Excellent Robot Organization
 *                     https://github.com/HERO-ECG
 *                     https://gitee.com/HIT-718LC
 *
 * FileName   : app_monitor.c
 * Version    : v3.2
 * Author     : mqy,LvFp
 * Date       : 2020-02-14
 * Instructions：1.在drv_Monitor.h文件内枚举swdg_deviceID里添加新的ID值。
 *               2.在All_Swdg_Create(void)函数里，用Swdg_Create添加新的软件看门狗对象
 *				  3.在func_MonCallback.c内添加新的异常处理函数。
 *	              4.在需要监视的地方使用Swdg_Feed来喂狗，注意喂狗地方要有周期性，且当监视的设备离线时这个地方不能被执行到
 *				  5.拨动核心板左拨码开关，可关闭报警中的蜂鸣器
 *				  6.推荐喂狗写法
 *					#ifdef  CORE_USING_MONITOR
 *						Swdg_Feed(F_Left_ID);
 *					#endif
 *				  7.硬件看门狗，当使能宏BSP_USING_WDT时，如果监视器线程被阻塞超过1s，
 *					则程序会一直复位。通过在main函数开头写报警函数提示程序崩了。
 *					而main.c里的预定义里的内容是监视器除了Swdg_Feed之外唯一暴露在监视器文件外的代码。
 *					推荐写法：左写在main.c包含的头文件里，右包含在main函数开头
 *					#ifdef BSP_USING_WDT			| #ifdef BSP_USING_WDT
 *						#include "func_Monitor.h"	|   PROGRAM_RESET    //程序一直复位提醒程序
 *					#endif							| #endif
 ********************************************************************************/
#include "drv_thread.h"
#include "mod_Monitor.h"
#include "func_MonHandling.h"

extern swdg_dev_t *monitor_hp;

static rt_thread_t Monitor_Thread = RT_NULL; // 监视器线程
static rt_timer_t Monitor_Timer = RT_NULL;   // 监视器定时器
static rt_sem_t Monitor_Sem = RT_NULL;       // 监视器信号量

static void Monitor_Timer_Timeout_Handler(void *parameter)
{
    if (Monitor_Sem)
    {
        while (rt_sem_trytake(Monitor_Sem) == RT_EOK)
            continue;
        rt_sem_release(Monitor_Sem);
    }
}

/**
 * @brief    监视器线程
 * @param    None
 * @return   None
 * @author   mqy
 */
static void Monitor_Thread_Entry(void *parameter)
{
    while (1)
    {
        swdg_dev_t *swdg_dev_tem = monitor_hp;
        while (swdg_dev_tem != RT_NULL)
        {
            // 需要判断是否初始化, 否则会操作未知内存空间
            if ((SWDG_INITED_FLAG == swdg_dev_tem->flag_inited) && (RT_TRUE == swdg_dev_tem->if_start))
            {
                //不用swdg_dev_tem->if_error == RT_FALSE判断,会耗时过长
                if ((swdg_dev_tem->if_error == RT_FALSE) && (swdg_dev_tem->time_deadline < rt_tick_get()))
                {
                    swdg_dev_tem->if_error = RT_TRUE;
#if defined(BSP_USING_RGB_LIGHT) || defined(BSP_USING_BUZZER)
                    Mlist_Insert(swdg_dev_tem); //挂载异常监视器
#endif
                    if (swdg_dev_tem->handle != RT_NULL)
                        //没有处理函数则不调用
                        (*swdg_dev_tem->handle)(swdg_dev_tem->if_error); //处理函数
                }
            }
            swdg_dev_tem = swdg_dev_tem->next;
        }

//喂硬件看门狗
#ifdef BSP_USING_WDT
        Hwdt_Feed();
#endif

        //延时
        rt_sem_take(Monitor_Sem, RT_WAITING_FOREVER);
    }
}

/***
 * @brief  所有软件看门狗的统一创建
 * @param  None
 * @return None
 * @author Lvfp
 * @Date   2020-11-28
 ***/
static rt_err_t All_Swdg_Create(void)
{
    rt_err_t res[(int)MONITOR_ID_ALL] = {RT_EOK};
    res[(int)SWDG_IMU_ID] = Swdg_Create(SWDG_IMU_ID, ALARM_BROWN, RT_TRUE, 50, Thread_Err_Exception, SWDG_INITED_FLAG);
    res[(int)SWDG_TEMPCTRL_ID] = Swdg_Create(SWDG_TEMPCTRL_ID, ALARM_BROWN, RT_TRUE, 5000, Thread_Err_Exception, SWDG_INITED_FLAG);
    res[(int)SWDG_CAN1_ID] = Swdg_Create(SWDG_CAN1_ID, ALARPINK, RT_TRUE, 5000, Thread_Err_Exception, SWDG_INITED_FLAG);
    res[(int)SWDG_CAN2_ID] = Swdg_Create(SWDG_CAN2_ID, ALARPINK, RT_TRUE, 5000, Thread_Err_Exception, SWDG_INITED_FLAG);
    res[(int)SWDG_GIMBAL_ID] = Swdg_Create(SWDG_GIMBAL_ID, ALARPINK, RT_TRUE, 50, Thread_Err_Exception, SWDG_INITED_FLAG);
    res[(int)SWDG_AIMBOT_SEND_ID] = Swdg_Create(SWDG_AIMBOT_SEND_ID, ALARPINK, RT_TRUE, 200, Thread_Err_Exception, SWDG_INITED_FLAG);
    res[(int)SWDG_GUNDATA_ID] = Swdg_Create(SWDG_GUNDATA_ID, ALARPINK, RT_TRUE, 50, Thread_Err_Exception, SWDG_INITED_FLAG);
    res[(int)SWDG_STRIKE_ID] = Swdg_Create(SWDG_STRIKE_ID, ALARPINK, RT_TRUE, 50, Thread_Err_Exception, SWDG_INITED_FLAG);
    res[(int)SWDG_ROBOCONTROL_ID] = Swdg_Create(SWDG_ROBOCONTROL_ID, ALARPINK, RT_TRUE, 500, Thread_Err_Exception, SWDG_INITED_FLAG);

    for (int i = 0; i < (int)MONITOR_ID_ALL; i++)
        if (res[i] != RT_EOK)
            return res[i];

    return RT_EOK;
}

/**
 * @brief    该函数初始化监视器
 * @param    None
 * @return   RT_ERROR：初始化失败
 * @author   mqy,lfp
 */
rt_err_t Monitor_Init(void)
{
/*报警线程初始化*/
#if (defined(BSP_USING_RGB_LIGHT) || defined(BSP_USING_BUZZER)) && (defined(MONITOR_NEED_ALARM))
    if (Alarm_Init() != RT_EOK)
        return RT_ERROR;
#endif

    //软件看门狗结构体创建
    if (All_Swdg_Create() != RT_EOK)
        return RT_ERROR;

    // 监视器信号量创建
    Monitor_Sem = rt_sem_create("monitor sem", 0, RT_IPC_FLAG_FIFO);
    if (!Monitor_Sem)
        return RT_ERROR;
    rt_sem_trytake(Monitor_Sem);
    // 监视器定时器创建
    Monitor_Timer = rt_timer_create(
        "monitor timer", Monitor_Timer_Timeout_Handler,
        RT_NULL, MONITOR_PERIOD, RT_TIMER_FLAG_PERIODIC | RT_TIMER_FLAG_SOFT_TIMER);
    if (Monitor_Timer)
    {
        if (rt_timer_start(Monitor_Timer) != RT_EOK)
            return RT_ERROR;
    }
    else
        return RT_ERROR;
    //监视器线程创建
    Monitor_Thread = rt_thread_create(
        "monitor", Monitor_Thread_Entry, RT_NULL,
        1024, 1, THREAD_PRIO_MONITOR);

    // 查看是否创建成功
    if (Monitor_Thread != RT_NULL)
    {
        if (rt_thread_startup(Monitor_Thread) != RT_EOK)
            return RT_ERROR;
    }
    else
        return RT_ERROR;

        /*硬件看门狗初始化*/
#ifdef BSP_USING_WDT
    if (Hwdt_Init() != RT_EOK)
        return RT_ERROR;
    // 初始化看门狗以后进行一次喂狗
    Hwdt_Feed();
#endif

    return RT_EOK;
}

/**
 * @brief    给看门狗喂食(移出ID对应的报警节点,复位剩余时间)
 * @param    mID 看门狗id
 * @return   None
 * @author   mqy
 */
void Swdg_Feed(swdg_deviceID mID)
{
    swdg_dev_t *swdg_dev_tem = monitor_hp;

    while (swdg_dev_tem != RT_NULL) //索引
    {
        // 需要判断该模块是否真的已经被初始化了
        if ((SWDG_INITED_FLAG == swdg_dev_tem->flag_inited) && (mID == swdg_dev_tem->ID))
        {
            swdg_dev_tem->time_deadline = rt_tick_get() + swdg_dev_tem->time_threshold;
            if (swdg_dev_tem->if_error == RT_TRUE) //对异常看门狗节点进行恢复
            {
                swdg_dev_tem->if_error = RT_FALSE;
                if (swdg_dev_tem->handle != RT_NULL)
                    //没有处理函数则不调用
                    (*swdg_dev_tem->handle)(swdg_dev_tem->if_error); //恢复函数
#if defined(BSP_USING_RGB_LIGHT) || defined(BSP_USING_BUZZER)
                Mlist_Remove(mID); //移出对应的报警链表
#endif
            }
            return;
        }
        swdg_dev_tem = swdg_dev_tem->next;
    }
}

/**
 * @brief 启动一个监视器
 * @author fwlh
 * @param  mID              待启动监视器的 ID
 */
void Swdg_Start(swdg_deviceID mID)
{
    swdg_dev_t *swdg_dev_tem = monitor_hp;

    while (swdg_dev_tem != RT_NULL) //索引
    {
        // 需要判断该模块是否真的已经被初始化了
        if ((SWDG_INITED_FLAG == swdg_dev_tem->flag_inited) && (mID == swdg_dev_tem->ID))
        {
            swdg_dev_tem->if_start = RT_TRUE;
            // 更新标志位的时候一定要注意更新时刻
            swdg_dev_tem->time_deadline = rt_tick_get() + swdg_dev_tem->time_threshold;
            return;
        }
        swdg_dev_tem = swdg_dev_tem->next;
    }
}

/**
 * @brief    查询看门狗对象是否异常
 * @param    mID 看门狗id
 * @return   RT_FALSE：正常，RT_TRUE：异常
 * @author   lfp
 */
rt_bool_t Swdg_If_Error(swdg_deviceID mID)
{
    swdg_dev_t *swdg_dev_tem = monitor_hp;

    while (swdg_dev_tem != RT_NULL) //索引
    {
        // 需要判断该模块是否真的已经被初始化了
        if ((SWDG_INITED_FLAG == swdg_dev_tem->flag_inited) && (mID == swdg_dev_tem->ID))
            return swdg_dev_tem->if_error;

        swdg_dev_tem = swdg_dev_tem->next;
    }

    return RT_FALSE; // ID没找到，返回false
}
