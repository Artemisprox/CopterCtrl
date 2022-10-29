#include "app_oled.h"

static rt_thread_t OLED_Show_App_Thread_tid = RT_NULL; // OLED显示控制线程句柄
static Cap_Energy_Type CAP_Energy_data;

//OLED显示控制线程
void OLED_Show_App_Thread(void* para)
{
    OLED_Mod_Init();//初始化OLED显示模块

    while(1)
    {
        //轮询显示电量和状态信息
        Get_CAP_Energy(&CAP_Energy_data);
        if (CAP_Energy_data.Supply_Ready==CAP_SUPPLY_READY)
        {
            //此时电容电量已经达到输出要求
            OLED_FreshEnergy(CAP_Energy_data.Energy_Percentage, SHOW_SQARETYPE_SOLID);
        }
        else
        {
            //此时电容电量不满足输出要求
            OLED_FreshEnergy(CAP_Energy_data.Energy_Percentage, SHOW_SQARETYPE_HOLLOW);
        }
        if(Get_CAN_Return_Valid()==RT_EOK)
        {
            //缓冲能量数据有效，显示实心进度条
            OLED_FreshBuff(Get_EnergyBuff(), SHOW_SQARETYPE_SOLID);
        }
        else
        {
            //缓冲能量数据无效，显示空心满进度条
            OLED_FreshBuff(1, SHOW_SQARETYPE_HOLLOW);
        }
        
        OLED_Fresh_PowerSet(Get_Charge_PowerSet());
        rt_thread_delay(200);//200ms刷新一次进度条
    }
}

//初始化OLED显示控制线程
rt_err_t OLED_Show_App_Init()
{
    //启动显示控制线程
    OLED_Show_App_Thread_tid = rt_thread_create("SHOW_BAR",
                                                OLED_Show_App_Thread, RT_NULL,
                                                1024,
                                                OLED_SHO_APP_THREAD_PRIO, 2);
    /* 如果获得线程控制块，启动这个线程 */
    if (OLED_Show_App_Thread_tid != RT_NULL)
        rt_thread_startup(OLED_Show_App_Thread_tid);
    return RT_EOK;
}
