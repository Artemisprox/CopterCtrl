#include "app_charge.h"

#include "drv_monitor.h"

static rt_thread_t CAP_Ctrl_App_Thread_tid = RT_NULL; // 超级电容总控线程句柄
static char NewState_Flag = 0;                        //新数据标志位
static char LocalCtrl_Flag = 1;                       // 底盘指定是否需要本地功率闭环
static int Power_Set_Rem;                             //裁判系统功率上限记录
static char ChargeClose_Flag;                              // 当前云台是否允许进行充电
static volatile float PowerData_Energy_Buff;          //缓冲能量
static char CAN_NoData_Count;                         //用来计数判断底盘反馈通信是否发生了异常

//获取裁判系统通信有效性
rt_err_t Get_CAN_Return_Valid()
{
    if (CAN_NoData_Count > 10)
    {
        return RT_ERROR;
    }
    else
    {
        return RT_EOK;
    }
}

// 获取当前是否允许进行充电的指令, 返回值为真代表不允许充电
rt_uint8_t Get_Charge_Permission(void)
{
    return ChargeClose_Flag;
}

//获取底盘能量缓存数值，用于OLED显示
float Get_EnergyBuff()
{
    return (float)PowerData_Energy_Buff / CHARGE_ENERGYBUFF_MAX;
}

//解算裁判系统数据反馈
void Robostate_NewData(RoboState_Type *Robostate)
{
    if (Power_Set_Rem != Robostate->Power_Set)
    { // 更新电源设定值
        // 对数据进行限幅
        if (Robostate->Power_Set > 300)
        {
            Robostate->Power_Set = 300;
        }
        else if (Robostate->Power_Set < 30)
        {
            Robostate->Power_Set = 30;
        }

        Power_Set_Rem = Robostate->Power_Set;
    }
    ChargeClose_Flag = Robostate->ChargeClose_Flag;
    PowerData_Energy_Buff = Robostate->Energy_Buff;
    LocalCtrl_Flag = Robostate->LocalPowerCtrl_Flag;
    NewState_Flag = 1;
}

//超级电容功率设定值控制算法 --对底盘缓冲能量余量进行控制
void CAP_Ctrl_PowerSet(void)
{
    float ADD_POWER_temp;
    float PowerSetTemp;
    float PowerTemp_Error; // 用于纯P控制缓冲能量剩余量，缓冲能量剩余量过多为正
    if ((CAN_NoData_Count > 10) || LocalCtrl_Flag)
    {
        //裁判系统反馈离线，需要使用本地数据进行计算
        PowerSetTemp = Power_Set_Rem * (1 - CHARGE_SPEED_SAVE_SET);
        if (PowerSetTemp < 30)
        {
            PowerSetTemp = 30;
        }
        Charge_Ctrl_set_P(PowerSetTemp);
        if (CAN_NoData_Count > 11)
            CAN_NoData_Count = 11; //防止溢出
    }
    else // 20ms内仍有新的有效反馈
    {
        PowerTemp_Error = PowerData_Energy_Buff - POWER_TEMP_RSV; // 纯P控制的ERR计算
        if (PowerData_Energy_Buff > 25)
        {                                                            // 缓冲能量余量充足
            ADD_POWER_temp = PowerTemp_Error / 1.0f + Power_Set_Rem; // 按照1S内消耗完多余能量的功率进行设定
            if (ADD_POWER_temp > CHARGE_POWER_MAX_SET)
            {
                ADD_POWER_temp = CHARGE_POWER_MAX_SET;
            }
            else if (ADD_POWER_temp < 0)
            {
                ADD_POWER_temp = 0;
            }
            Charge_Ctrl_set_P(ADD_POWER_temp); // 以特定额外功率使用缓冲能量，且保证数据延迟不超过800ms时不会出现超功率
        }
        else
        {                             // 缓冲能量余量极少
            Charge_Ctrl_set_P(10.0f); // 立即降低充电速度，防止掉血
        }
    }
}

//超级电容总控线程
void CAP_Ctrl_App_Thread(void *para)
{
    CAN_NoData_Count = 125;    //默认认为没有连接数据反馈
    PowerData_Energy_Buff = 0; //默认认为缓冲能量为空
    //初始化CAN通信
    CAN_Mod_Init();
    while (rt_pin_read(HW_KEY_PR) == PIN_HIGH && rt_pin_read(HW_KEY1) == PIN_LOW)
    {
        rt_thread_delay(10);
    }

    mod_charge_init(POWER_DEFAULT_SET);
    Power_Set_Rem = POWER_DEFAULT_SET;

    //指示灯闪烁
    HWFUN_LED1_ON;
    HWFUN_LED2_ON;
    HWFUN_LED3_ON;
    { // 蜂鸣器
        set_buzzer(1800, 0.5);
        rt_thread_delay(40);
        set_buzzer(2800, 0.3);
        rt_thread_delay(75);
        set_buzzer(0, 0);
    }

    WDT_Init(); // 开始充电了，可以初始化看门狗

    while (1)
    { //轮询新数据
        if (NewState_Flag)
        {
            CAN_NoData_Count = 0; //清除标志位
            NewState_Flag = 0;    //清除标志位
        }
        else
        {
            CAN_NoData_Count++; //对没有数据的情况进行计数
        }
        // 如果与底盘通信断开自动重新开始充电
        if (Get_CAN_Return_Valid() != RT_EOK)
            ChargeClose_Flag = 0;
        CAP_Ctrl_PowerSet();
        rt_thread_delay(2); // 2ms轮询一次
    }
}

//初始化超级电容总控线程
rt_err_t CAP_Ctrl_App_Init()
{
    //启动总控线程
    CAP_Ctrl_App_Thread_tid = rt_thread_create("CAP_CTRL",
                                               CAP_Ctrl_App_Thread, RT_NULL,
                                               1024,
                                               CAP_CTRL_APP_THREAD_PRIO, 2);
    /* 如果获得线程控制块，启动这个线程 */
    if (CAP_Ctrl_App_Thread_tid != RT_NULL)
        rt_thread_startup(CAP_Ctrl_App_Thread_tid);
    return RT_EOK;
}
