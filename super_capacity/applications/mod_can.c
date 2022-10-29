#include "mod_can.h"

static rt_thread_t CAN_Mod_Thread_tid = RT_NULL; // CAN通信处理线程句柄

RoboState_Type RoboState_Data;

char MOD_CAN_Fresh = 0;// 用于记录CAN工作状态的变量，在mod charge中的看门狗程序中清零

//需要在外部定义新数据处理函数
extern void Robostate_NewData(RoboState_Type*);

static Cap_Energy_Type CAP_State;

//CAN接收发送数据处理线程
void CAN_Mod_Thread(void*para)
{
    int Time_Count = 0;
    while(1)
    {//轮询新的功率设定值，循环发送电容数据
        MOD_CAN_Fresh = 1;
        if (_RoboState_Data.NewData)
        {// 检查缓冲区内是否有新数据 如果有新数据，则进行读取
            RoboState_Data.NewData = 1;
            _RoboState_Data.NewData = 0;
            RoboState_Data.ChargeClose_Flag = _RoboState_Data.ChargeClose_Flag;
            RoboState_Data.Energy_Buff = _RoboState_Data.Energy_Buff;
            RoboState_Data.Power_Set = _RoboState_Data.Power_Set;
            RoboState_Data.LocalPowerCtrl_Flag = _RoboState_Data.LocalPowerCtrl_Flag;
            //新的回传数据
            Robostate_NewData(&RoboState_Data);
        }
        Time_Count++;
        if(Time_Count>10)
        {
            Time_Count = 0;
            // 定时进行反馈
            Get_CAP_Energy(&CAP_State);
            if (CAP_State.Supply_Ready == CAP_SUPPLY_READY)
            {
                CAN1_SEND_CAPState(CAP_State.Energy_Percentage, (rt_uint16_t)-1);
            }
            else
            {
                CAN1_SEND_CAPState(0, (rt_uint16_t)-1);
            }
        }
        else if(Time_Count<0)
        {
            Time_Count = 0;
        }
        rt_thread_delay(1); //1ms轮询一次电容状态数据
    }
}


//初始化CAN接收发送数据处理控制线程
rt_err_t CAN_Mod_Init()
{
    //启动CAN控制线程
    CAN_Mod_Thread_tid = rt_thread_create("CAN_MOD",
                                          CAN_Mod_Thread, RT_NULL,
                                          1024,
                                          CAN_MOD_THREAD_PRIO, 2);
    /* 如果获得线程控制块，启动这个线程 */
    if (CAN_Mod_Thread_tid != RT_NULL)
        rt_thread_startup(CAN_Mod_Thread_tid);
    return RT_EOK;
}
