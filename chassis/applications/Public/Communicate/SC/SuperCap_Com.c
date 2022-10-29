#include "SuperCap_Com.h"
#include "drv_canthread.h"
#include "HCanID_data.h"
#include "HThread_data.h"
#include "mod_Monitor.h"
#include "app_GetRef.h"
#include "drv_GimbalCom.h"
#include "mod_RefSystem.h"
#include "app_GetGim.h"

static float remain_cap;         //剩余电容量,0.0~1.0。1.0为满电
static rt_tick_t Fresh_Tick = 0; // 上次数据刷新的时刻

/***
 * @brief    向超级电容端发送数据
 * @note     当输入都为-1时，表示告知超级电容，裁判系统的通信断开
 * @param    Whether_To_Charge 当前是否需要充电
 * @param    power_buffer      底盘功率缓冲
 * @param    power_limit       底盘功率上限
 * @return   rt_err_t：can2报文发送成功or失败
 ***/
static rt_err_t Send_To_Scpr(uint8_t Whether_To_Charge, rt_uint16_t power_buffer, rt_uint16_t power_limit)
{
    struct rt_can_msg tx_Scpr;

    tx_Scpr.id = SCPR_TX;       //设置ID
    tx_Scpr.ide = RT_CAN_STDID; //标准帧
    tx_Scpr.rtr = RT_CAN_DTR;   //数据帧
    tx_Scpr.priv = 2;           //报文优先级次次高
    tx_Scpr.len = 8;            //长度8

    if (power_limit == 0)
    {
        // 可以按照规则填入缺省值
#if defined CORE_USING_INFANTRY
        power_limit = 40;
#elif defined CORE_USING_HERO
        power_limit = 50;
#endif
    }

    //发送数据
    tx_Scpr.data[0] = Whether_To_Charge;
    tx_Scpr.data[4] = power_buffer >> 8;
    tx_Scpr.data[5] = power_buffer;
    tx_Scpr.data[6] = power_limit >> 8;
    tx_Scpr.data[7] = power_limit;

    if (!rt_device_write(can2_dev, 0, &tx_Scpr, sizeof(tx_Scpr)))
        return RT_ERROR;
    else
        return RT_EOK;
}

static uint8_t ChargeClose_Flag = 1;
/***
 * @brief    与超级电容端通信线程
 * @param    None
 * @return   None
 ***/
static void Scpr_Thread(void *parameter)
{
    static rt_uint8_t first_fg = 0; //是否第一次进入
    SWDG_START(SWDG_SCAPCOM_ID);

    while (1)
    {
        ChargeClose_Flag = Get_Charge_Cmd();
        if (first_fg && (rt_tick_get() - RefReceiveTime.power_heat_data < 800) &&
#ifdef CORE_USING_INFANTRY
            Ref_Chassis_Power_Limit() >= 40)
#elif defined CORE_USING_HERO
            Ref_Chassis_Power_Limit() >= 55)
#else
            0)
#endif
        {
            Send_To_Scpr(ChargeClose_Flag, Ref_Chassis_Power_Buffer(), Ref_Chassis_Power_Limit());
        }
        else
        {
            /*发送给超级电容端的数据全部置-1,与裁判系统通信断掉，告知超级电容端*/
            Send_To_Scpr(ChargeClose_Flag, (uint16_t)-1, MaxPower);

            /*第一次发送报文*/
            if (!first_fg)
                first_fg = 1;
        }

        rt_thread_mdelay(SC_PERIOD);
        SWDG_FEED(SWDG_SCAPCOM_ID);
    }
}

/**
 * @brief    初始化与超级电容端通信
 * @param [in]	无
 * @return   true:初始化成功	false:初始化失败
 * @author  lfp
 */
rt_err_t Scpr_Com_Init(void)
{
    rt_thread_t Scpr_handler = RT_NULL; //线程句柄

    //初始化超级电容通信线程
    Scpr_handler = rt_thread_create(
        "SCR_ctrl",      //线程名
        Scpr_Thread,     //线程入口
        RT_NULL,         //入口参数无
        THREAD_STACK_SC, //线程栈
        THREAD_PRIO_SC,  //线程优先级
        THREAD_TICK_SC); //线程时间片大小

    //线程创建失败返回false
    if (Scpr_handler == RT_NULL)
    {
        return RT_ERROR;
    }

    //线程启动失败返回false
    if (rt_thread_startup(Scpr_handler) != RT_EOK)
    {
        return RT_ERROR;
    }

    return RT_EOK;
}

/***
 * @brief    处理超级电容端发送报文
 * @param    msg can2报文
 * @return   None
 ***/
void Refresh_Scprdata(struct rt_can_msg *msg)
{
    remain_cap = *(float *)(&msg->data[0]);
    Fresh_Tick = rt_tick_get();
}

/**
 * @brief 获取超级电容控制板当前是否在线
 * @author fwlh
 * @return rt_bool_t        超级电容控制板在线返回真
 */
rt_bool_t Read_Super_Capacity_Online(void)
{
    return ((Fresh_Tick && (rt_tick_get() - Fresh_Tick < 200)) ? RT_TRUE : RT_FALSE);
}

/***
 * @brief    获得超级电容剩余电量
 * @param    None
 * @return   剩余电容量,0~100。100为满电
 ***/
float Get_RemainCapcity(void)
{
#ifdef USE_CAPACITY
    if (Fresh_Tick && (rt_tick_get() - Fresh_Tick < 500))
        return remain_cap * 100.f;
    else
        return 40.f;
#else
    return 100.f;
#endif /* USE_CAPACITY */
}
