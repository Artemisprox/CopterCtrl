#include "drv_GimbalCom.h"
#include "drv_canthread.h"
#include "HThread_data.h"
#include "can_receive.h"
#include "app_GetRef.h"
#include "HChassis_data.h"
#include "HCanID_data.h"
#include "app_ChassisCtrl.h"
#include "drv_CustomUI_AuxAiming.h"
#include "mod_RefSystem.h"
#include "mod_Monitor.h"
#include "func_MonHandling.h"

#ifdef CORE_USING_HERO
uint8_t MaxPower = 55;
#elif defined CORE_USING_INFANTRY
uint8_t MaxPower = 45;
#endif

gimbal_msg_t gimbal_msg = {0};
rt_tick_t gimbal_msg_freshtick = 0;

/***
 * @brief    向云台端发送热量和射速信息
 * @param    cooling_heat 枪口热量
 * @param    cooling_rate 枪口每秒冷却值
 * @param    cooling_limit 枪口热量上限
 * @param    speed_limit 枪口上限速度,单位m/s
 * @return   rt_err_t：can2报文发送成功or失败
 ***/
static rt_err_t Send_Shooter_Data(rt_uint16_t cooling_heat, rt_uint16_t cooling_rate, rt_uint16_t cooling_limit, rt_uint16_t speed_limit, rt_uint8_t rune_buff)
{
    struct rt_can_msg tx_gimbal;

    tx_gimbal.id = GIMBAL_SHOOT_TX; //设置ID
    tx_gimbal.ide = RT_CAN_STDID;   //标准帧
    tx_gimbal.rtr = RT_CAN_DTR;     //数据帧
    tx_gimbal.priv = 1;             //报文优先级次高
    tx_gimbal.len = 8;              //长度8

    tx_gimbal.data[0] = (rt_uint8_t)(cooling_heat >> 8);
    tx_gimbal.data[1] = (rt_uint8_t)cooling_heat;
    tx_gimbal.data[2] = (rt_uint8_t)(cooling_rate >> 8) | (rune_buff << 7);
    tx_gimbal.data[3] = (rt_uint8_t)cooling_rate;
    tx_gimbal.data[4] = (rt_uint8_t)(cooling_limit >> 8);
    tx_gimbal.data[5] = (rt_uint8_t)cooling_limit;
    tx_gimbal.data[6] = (rt_uint8_t)(speed_limit >> 8);
    tx_gimbal.data[7] = (rt_uint8_t)speed_limit;

    if (!rt_device_write(can2_dev, 0, &tx_gimbal, sizeof(tx_gimbal)))
        return RT_ERROR;
    else
        return RT_EOK;
}

/***
 * @brief   发送xy位置增量给云台
 * @param   fin_xspeed/fin_yspeed 经过缓动后的速度大小
 * @return  rt_err_t 是否成功发送报文
 ***/
static rt_err_t Send_Mixed_Data(rt_int16_t xDelta, rt_int16_t yDelta, rt_uint16_t bullet_v, Ref_team_color_e color)
{
    struct rt_can_msg tx_gimbal;

    tx_gimbal.id = GIMBAL_DATA_TX; //设置ID
    tx_gimbal.ide = RT_CAN_STDID;  //标准帧
    tx_gimbal.rtr = RT_CAN_DTR;    //数据帧
    tx_gimbal.priv = 1;            //报文优先级次高
    tx_gimbal.len = 8;             //长度8

    tx_gimbal.data[0] = (rt_uint8_t)(xDelta >> 8);
    tx_gimbal.data[1] = (rt_uint8_t)xDelta;
    tx_gimbal.data[2] = (rt_uint8_t)(yDelta >> 8);
    tx_gimbal.data[3] = (rt_uint8_t)yDelta;
    tx_gimbal.data[4] = (rt_uint8_t)(bullet_v >> 8);
    tx_gimbal.data[5] = (rt_uint8_t)bullet_v;
    tx_gimbal.data[6] = (rt_uint8_t)color;
    tx_gimbal.data[7] = 0;

    if (!rt_device_write(can2_dev, 0, &tx_gimbal, sizeof(tx_gimbal)))
        return RT_ERROR;
    else
        return RT_EOK;
}

/***
 * @brief    与云台端通信线程,发送热量数据给云台，并从云台端读取控制数据
 * @param    parameter
 * @return   None
 ***/
static void Gimbal_Com_Thread(void *parameter)
{
    SWDG_START(SWDG_GIMBALCOM_ID);      // 监视通信线程正常运行
    while (1)
    {
        // 如果通信存在异常断连, 就直接发送 -1 告知云台
        if ((rt_tick_get() - RefReceiveTime.game_robot_state > 1500) || (rt_tick_get() - RefReceiveTime.power_heat_data > 1500) ||
            (!RefReceiveTime.game_robot_state) || (!RefReceiveTime.power_heat_data))
            Send_Shooter_Data((rt_uint16_t)-1, (rt_uint16_t)-1, (rt_uint16_t)-1, (rt_uint16_t)-1, (rt_uint8_t)-1);
        else
            //发送枪管信息
            Send_Shooter_Data(Ref_Shooter_Cooling_Heat(), Ref_Shooter_Cooling_Rate(), Ref_Shooter_Cooling_Limit(), Ref_Bullet_Speed_Limit() / 100, Ref_Power_Rune_Buff());

        //发送各类信息
        Vector2_t temp = ExMotMod_Get_xyDelta();
        Send_Mixed_Data(temp.x, temp.y, Get_Bullet_Speed(), Ref_Team_Color());

        rt_thread_mdelay(TX_G_PERIOD);
        SWDG_FEED(SWDG_GIMBALCOM_ID);
    }
}

/**
 * @brief    初始化与云台通信，并控制底盘运动部分
 * @param [in]	无
 * @return   true:初始化成功	false:初始化失败
 * @author   lfp
 */
rt_err_t Gimbal_Com_Init(void)
{
    rt_thread_t handler = RT_NULL; //线程句柄

    //初始化云台控制线程
    handler = rt_thread_create(
        "gimbal_ctrl",       //线程名
        Gimbal_Com_Thread,   //线程入口
        RT_NULL,             //入口参数无
        THREAD_STACK_GIMBAL, //线程栈
        THREAD_PRIO_GIMBAL,  //线程优先级
        THREAD_TICK_GIMBAL); //线程时间片大小

    //线程创建失败返回false
    if (handler == RT_NULL)
        return RT_ERROR;

    //线程启动失败返回false
    if (rt_thread_startup(handler) != RT_EOK)
        return RT_ERROR;

    return RT_EOK;
}
////////////////////////////////////////////////////////向外接口函数////////////////////////////////////////////////////////
CTRL_DATA_ZERO(gimbal_ctrl, SMALL_TOP);
/***
 * @brief    处理云台发送的运动控制报文
 * @param    msg     can2报文
 * @return   None
 ***/
void Refresh_Ctldata(struct rt_can_msg *msg)
{
    gimbal_ctrl.vel_x = (float)MSG_READ_2BYTES(0, 1);
    gimbal_ctrl.vel_y = (float)MSG_READ_2BYTES(2, 3);
    gimbal_ctrl.mode = (Motion_mode_e)(msg->data[6]);

    if (gimbal_ctrl.mode == FOLLOW_GIMBAL)
    {
        gimbal_ctrl.fol_ang = (float)MSG_READ_2BYTES(4, 5) / 8192.0f * 360.0f; //云台端通信跟随角单位为8192(对应360°)
        gimbal_ctrl.angvel = 0;
    }
    else if (gimbal_ctrl.mode == SMALL_TOP)
    {
        gimbal_ctrl.angvel = (float)MSG_READ_2BYTES(4, 5);
        gimbal_ctrl.fol_ang = 0;
    }

    MaxPower = msg->data[7];

    //传入数据
    Source_Write_Data(CS_GIMBAL, gimbal_ctrl);
}

/***
 * @brief    处理云台发送的各类数据报文
 * @param    msg     can2报文
 * @return   None
 ***/
void Refresh_Gimbaldata(struct rt_can_msg *msg)
{
    gimbal_msg_freshtick = rt_tick_get();
    rt_memcpy(&gimbal_msg, msg->data, 8);
    // 读取复位指令
    if (gimbal_msg.chassis_reset)
        Robot_Chassis_Reset(RT_TRUE);
}
