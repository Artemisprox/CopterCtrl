#include "app_GetGim.h"
#include "drv_utils.h"

extern gimbal_msg_t gimbal_msg;
static rt_uint16_t bullet_speed = 0; //云台端的上一次发弹的弹速设定值

/***
 * @brief:   获取云台端传来的发弹模式(单发/三连发)
 * @param:   None
 * @return:  枪管模式
 ***/
strike_mode_e Get_StrikeMode(void)
{
    return ((gimbal_msg.strike_mode == 0) ? SINGLE_STRIKE : ((strike_mode_e)gimbal_msg.strike_mode));
}

/***
 * @brief:   获取云台端上一次弹丸射速设定值
 * @param:   None
 * @return:  上一次弹丸射速设定值
 ***/
rt_uint16_t Get_Bullet_Speed_Set(void)
{
    return bullet_speed;
}

/***
 * @brief 返回底盘运动模式
 * @retval
 ***/
ui_motion_mode_e Get_Motion_Mode(void)
{
    return (ui_motion_mode_e)gimbal_msg.motion_mode;
}

/***
 * @brief    //返回自瞄模式
 * @retval   0: 普通自瞄
 * @retval   1: 小能量机关
 * @retval   2: 大能量机关
 * @retval   3: 静止击打前哨战
 * @retval   4: 旋转击打前哨战
 * @retval   5: 第三种击打前哨战的模式
 * @retval   6: 吊射模式
 ***/
ui_aimbot_mode_e Get_Aimbot_Mode(void)
{
    return (ui_aimbot_mode_e)gimbal_msg.aimbot_mode;
}

/**
 * @brief 获取当前自瞄的己方颜色
 * @author fwlh
 * @return My_Color_Enum    自方颜色
 */
My_Color_Enum Get_Color_Myself(void)
{
    return (My_Color_Enum)gimbal_msg.self_color;
}

/***
 * @brief 返回Ui重置标志位
 * @retval
 ***/
rt_uint8_t Get_UI_Reset_Flg(void)
{
    return gimbal_msg.ui_reset;
}

//返回弹仓状态,1:开，0:关
rt_int8_t Get_Magazine_Status(void)
{
    return gimbal_msg.magazine_status;
}

//热量控制开关状态
rt_uint8_t Get_HeatLimit_Status(void)
{
    return gimbal_msg.heatlimit_status;
}

//获取二维按键当前处于哪个菜单下
ui_option_display_e Get_UI_Option_Flag(void)
{
    return (ui_option_display_e)gimbal_msg.current_menu;
}

// 读取当前是否需要预留部分超级电容的电量
rt_uint8_t Get_PowerRestriction_Status(void)
{
    return gimbal_msg.power_restrictions_lim;
}

// 读取当前是否需要超级电容充电, 返回 1 为不需要
rt_uint8_t Get_Charge_Cmd(void)
{
    return gimbal_msg.capacity_close_flag;
}

// 读取云台是否强制裁判系统离线的指令
rt_uint8_t Get_Ref_Offline_Cmd(void)
{
    return gimbal_msg.force_refsystem_offline;
}

// 读取上一次云台控制信息刷新时间
rt_tick_t Get_Gim_FreshTick(void)
{
    return gimbal_msg_freshtick;
}

// 读取云台当前是否刚刚发生复位事件
rt_uint8_t Get_Gim_Reset_Status(void)
{
    return ((gimbal_msg.tick_now < 20) ? 1 : 0);
}

// 读取当前摩擦轮是否开启
rt_bool_t Read_Rub_Started(void)
{
    return (gimbal_msg.rub_started ? RT_TRUE : RT_FALSE);
}

// 读取当前是否处于客户端模式, 返回真代表在客户端模式
rt_bool_t Get_Client_Status(void)
{
    return (gimbal_msg.now_client_control ? RT_TRUE : RT_FALSE);
}

// 获取当前是否处于探头模式
rt_bool_t Get_Now_Viewing(void)
{
    return (rt_bool_t)gimbal_msg.now_viewing;
}

// 获取当前的本地底盘模式(0功率优先 1血量优先)
Local_Chassis_Mode_Enum Get_Local_Chassis_Type(void)
{
    return (Local_Chassis_Mode_Enum)gimbal_msg.set_chassis_mode;
}

// 获取当前的本地枪口模式(0爆发优先 1冷却优先 2弹速优先)
Local_AmmoBooster_Mode_Enum Get_Local_AmmoBooster_Type(void)
{
#if defined CORE_USING_HERO
    return (Local_AmmoBooster_Mode_Enum)(gimbal_msg.set_ammobooster_mode ? 2 : 0);
#elif defined CORE_USING_INFANTRY
    return (Local_AmmoBooster_Mode_Enum)gimbal_msg.set_ammobooster_mode;
#endif
}

// 获取本地的机器人等级
rt_uint8_t Get_Local_Robot_Level(void)
{
    return gimbal_msg.set_level + 1;
}

// 获取云台中的离线电机, 从低到高分别为：Yaw、Pitch、右摩擦轮、左摩擦轮、播弹盘
rt_uint8_t Get_Gimbal_Motor_Offline_State(void)
{
    rt_uint8_t OfflineMotor = 0;
    OfflineMotor |= (!gimbal_msg.yaw_motor_online) << 0;
    OfflineMotor |= (!gimbal_msg.pitch_motor_online) << 1;
    OfflineMotor |= (!gimbal_msg.right_rub_motor_online) << 2;
    OfflineMotor |= (!gimbal_msg.left_rub_motor_online) << 3;
    OfflineMotor |= (!gimbal_msg.launch_motor_online) << 4;
    return OfflineMotor;
}

// 获取发射机构是否存在卡弹情况
rt_bool_t Get_Gimbal_Stuck_State(void)
{
    return gimbal_msg.strike_stuck;
}

// 获取视觉模块异常状态
Visual_Error_State_Enum Get_Visual_Working_Error_State(void)
{
    switch (gimbal_msg.visual_com_online << 1 | gimbal_msg.visual_working_correct)
    {
    case (1 << 1 | 1):
        return Visual_Working_EOK;
    case (1 << 1 | 0):
        return Visual_Working_Failure;
    case (0 << 1 | 1):
    case (0 << 1 | 0):
    default:
        return Visual_Com_Failure;
    }
}
