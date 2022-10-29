#ifndef __APP_GETGIM_H__
#define __APP_GETGIM_H__
#include <rtthread.h>
#include "drv_GimbalCom.h"

/***
 * @brief:   获取云台端传来的发弹模式(单发/三连发)
 * @param:   None
 * @return:  枪管模式
 ***/
extern strike_mode_e Get_StrikeMode(void);

/***
 * @brief:   获取云台端上一次弹丸射速设定值
 * @param:   None
 * @return:  上一次弹丸射速设定值
 ***/
extern rt_uint16_t Get_Bullet_Speed_Set(void);

/***
 * @brief 返回底盘运动模式
 * @retval
 ***/
extern ui_motion_mode_e Get_Motion_Mode(void);

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
extern ui_aimbot_mode_e Get_Aimbot_Mode(void);

/**
 * @brief 获取当前自瞄的己方颜色
 * @author fwlh
 * @return My_Color_Enum    自方颜色
 */
extern My_Color_Enum Get_Color_Myself(void);

/***
 * @brief 返回Ui重置标志位
 * @retval
 ***/
extern rt_uint8_t Get_UI_Reset_Flg(void);

//返回弹仓状态,1:开，0:关
extern rt_int8_t Get_Magazine_Status(void);

//热量控制开关状态,0:开，1:关
extern rt_uint8_t Get_HeatLimit_Status(void);

//获取二维按键当前处于哪个菜单下
extern ui_option_display_e Get_UI_Option_Flag(void);

// 读取当前是否需要预留部分超级电容的电量, 返回真需要预留
extern rt_uint8_t Get_PowerRestriction_Status(void);

// 读取当前是否需要超级电容充电, 返回 1 为不需要
extern rt_uint8_t Get_Charge_Cmd(void);

// 读取云台是否强制裁判系统离线的指令, 返回真强制离线
extern rt_uint8_t Get_Ref_Offline_Cmd(void);

// 读取上一次云台控制信息刷新时间
extern rt_tick_t Get_Gim_FreshTick(void);

// 读取云台当前是否刚刚发生复位事件
extern rt_uint8_t Get_Gim_Reset_Status(void);

// 读取当前摩擦轮是否开启
extern rt_bool_t Read_Rub_Started(void);

// 读取当前是否处于客户端模式, 返回真代表在客户端模式
extern rt_bool_t Get_Client_Status(void);

// 获取当前是否处于探头模式
extern rt_bool_t Get_Now_Viewing(void);

// 获取当前的本地底盘模式(0功率优先 1血量优先)
extern Local_Chassis_Mode_Enum Get_Local_Chassis_Type(void);

// 获取当前的本地枪口模式(0爆发优先 1冷却优先 2弹速优先)
extern Local_AmmoBooster_Mode_Enum Get_Local_AmmoBooster_Type(void);

// 获取本地的机器人等级
extern rt_uint8_t Get_Local_Robot_Level(void);

// 获取云台中的离线电机, 从低到高分别为：Yaw、Pitch、右摩擦轮、左摩擦轮、播弹盘
extern rt_uint8_t Get_Gimbal_Motor_Offline_State(void);

// 获取发射机构是否存在卡弹情况
extern rt_bool_t Get_Gimbal_Stuck_State(void);

// 获取视觉模块异常状态
extern Visual_Error_State_Enum Get_Visual_Working_Error_State(void);

#endif
