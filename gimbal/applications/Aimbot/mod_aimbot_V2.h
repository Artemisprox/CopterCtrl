#ifndef __MOD_AIMBOT_V2_H__
#define __MOD_AIMBOT_V2_H__

#include "can_receive.h"
#include "func_Aimbot_Com.h"
#include "drv_IMU.h"
#include "drv_Aimbot_Public.h"
#include <stdbool.h>

#define AIMBUFF_PITCH_SPEED_MAX (50) // 自瞄能量机关状态下，云台设定值允许的最大变化速度 dps
#define AIMBUFF_YAW_SPEED_MAX (50)   // 自瞄能量机关状态下，云台设定值允许的最大变化速度 dps
#define AIMBOT_PITCH_SPEED_MAX (90) // 自瞄状态下，云台设定值允许的最大变化速度 dps
#define AIMBOT_YAW_SPEED_MAX (120)   // 自瞄状态下，云台设定值允许的最大变化速度 dps

// 刷新云台限幅设定值
// 输入单位：度
extern void Refresh_Gimbal_Lim(float Up, float Down);

//刷新弹速设定值
//输入单位：m/s
extern void Refresh_Muzzle_V(float V_New);

// 切换自瞄模式函数
extern void Refresh_VisualMode(drv_VisualMode_e Mode_Set);

// 输入输出角度单位：°
// 输入输出零位：与IMU数据零位一致
// 获取所需的云台角度设定值函数
// 函数一般每ms运行一次，通过限制单次变化量，可以实现缓动或设定值变化速度限幅
extern void Aimbot_Get_GimbalSet(Gimbal_SetCal_Type *Gimbal_SetData_Out);

// 二代自瞄启动程序
extern int AimbotV2_Init(void);

/**
 * @brief 重启视觉给定的设定角度的滤波器
 * @author fwlh
 * @param  NewPitch         重启以后的 Pitch 设定值
 * @param  NewYaw           重启以后的 Yaw 设定值
 */
extern void Smooth_Restart_VisualSet(float NewPitch, float NewYaw);

/**
 * @brief 在电控端向视觉开放开火和云台控制的权限
 * @author fwlh
 * @param  Enable           输入真时开放权限, 反之关闭相关权限
 */
extern void Aimbot_Enable_Visual_CtrlFire(bool Enable);

#endif
