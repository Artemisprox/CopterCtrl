#ifndef __MOD_GIMBAL_CONTROL_H__
#define __MOD_GIMBAL_CONTROL_H__

#include <rtthread.h>
#include <rtdevice.h>
#include <board.h>

#include "robodata.h"
#include "can_receive.h"
#include "drv_Aimbot_Public.h"

// 输出单位：rad
// 获取指定tick时的云台姿态
extern void Gimbal_GetAtti_Tick(AttitudeData_Type *Atti_Out, rt_tick_t Tick);

extern int gimbal_init(void);

// 传入 0 可停止对云台电机的控制
extern void Gimbal_Motor_EN(rt_uint8_t GimbalMotor_Enable);

// 更新控制误差数据
extern void Ctrl_Err_Cal(float SetPitchAng, float SetYawAng);

#endif
