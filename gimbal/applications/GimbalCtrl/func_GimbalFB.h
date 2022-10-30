#ifndef __FUNC_GIMBALFB_H__
#define __FUNC_GIMBALFB_H__

#include "drv_GimbalPublic.h"
#include "drv_IMU.h"

typedef struct
{
    GimbalFBSelection_E FBS_Now_Pitch; // 当前反馈量
    GimbalFBSelection_E FBS_Now_Yaw; // 当前反馈量

    AttitudeData_Type ENCD_BIAS;  // 当前编码器偏置角度
    AttitudeData_Type ENCD_ATTI; // 由编码器数据计算的角度
    AttitudeData_Type IMU_ATTI; // 姿态传感器数据
    AttitudeData_Type FB_This;    // 供云台本次计算PID使用的数据
    AttitudeData_Type DeltaAtti;    // 当前：编码器角度 - 姿态传感角度
} FBData_S;                       // 用于处理云台编码器和姿态数据切换的结构体

extern FBData_S GimbalFB; // 反馈量信息计算结构体

// 工具：float跨圈处理
extern void FloatDataFix(float *data, float UpLim, float DownLim);

// 刷新：云台反馈量计算结构体 GimbalFB
extern void Gimbal_PID_FB_Fresh(void);

// 获取当前云台角度反馈量
extern void Gimbal_FB_Get(AttitudeData_Type *FB_Atti);

// 设定所需的反馈数据源
extern void GimbalPitch_FB_Select_Set(GimbalFBSelection_E FB_Set);
extern void GimbalYaw_FB_Select_Set(GimbalFBSelection_E FB_Set);

// 读取现在使用的的反馈数据源
extern GimbalFBSelection_E GimbalPitch_FB_Select_Get(void);
extern GimbalFBSelection_E GimbalYaw_FB_Select_Get(void);

// 初始化云台控制反馈信息
extern void GimbalFB_Init(void);

#endif
