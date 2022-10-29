#ifndef __DRV_GIMBALPUBLIC_H_
#define __DRV_GIMBALPUBLIC_H_

#include <rtdef.h>

#include <drv_motor.h>
#include "drv_Aimbot_Public.h"
#include "drv_ExactSmooth.h"

// 电机闭环相关设置
// PAL_SCALE_SET 角速度环分频，即周期为PAL_SCALE_SET个1ms
// ANG_SCALE_SET 角度环分频，即周期为ANG_SCALE_SET个1ms
#define PAL_SCALE_SET (1)
#define ANG_SCALE_SET (2)

typedef enum
{
    Aimbot,
    RoboControl
} GimbalSet_Source_Enum; // 用于标记当前设定值数据来源的枚举类型

typedef struct
{
    float Pitch;                      // Pitch设定值
    float Yaw;                        // Yaw设定值
    GimbalSet_Source_Enum Set_Source; // 设定值来源
} GimbalCTRL_Set_Type;

typedef enum
{
    FB_NONE,           // 错误值
    FB_IMU,            // 使用姿态传感器数据
    FB_ENCD,           // 使用编码器
} GimbalFBSelection_E; // 反馈来源枚举

extern Motor_t Yaw;
extern Motor_t Pitch;
extern int Exit_AimbotFlag;
extern float AnglePID_OUT; //角度环输出
extern rt_int8_t motion_mode_record; // 用于记录上次的底盘模式
extern float ViewStartYaw;           // 开始探头的云台yaw角
// 定义一个用来查询二代自瞄状态的结构体
extern Gimbal_SetCal_Type Gimbal_SetData_Out;

extern ExactSmth_CTRL_S MouseXFix_GimbalSet_Filter, MouseYFix_GimbalSet_Filter;

extern float PitchFix;
extern float YawFix;

extern float GimbalSetAdd_Rec;           // 用于一键回头的增量标志
extern float CtrlErr_Pitch, CtrlErr_Yaw; // 计算云台控制误差

extern float Read_YawSet(void);
extern float Read_PitchSet(void);
extern float Read_YawNow(void);
extern float Read_PitchNow(void);
extern float Read_YawSpeedNow(void);

// 修改当前云台预期数据源
extern void Gimbal_FBS_Set_Pitch(GimbalFBSelection_E Set);
extern void Gimbal_FBS_Set_Yaw(GimbalFBSelection_E Set);

#endif
