#ifndef __FUNC_GIMBALSET_H__
#define __FUNC_GIMBALSET_H__

#include <drv_GimbalPublic.h>
#include "robodata.h"
#include "drv_SetPlanning.h"
#include "drv_ExactSmooth.h"

#define GIMBAL_LIM_LEN (1.5f) //定义云台设定值接近限幅值的缓冲区的大小 单位 °

typedef struct
{
    float UpLim;
    float DownLim;
} GimbalLiPItch_Type;

typedef enum
{
    Pitch_Set,
    Yaw_Set
} SetData_Type_Enum;

extern ExactSmth_CTRL_S Smooth_PitchAngleSet, Smooth_YawAngleSet;

/**
 * @brief 云台设定值获取主函数
 * @param [GimbalCTRL_Set_Type*] Gimbal_Setang：云台设定值结构体 调用函数前为上一次的设定值，调用后为这一次的设定值
 * @return 无
 * @author ych
 */
extern void Gimbal_getset(GimbalCTRL_Set_Type *Gimbal_Setang);

/**
 * @brief 设定值及相关功能初始化
 * @param [GimbalCTRL_Set_Type*] SetSTR：姿态角设定值结构体指针
 * @param [float] PitchSet: 上电后的初始姿态角设定值
 * @param [float] YawSet: 上电后的初始姿态角设定值
 * @return 无
 * @author ych
 */
extern void GimbalSet_Init(GimbalCTRL_Set_Type *SetSTR, float PitchSet, float YawSet);

// 用于外部读取当前的实际角度设定值
extern float Read_Real_Set(SetData_Type_Enum Data);

#if defined CORE_USING_HERO
// 用于退出或进入吊射模式
extern void Enter_Dangling_Mode(rt_uint8_t Enter);

// 读取当前是否处于吊射模式中
extern rt_uint8_t Read_Dangling_Mode(void);
#endif /* CORE_USING_HERO */

#endif
