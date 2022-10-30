#ifndef __FUNC_GIMBALPID_H__
#define __FUNC_GIMBALPID_H__

#include <drv_GimbalPublic.h>

// 缓启动相关设置
#define SOFTSTART_PALLIM_SET (60)//上电缓启动过程中角速度限幅数值设置 单位dps


/**
* @brief pitch云台PID运行函数（包括角度环角速度环）
* @param [gimbalmotor_t*] motor 云台电机结构体
* @param [float] nowPal 当前角速度值
* @param [float] nowAngle 当前角度值
* @param [float] FeedForwardRate_Set 前馈系数
* @return 无
* @author zzj
*/
extern void Gimbal_Pitch_PID_RUN(Motor_t *motor,float nowPal,float nowAngle, float FeedForwardRate_Set);

/**
* @brief yaw云台PID运行函数（包括角度环角速度环）
* @param [gimbalmotor_t*] motor 云台电机结构体
* @param [float] nowPal 当前角速度值
* @param [float] nowAngle 当前角度值
* @param [float] FeedForwardRate_Set 前馈系数
* @return 无
* @author zzj
*/
extern void Gimbal_Yaw_PID_RUN(Motor_t *motor,float nowPal,float nowAngle, float FeedForwardRate_Set);

/**
* @brief：用于控制上电时的云台缓动, 在PID每次运行之前调用此函数即可，此函数在首次运行时会设定缓动参数，完成缓启动之后会自动恢复PID参数
* @param [pid_t*]	Motor_Pitch_PID：云台Pitch闭环PID结构体
* @author：ych
*/
void Gimbal_SoftStart_Ctrl(pid_t *Motor_Pitch_PID);

#endif
