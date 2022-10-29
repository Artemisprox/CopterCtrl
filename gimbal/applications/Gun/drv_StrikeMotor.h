#ifndef __DRV_STRIKEMOTOR_H__
#define __DRV_STRIKEMOTOR_H__

#include "rtthread.h"
#include "drv_motor.h"

typedef enum
{
    MOTORCTRL_CLR = 1, // 停止闭环
    MOTORCTRL_SPE,     // 转速闭环
    MOTORCTRL_ANG,     // 角度闭环
} Motor_CtrlMode_E;

typedef enum
{
    LaunchMotor = 0,
    RubMotorLeft,
    RubMotorRight,
    GunMotor_All
} Gun_Motor_Enum; // 发射机构的所有电机

// 电机结构体，闭环状态记录结构体
extern Motor_CtrlMode_E CTRLMode_Motor[(int)GunMotor_All];
extern char LaunchMotor_SleepFlag; // 置1可以关闭拨弹电机的PID输出

typedef struct
{
    float DataOut[(int)GunMotor_All];
} StrikeMotor_CtrlData_s;

/**
 * @brief  发射机构电机闭环初始化
 */
extern void StrikeMotor_init(void);

// 读取发射机构是否被初始化
extern int Read_Gun_Inited(void);

// 读取发射机构的指定电机结构体, 注意判断返回值是否为 NULL
extern Motor_t *Read_Gun_Motor(Gun_Motor_Enum GunMotor);

// 关闭摩擦轮
extern void StrikeMotor_Enable(int Enable);

/**
 * @brief  摩擦轮转速设定
 * @param  speed：转速
 */
extern void Rub_speed_set(rt_int16_t speed);

// 读取当前摩擦轮是否开启
extern rt_bool_t Read_Rub_Started(void);

// 读取当前摩擦轮转速设定值
extern rt_int16_t Rub_speed_ReadSet(void);

// 指定轮询函数指针
extern void CTRLRoutine_Set(void (*Func)(void));

#if defined CORE_USING_HERO
// 发射机构闭环逻辑及控制计算
extern void StrikeMotor_CtrlRoutine(StrikeMotor_CtrlData_s *DataOut);
#endif

#endif
