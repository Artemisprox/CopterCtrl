#include "drv_magazine.h"
#include "robodata.h"

#include <rtthread.h>
#include <rtdevice.h>

#if (SERVO_CTRL_EN)
// 默认舵机状态
#define MAGAZINE_DEFUALT_SET SERVO_CLOSE

// 舵机PWM硬件参数设置
#define MAG_PWMDEV_SET "pwm1" // PWM设备名称
#define MAG_PWMCHANNEL 1 // 定时器PWM通道设置

static struct rt_device_pwm *servo_dev;

// 记录舵机历史设定值，用于减少不必要的PWM占空比设定
static float magazine_PWMSet_REC = -1.0f;
#endif

/**
 * @brief  舵机（弹仓门开关）初始化
 * @param  duty 占空比（0-1000对于0%-%100）
 */
void Magazine_servo_init(void)
{
#if (SERVO_CTRL_EN)
    servo_dev = (struct rt_device_pwm *)rt_device_find(MAG_PWMDEV_SET);
    /*设置周期和脉冲宽度*/
    Magazine_servo_set(MAGAZINE_DEFUALT_SET); // 默认弹舱开关状态
    /* 使能设备 */
    rt_pwm_enable(servo_dev, MAG_PWMCHANNEL);
#endif
}

/**
 * @brief  弹仓舵机角度设定
 * @param  Set_Percent 舵机信号比例，0-1对应舵机整个转动范围
 */
void Magazine_servo_set(int OpenFlag)
{
#if (SERVO_CTRL_EN)
    rt_int32_t PWM_PulseSet_us; // us单位的正脉宽设定值

    if (OpenFlag != magazine_PWMSet_REC)
    {
        // 保护限幅
        if (OpenFlag > 1)
            OpenFlag = 1;
        else if (OpenFlag < 0)
            OpenFlag = 0;
        magazine_PWMSet_REC = OpenFlag; // 记录当前PWM设定状态
        PWM_PulseSet_us = (rt_int32_t)(MAGAZINE_SERVO_CLOSE_PULSE + MAGAZINE_SERVO_OPEN_PULSE * OpenFlag);
        /*设置周期和脉冲宽度*/
        rt_pwm_set(servo_dev, MAG_PWMCHANNEL, 40000000, 2000 * PWM_PulseSet_us); // 20ms周期，按照舵机0.5-2.5ms进行角度百分比设定
    }
#endif
}
