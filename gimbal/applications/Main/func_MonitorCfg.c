/**
 * @file func_MonitorCfg.c
 * @brief 该文件主要用于工程内部进行监视器的初始化参数配置
 * @author fwlh
 * @version 1.0
 * @date 2022-11-18
 *
 * @copyright Copyright (c) 2022  哈尔滨工业大学(威海)HERO战队
 */
#include "func_MonitorCfg.h"
#include <rthw.h>
#include "mod_gimbal.h"
#include "drv_StrikeMotor.h"

/**
 * @brief 机器人云台单片机复位
 * @author fwlh
 */
void Robot_Reset_Gimbal(void)
{
    // 关闭云台电机输出
    Gimbal_Motor_EN(0);
    // 关闭发射机构电机输出
    StrikeMotor_Enable(0);
    // 等待相关程序响应
    rt_thread_mdelay(200);
    // 直接复位重启
    rt_hw_cpu_reset();
}

/**
 * @brief 线程出现异常时的通用回调函数
 * @author fwlh
 * @param  status           当前状态, 输入 RT_ERROR 代表需要复位
 * @param  param            传入的参数, 该参数为初始化时的自定义参数
 * @return rt_err_t         返回值, 恒定为 RT_EOK
 */
rt_err_t Thread_Err_Exception(rt_bool_t status, void *param)
{
    // 准备复位单片机
    if (status == RT_ERROR)
        Robot_Reset_Gimbal();
    return RT_EOK;
}
