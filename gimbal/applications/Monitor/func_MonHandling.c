#include "func_MonHandling.h"
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

rt_err_t Thread_Err_Exception(rt_bool_t status)
{
    if (status == RT_ERROR)
        Robot_Reset_Gimbal();
    return RT_EOK;
}
