#include "func_MonHandling.h"
#include "drv_wheel.h"
#include <rthw.h>

// 复位底盘单片机
rt_err_t Robot_Chassis_Reset(rt_bool_t status)
{
    if (status != RT_TRUE)
        return RT_EOK;
    Wheel_Enable(0);
    rt_thread_mdelay(200);
    rt_hw_cpu_reset();
    return RT_EOK;
}
