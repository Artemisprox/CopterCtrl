/*
 * Copyright (c) 2006-2018, RT-Thread Development Team
 *
 * SPDX-License-Identifier: Apache-2.0
 *
 * Change Logs:
 * Date           Author       Notes
 * 2018-11-06     SummerGift   first version
 */

#include <rtthread.h>

#include "drv_buzzer.h"
#include "mod_Monitor.h"
#include "drv_canthread.h"
#include "func_gun.h"
#include "func_bmi088.h"
#include "mod_Atti.h"
#include "func_TempCtr.h"
#include "drv_IMU.h"
#include "drv_remote.h"
#include "app_MenuSet.h"
#include "mod_aimbot_V2.h"
#include "mod_gimbal.h"
#include "app_robocontrol.h"
#include "func_MonitorCfg.h"
#include "drv_thread.h"

#if (!defined CORE_USING_INFANTRY) && (!defined CORE_USING_HERO)
#error "Please specify the robot type!"
#endif

int main(void)
{
    // 上电提示音
    set_buzzer(4200, 1);
    rt_thread_mdelay(20);
    set_buzzer(0, 1);

    // 所有程序注意, 如果初始化函数存在非 RT_EOK 的返回值单片机会直接复位
    if (MONITOR_INIT(1024, 1, THREAD_PRIO_MONITOR) != RT_EOK)
        Robot_Reset_Gimbal();

    if (can1_init() != RT_EOK)
        Robot_Reset_Gimbal();
    if (can2_init() != RT_EOK)
        Robot_Reset_Gimbal();

    if (BMI088_Init() != RT_EOK)
        Robot_Reset_Gimbal();
    if (Atti_init() != RT_EOK)
        Robot_Reset_Gimbal();
    if (IMU_WaitForInit() != RT_EOK)
        Robot_Reset_Gimbal();

    if (Gun_Init() != RT_EOK) // 发射机构初始化
        Robot_Reset_Gimbal();

    if (remote_uart_init() != RT_EOK) // 遥控器初始化
        Robot_Reset_Gimbal();
    if (Ctrl_Menu_Init() != RT_EOK)
        Robot_Reset_Gimbal();

    if (AimbotV2_Init() != RT_EOK) // 启动二代自瞄
        Robot_Reset_Gimbal();
    if (gimbal_init() != RT_EOK) // 云台初始化
        Robot_Reset_Gimbal();
    if (RoboControl_init() != RT_EOK)
        Robot_Reset_Gimbal();

    return RT_EOK;
}
