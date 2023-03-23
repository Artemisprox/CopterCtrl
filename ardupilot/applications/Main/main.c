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
#include "drv_RC_PPM.h"
#include "drv_PWM_motor.h"
//#include "drv_buzzer.h"
#include "mod_Monitor.h"
#include "func_bmi088.h"
#include "func_TempCtr.h"
#include "drv_IMU.h"
#include "drv_remote.h"
#include "func_MonitorCfg.h"
#include "drv_thread.h"
#include "mod_Atti.h"
#include "drv_TF_mini.h"
#include "drv_NimingFlow.h"
#include "drv_canthread.h"
#if (!defined CORE_USING_INFANTRY) && (!defined CORE_USING_HERO) && (!defined CORE_USING_COPTER)
#error "Please specify the robot type!"
#endif

int main(void)
{
    // 上电提示音
/*    set_buzzer(4200, 1);
    rt_thread_mdelay(20);
    set_buzzer(0, 1);
*/
    // 所有程序注意, 如果初始化函数存在非 RT_EOK 的返回值单片机会直接复位
/*    if (MONITOR_INIT(1024, 1, THREAD_PRIO_MONITOR) != RT_EOK)
        Robot_Reset_Gimbal();

    if (BMI088_Init() != RT_EOK)
        Robot_Reset_Gimbal();
    if (Atti_init() != RT_EOK)
        Robot_Reset_Gimbal();
    if (IMU_WaitForInit() != RT_EOK)
        Robot_Reset_Gimbal();
*/
	 // NiMingFlow_Init();
	can1_init();
	can2_init();
	//TF_mini_Init();
/*
    if (remote_uart_init() != RT_EOK) // 遥控器初始化
        Robot_Reset_Gimbal();
*/
//	MX_TIM1_PWM_Init();
//	RC_PPM_Init();
	
//		while(1);

//    return RT_EOK;
}
