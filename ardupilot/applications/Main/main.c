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
#include "func_motor.h"
// #include "drv_buzzer.h"
#include "mod_Monitor.h"
#include "func_SensorRAW.h"
#include "func_TempCtr.h"
#include "drv_IMU.h"
#include "drv_remote.h"
#include "func_MonitorCfg.h"
#include "drv_thread.h"
#include "mod_Atti.h"
#include "func_remote.h"
#include "func_sensor.h"
#include "drv_canthread.h"
#include "func_state.h"
#define TEST 1

#if (TEST)
#define KEY_TEST 1
#define CAN_TEST 0
#define RGB_TEST 0
#define PWM_TEST 0
#define UART_TEST 0
#define IMU_TEST 1
#include "CAN_TEST.h"
#include "RGB_TEST.h"
#include "KEY_TEST.h"
#include "PWM_TEST.h"
#include "UART_TEST.h"
#include "IMU_TEST.h"
#endif

#if (!defined CORE_USING_INFANTRY) && (!defined CORE_USING_HERO) && (!defined CORE_USING_COPTER)
#error "Please specify the robot type!"
#endif

int main(void)
{
	
#if (CAN_TEST)
    CAN_Init();
#endif

#if (RGB_TEST)
    RGB_Init();
#endif

#if (KEY_TEST)
   // KEY_Init();
#endif

#if (PWM_TEST)
    PWM_Init();
#endif

#if (UART_TEST)
    UART_Init();
#endif

#if (IMU_TEST)
  //  IMU_Init();
#endif

	RC_init();
	//remote_uart_init();
	
	Atti_init();
	IMU_WaitForInit();
	
	StateDecide_Init();
	
	Motor_init();
	
    // 上电提示音
    /*    set_buzzer(4200, 1);
        rt_thread_mdelay(20);
        set_buzzer(0, 1);
        MainRGB_init();
        green_quickly();
    */
    // 所有程序注意, 如果初始化函数存在非 RT_EOK 的返回值单片机会直接复位
/*        if (MONITOR_INIT(1024, 1, THREAD_PRIO_MONITOR) != RT_EOK)
            Robot_Reset_Gimbal();

        if (SensorRawProcess_Init() != RT_EOK)
            Robot_Reset_Gimbal();
        if (Atti_init() != RT_EOK)
            Robot_Reset_Gimbal();
        if (IMU_WaitForInit() != RT_EOK)
            Robot_Reset_Gimbal();

        if(Sensor_Init() != RT_EOK)
            Robot_Reset_Gimbal();
        if (RC_init() != RT_EOK) // 遥控器初始化
            Robot_Reset_Gimbal();

        if(StateDecide_Init() != RT_EOK)
            Robot_Reset_Gimbal();
        
        Motor_init();
*/
    //		while(1);

    //    return RT_EOK;
}
