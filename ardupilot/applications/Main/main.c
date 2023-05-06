/*
 * Copyright (c) 2006-2018, RT-Thread Development Team
 *
 * SPDX-License-Identifier: Apache-2.0
 *
 * Change Logs:
 * Date           Author       Notes
 * 2018-11-06     SummerGift   first version
 */


//#include "drv_buzzer.h"
#include "drv_TF_mini.h"
#include <rtthread.h>
#include "drv_RC_PPM.h"
#include "func_motor.h"
#include "drv_RGB.h"
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
#include "func_ESC_Cali.h"
#define TEST 0

#if (TEST)
#define KEY_TEST 0
#define CAN_TEST 0
#define RGB_TEST 0
#define PWM_TEST 0
#define UART_TEST 0
#define IMU_TEST 0
#define ESC_CALI 1
#include "CAN_TEST.h"
#include "RGB_TEST.h"
#include "KEY_TEST.h"
#include "PWM_TEST.h"
#include "UART_TEST.h"
#include "IMU_TEST.h"
#include "drv_RGB.h"
#endif

#if (!defined CORE_USING_INFANTRY) && (!defined CORE_USING_HERO) && (!defined CORE_USING_COPTER)
#error "Please specify the robot type!"
#endif

int main(void)
{
#if (TEST)
#if (CAN_TEST)
    CAN_Init();
#endif

#if (RGB_TEST)
    RGB_Init();
#endif

#if (KEY_TEST)
    KEY_Init();
#endif

#if (PWM_TEST)
    PWM_Init();
#endif

#if (UART_TEST)
    UART_Init();
#endif

#if (IMU_TEST)
    IMU_Init();
#endif

#if (ESC_CALI)
	MX_TIM1_PWM_Init();
	MainRGB_init();
  ESC_cali_Init();
#endif
#else

	/* 蜂鸣器引脚为输出模式 */
    rt_pin_mode(BEEP_PIN_NUM, PIN_MODE_OUTPUT);
  /* 默认低电平 */
    rt_pin_write(BEEP_PIN_NUM, PIN_LOW);

    // 上电提示音
				rt_pin_write(BEEP_PIN_NUM, PIN_HIGH);
        rt_thread_mdelay(20);
				rt_pin_write(BEEP_PIN_NUM, PIN_LOW);
        MainRGB_init();
        //red_keepon();
				//red_keepon();
//所有程序注意, 如果初始化函数存在非 RT_EOK 的返回值单片机会直接复位
//        if (MONITOR_INIT(1024, 1, THREAD_PRIO_MONITOR) != RT_EOK)
//            Robot_Reset_Gimbal();

        if (SensorRawProcess_Init() != RT_EOK)
            Robot_Reset_Gimbal();
        if (Atti_init() != RT_EOK)
            Robot_Reset_Gimbal();
        if (IMU_WaitForInit() != RT_EOK)
            Robot_Reset_Gimbal();

        if( Sensor_Init() != RT_EOK)
            Robot_Reset_Gimbal();

        if(StateDecide_Init() != RT_EOK)
            Robot_Reset_Gimbal();
				
//					MX_TIM1_PWM_Init();
//					MX_TIM_DUTY(TIM1,TIM_CHANNEL_1,0.1f);
//					MX_TIM_DUTY(TIM1,TIM_CHANNEL_2,0.1f);
//					MX_TIM_DUTY(TIM1,TIM_CHANNEL_3,0.1f);
//					MX_TIM_DUTY(TIM1,TIM_CHANNEL_4,0.1f);
        Motor_init();
					
		//			RC_init();
    //		while(1);

    //    return RT_EOK;
#endif
}
