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
#include <rtdevice.h>
#include <board.h>

#include "func_HW_Pin_Set.h"

#include "app_oled.h"
#include "app_charge.h"
#include "drv_test.h"

int main(void)
{
    adc_dev_init();//初始化adc采样
    rt_thread_delay(50);
    can1_init();
#if (TEST_MEASURE)
    Test_Measure_Init();
#else
    OLED_Show_App_Init();
    CAP_Ctrl_App_Init();
#endif
}
