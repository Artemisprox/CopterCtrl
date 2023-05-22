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
#include "drv_RGB.h"
#include "func_RGBctrl.h"
#include "drv_beep.h"

#define TEST 0

#if (TEST)
#include "EnergyOrgan_Test.h"
#else

#endif
extern void cpu_usage_init(void);
int main(void)
{
    cpu_usage_init();
#if (TEST)
    EnergyOrgan_Test_Init();
#else
    RGB_Init();
    RGB_init_set();
		BEEP_init();
#endif

    return 0;
}
