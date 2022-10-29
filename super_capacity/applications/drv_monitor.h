#ifndef __DRV_MONITOR_H__
#define __DRV_MONITOR_H__

#include "drv_test.h"

#include "rtconfig.h"
#ifdef BSP_USING_WDT
#define USE_WDG (1)
#else
#define USE_WDG (0)
#endif
#if (TEST_MEASURE || TEST_LIMIT_CAP_VOLTAGE)
// 测试程序不启动看门狗
#else
    #if USE_WDG
        #define _WDG_ENABLE_
    #endif
#endif

// 看门狗喂狗 在modcharge中喂狗
void WDT_Feed(void);

// 看门狗初始化 在appcharge中启动充电控制之后初始化
int WDT_Init(void);

#endif
