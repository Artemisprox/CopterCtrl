#ifndef __APP_TEST_H__
#define __APP_TEST_H__

#define TEST_LIMIT_CAP_VOLTAGE (0) // 写1启动超级电容满电停充，用于限制超级电容电量
#define TEST_LIMIT_CAP_VOLTAGE_VALUE (13.0f)

#define TEST_MEASURE (0)// 写1打开相关参数测定模式

#define WDT_AlwaysFeed (0)

#if TEST_MEASURE

// 0--DAC设定值/实际值 同时可以采集数据进行DAC输出电压-升降压模块实际限流参数测定
#define TEST_SELECT 0 // 在这里选择测试项目

#define USE_MEASURED_DATA 0 //写1--使用标定后的ADC数据 0--不使用标定数据  --标定adc时候应选择0

extern void Test_Measure_Init(void);

#endif

#endif
