#ifndef __FUNC_ALARM_H__
#define __FUNC_ALARM_H__
#include <rtthread.h>
#ifdef BSP_USING_RGB_LIGHT
#include "drv_rgblight.h"
#endif

/*设定报警周期，单位ms*/
#define ALARM_PERIOD 100

///////////////////////////*根据板子上是否有蜂鸣器来选择*///////////////////////////
#ifdef BSP_USING_BUZZER
#include "drv_buzzer.h"

/*控制报警时蜂鸣器开启或关闭的开关引脚*/
#define SWITCH_BUZZER_PIN 67

/*程序初始化失败和复位的提醒；串口会一直重复打印;蜂鸣器会一直滴答滴答响*/
#define WARN_PROGRAM_EXCEPTION               \
    do                                       \
    {                                        \
        rt_kprintf("Program exception!!\n"); \
        set_buzzer(2000);                    \
        rt_thread_mdelay(50);                \
        set_buzzer(0);                       \
        rt_thread_mdelay(50);                \
    } while (0);

//监视器自定义报警风格
#define ALARM_SET         \
    set_buzzer(1800);     \
    rt_thread_mdelay(50); \
    set_buzzer(0);        \
    rt_thread_mdelay(400); //默认1500Hz
#define ALARM_RESET       \
    set_buzzer(900);      \
    rt_thread_mdelay(50); \
    set_buzzer(0);        \
    rt_thread_mdelay(800); //以低频率声音来作为间隔
                           //#define ALARM_RESET  rt_thread_mdelay(2000);//以静音来作为间隔
#else
#define WARN_PROGRAM_EXCEPTION
#define ALARM_SET
#define ALARM_RESET
#endif
/////////////////////////////////////////////////////////////////////////////////

/**
 * @brief    报警初始化
 * @param    无
 * @return   初始化成功or失败
 * @author   Lvfp
 */
rt_err_t Alarm_Init(void);

#endif
