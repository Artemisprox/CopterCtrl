#include "RGB_TEST.h"

#include <rtdevice.h>

#define PWM_DEV_NAME        "pwm3"  /* PWM设备名称 */

#define RGB_G     1       /* PWM通道 */
#define RGB_R     2       /* PWM通道 */
#define RGB_B     3       /* PWM通道 */

static rt_uint32_t period_R, pulse_R,period_G,pulse_G,period_B,pulse_B;

static struct rt_device_pwm *pwm_dev;      /* PWM设备句柄 */

static void TEST_RGB_Ctrl(void *parameter)
{
    rt_int32_t count=0;
    while(1)
    {
        switch (count%3)
        {
        case 0:
            period_R = 200000;    /* 周期为0.2ms，单位为纳秒ns */
            pulse_R = 1000;          /* PWM脉冲宽度值，单位为纳秒ns */
//            period_G = 200000;    /* 周期为0.2ms，单位为纳秒ns */
//            pulse_G = 0;          /* PWM脉冲宽度值，单位为纳秒ns */
            period_B = 200000;    /* 周期为0.2ms，单位为纳秒ns */
            pulse_B = 0;          /* PWM脉冲宽度值，单位为纳秒ns */
//            rt_pwm_set(pwm_dev, RGB_G, period_G, pulse_G);
            rt_pwm_set(pwm_dev, RGB_R, period_R, pulse_R);
            rt_pwm_set(pwm_dev, RGB_B, period_B, pulse_B);
            break;

        case 1:
            period_R = 200000;    /* 周期为0.2ms，单位为纳秒ns */
            pulse_R = 0;          /* PWM脉冲宽度值，单位为纳秒ns */
//            period_G = 200000;    /* 周期为0.2ms，单位为纳秒ns */
//            pulse_G = 5000;          /* PWM脉冲宽度值，单位为纳秒ns */
            period_B = 200000;    /* 周期为0.2ms，单位为纳秒ns */
            pulse_B = 0;          /* PWM脉冲宽度值，单位为纳秒ns */
//            rt_pwm_set(pwm_dev, RGB_G, period_G, pulse_G);
            rt_pwm_set(pwm_dev, RGB_R, period_R, pulse_R);
            rt_pwm_set(pwm_dev, RGB_B, period_B, pulse_B);
            break;

        case 2:
            period_R = 200000;    /* 周期为0.2ms，单位为纳秒ns */
            pulse_R = 0;          /* PWM脉冲宽度值，单位为纳秒ns */
//            period_G = 200000;    /* 周期为0.2ms，单位为纳秒ns */
//            pulse_G = 0;          /* PWM脉冲宽度值，单位为纳秒ns */
            period_B = 200000;    /* 周期为0.2ms，单位为纳秒ns */
            pulse_B = 1000;          /* PWM脉冲宽度值，单位为纳秒ns */
//            rt_pwm_set(pwm_dev, RGB_G, period_G, pulse_G);
            rt_pwm_set(pwm_dev, RGB_R, period_R, pulse_R);
            rt_pwm_set(pwm_dev, RGB_B, period_B, pulse_B);
            break;
        
        default:
            break;
        }
        ++count;
        rt_thread_mdelay(500);
    }
}

rt_err_t RGB_Init(void)
{
    rt_thread_t thread;
    rt_err_t res = RT_EOK;

    period_R = 200000;    /* 周期为0.2ms，单位为纳秒ns */
    pulse_R = 0;          /* PWM脉冲宽度值，单位为纳秒ns */
    period_G = 200000;    /* 周期为0.2ms，单位为纳秒ns */
    pulse_G = 0;          /* PWM脉冲宽度值，单位为纳秒ns */
    period_B = 200000;    /* 周期为0.2ms，单位为纳秒ns */
    pulse_B = 0;          /* PWM脉冲宽度值，单位为纳秒ns */

    /* 查找设备 */
    pwm_dev = (struct rt_device_pwm *)rt_device_find(PWM_DEV_NAME);
    if (pwm_dev == RT_NULL)
    {
        rt_kprintf("pwm sample run failed! can't find %s device!\n", PWM_DEV_NAME);
        return RT_ERROR;
    }

    /* 设置PWM周期和脉冲宽度默认值 */
    res = rt_pwm_set(pwm_dev, RGB_G, period_G, pulse_G);
    RT_ASSERT(res == RT_EOK);
    res = rt_pwm_set(pwm_dev, RGB_R, period_R, pulse_R);
    RT_ASSERT(res == RT_EOK);
    res = rt_pwm_set(pwm_dev, RGB_B, period_B, pulse_B);
    RT_ASSERT(res == RT_EOK);
    /* 使能设备 */
    res = rt_pwm_enable(pwm_dev, RGB_G);
    RT_ASSERT(res == RT_EOK);
    res = rt_pwm_enable(pwm_dev, RGB_R);
    RT_ASSERT(res == RT_EOK);
    res = rt_pwm_enable(pwm_dev, RGB_B);
    RT_ASSERT(res == RT_EOK);

    thread = rt_thread_create("TEST_RGB_Ctrl", TEST_RGB_Ctrl, RT_NULL, 2048, THREAD_PRIO_TEST_RGB, 1);
    if (thread != RT_NULL)
        rt_thread_startup(thread);

    return RT_EOK;
}
