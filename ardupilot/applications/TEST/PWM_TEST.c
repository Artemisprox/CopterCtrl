#include "PWM_TEST.h"

#include <rtdevice.h>

#define PWM_DEV_NAME "pwm1" /* PWM设备名称 */

#define PWM_1 1 /* PWM通道 */
#define PWM_2 2 /* PWM通道 */
#define PWM_3 3 /* PWM通道 */
#define PWM_4 4 /* PWM通道 */

static rt_uint32_t period_PWM_1, pulse_PWM_1, period_PWM_2, pulse_PWM_2, period_PWM_3, pulse_PWM_3, period_PWM_4, pulse_PWM_4;

static struct rt_device_pwm *pwm_dev; /* PWM设备句柄 */

rt_err_t PWM_Init(void)
{
    rt_err_t res = RT_EOK;

    period_PWM_1 = 200000; /* 周期为0.2ms，单位为纳秒ns */
    pulse_PWM_1 = 100000;  /* PWM脉冲宽度值，单位为纳秒ns */
    period_PWM_2 = 200000; /* 周期为0.2ms，单位为纳秒ns */
    pulse_PWM_2 = 100000;  /* PWM脉冲宽度值，单位为纳秒ns */
    period_PWM_3 = 200000; /* 周期为0.2ms，单位为纳秒ns */
    pulse_PWM_3 = 100000;  /* PWM脉冲宽度值，单位为纳秒ns */
    period_PWM_4 = 200000; /* 周期为0.2ms，单位为纳秒ns */
    pulse_PWM_4 = 100000;  /* PWM脉冲宽度值，单位为纳秒ns */

    /* 查找设备 */
    pwm_dev = (struct rt_device_pwm *)rt_device_find(PWM_DEV_NAME);
    if (pwm_dev == RT_NULL)
    {
        rt_kprintf("pwm sample run failed! can't find %s device!\n", PWM_DEV_NAME);
        return RT_ERROR;
    }

    /* 设置PWM周期和脉冲宽度默认值 */
    res = rt_pwm_set(pwm_dev, PWM_1, period_PWM_1, pulse_PWM_1);
    RT_ASSERT(res == RT_EOK);
    res = rt_pwm_set(pwm_dev, PWM_2, period_PWM_2, pulse_PWM_2);
    RT_ASSERT(res == RT_EOK);
    res = rt_pwm_set(pwm_dev, PWM_3, period_PWM_3, pulse_PWM_3);
    RT_ASSERT(res == RT_EOK);
    res = rt_pwm_set(pwm_dev, PWM_4, period_PWM_4, pulse_PWM_4);
    RT_ASSERT(res == RT_EOK);
    /* 使能设备 */
    res = rt_pwm_enable(pwm_dev, PWM_1);
    RT_ASSERT(res == RT_EOK);
    res = rt_pwm_enable(pwm_dev, PWM_2);
    RT_ASSERT(res == RT_EOK);
    res = rt_pwm_enable(pwm_dev, PWM_3);
    RT_ASSERT(res == RT_EOK);
    res = rt_pwm_enable(pwm_dev, PWM_4);
    RT_ASSERT(res == RT_EOK);

    return RT_EOK;
}
