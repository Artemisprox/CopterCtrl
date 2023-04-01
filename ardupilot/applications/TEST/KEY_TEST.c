#include "KEY_TEST.h"

#include <rtdevice.h>
#include "drv_gpio.h"
#include "RGB_TEST.h"

/* 引脚编号，通过查看设备驱动文件drv_gpio.c确定 */
#define KEY1_PIN_NUM GET_PIN(B, 7)
#define KEY2_PIN_NUM GET_PIN(B, 6)
#define BEEP_PIN_NUM GET_PIN(B, 9)

#define PWM_DEV_NAME "pwm3" /* PWM设备名称 */

#define RGB_R 2 /* PWM通道 */
#define RGB_B 3 /* PWM通道 */

static struct rt_device_pwm *pwm_dev; /* PWM设备句柄 */

static rt_uint32_t period_R, pulse_R, period_B, pulse_B;

static void RGB_R_TEST(void *args)
{
    static rt_uint32_t a = 1;
    static rt_tick_t tick, tick_old = 0;
    tick = rt_tick_get();
    if (tick - tick_old > 300)
    {
        pulse_R = (++a % 2) > 0 ? 0 : 1000;
        rt_pwm_set(pwm_dev, RGB_R, period_R, pulse_R);
        tick_old = tick;
    }
}

static void RGB_B_TEST(void *args)
{
    static rt_uint32_t a = 1;
    static rt_tick_t tick, tick_old = 0;
    rt_int8_t Beep_Status;

    tick = rt_tick_get();
    if (tick - tick_old > 300)
    {
        Beep_Status = (++a % 2) > 0 ? PIN_LOW : PIN_HIGH;
        rt_pin_write(BEEP_PIN_NUM, Beep_Status);
        tick_old = tick;
    }
}

void KEY_Init(void)
{
    period_R = 200000;
    period_B = 200000;

    pwm_dev = (struct rt_device_pwm *)rt_device_find(PWM_DEV_NAME);

    rt_pwm_set(pwm_dev, RGB_R, period_R, pulse_R);
    rt_pwm_set(pwm_dev, RGB_B, period_B, pulse_B);
    /* 使能设备 */
    rt_pwm_enable(pwm_dev, RGB_R);
    rt_pwm_enable(pwm_dev, RGB_B);

    /* 蜂鸣器引脚为输出模式 */
    rt_pin_mode(BEEP_PIN_NUM, PIN_MODE_OUTPUT);
    /* 默认低电平 */
    rt_pin_write(BEEP_PIN_NUM, PIN_LOW);

    /* 按键0引脚为输入模式 */
    rt_pin_mode(KEY1_PIN_NUM, PIN_MODE_INPUT_PULLUP);
    /* 绑定中断，下降沿模式，回调函数名为beep_on */
    rt_pin_attach_irq(KEY1_PIN_NUM, PIN_IRQ_MODE_RISING, RGB_R_TEST, RT_NULL);
    /* 使能中断 */
    rt_pin_irq_enable(KEY1_PIN_NUM, PIN_IRQ_ENABLE);

    /* 按键1引脚为输入模式 */
    rt_pin_mode(KEY2_PIN_NUM, PIN_MODE_INPUT_PULLUP);
    /* 绑定中断，下降沿模式，回调函数名为beep_off */
    rt_pin_attach_irq(KEY2_PIN_NUM, PIN_IRQ_MODE_RISING, RGB_B_TEST, RT_NULL);
    /* 使能中断 */
    rt_pin_irq_enable(KEY2_PIN_NUM, PIN_IRQ_ENABLE);
}
