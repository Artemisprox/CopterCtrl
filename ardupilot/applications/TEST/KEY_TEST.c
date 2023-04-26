#include "KEY_TEST.h"

#include <rtdevice.h>
#include "drv_gpio.h"
#include "RGB_TEST.h"

/* 引脚编号，通过查看设备驱动文件drv_gpio.c确定 */
#define KEY1_PIN_NUM GET_PIN(B, 9)
#define KEY2_PIN_NUM GET_PIN(C, 10)
#define BEEP_PIN_NUM GET_PIN(C, 13)

#define PWM_DEV_NAME "pwm3" /* PWM设备名称 */

#define RGB_G 1 /* PWM通道 */
#define RGB_R 2 /* PWM通道 */
#define RGB_B 3 /* PWM通道 */

typedef enum
{
    BEEP_KEY,
    RGB_G_KEY,
    RGB_R_KEY,
    RGB_B_KEY
} Mode_e;

static struct rt_device_pwm *pwm_dev; /* PWM设备句柄 */

static rt_uint32_t period_R, pulse_R, period_B, pulse_B, period_G, pulse_G;

static Mode_e Mode_Key2 = BEEP_KEY;

static void Mode_Change(void *args)
{
    static Mode_e a = RGB_G_KEY;
    rt_uint8_t key1;
    rt_thread_mdelay(10);
    key1 = rt_pin_read(KEY1_PIN_NUM);
    if (!key1)
    {
        Mode_Key2 = a;
        a++;
        if (a > RGB_B_KEY)
        {
            a = BEEP_KEY;
        }
    }
}

static void RGB_B_TEST(void *args)
{
    static rt_uint32_t count_BEEP = 1;
    static rt_uint32_t count_G = 1;
    static rt_uint32_t count_R = 1;
    static rt_uint32_t count_B = 1;
    rt_uint8_t key2;
    rt_int8_t Beep_Status;
    rt_thread_mdelay(10);
    key2 = rt_pin_read(KEY2_PIN_NUM);
    if (!key2)
    {
        switch (Mode_Key2)
        {
        case BEEP_KEY:
            Beep_Status = (++count_BEEP % 2) > 0 ? PIN_LOW : PIN_HIGH;
            rt_pin_write(BEEP_PIN_NUM, Beep_Status);
            break;
        case RGB_G_KEY:
            pulse_G = (++count_G % 2) > 0 ? 0 : 1000;
            rt_pwm_set(pwm_dev, RGB_G, period_G, pulse_G);
            break;
        case RGB_R_KEY:
            pulse_R = (++count_R % 2) > 0 ? 0 : 1000;
            rt_pwm_set(pwm_dev, RGB_R, period_R, pulse_R);
            break;
        case RGB_B_KEY:
            pulse_B = (++count_B % 2) > 0 ? 0 : 1000;
            rt_pwm_set(pwm_dev, RGB_B, period_B, pulse_B);
            break;
        default:
            break;
        }
    }
}

void KEY_Init(void)
{
    period_G = 200000;
    period_R = 200000;
    period_B = 200000;

    pwm_dev = (struct rt_device_pwm *)rt_device_find(PWM_DEV_NAME);

    rt_pwm_set(pwm_dev, RGB_G, period_G, pulse_G);
    rt_pwm_set(pwm_dev, RGB_R, period_R, pulse_R);
    rt_pwm_set(pwm_dev, RGB_B, period_B, pulse_B);
    /* 使能设备 */
    rt_pwm_enable(pwm_dev, RGB_G);
    rt_pwm_enable(pwm_dev, RGB_R);
    rt_pwm_enable(pwm_dev, RGB_B);

    /* 蜂鸣器引脚为输出模式 */
    rt_pin_mode(BEEP_PIN_NUM, PIN_MODE_OUTPUT);
    /* 默认低电平 */
    rt_pin_write(BEEP_PIN_NUM, PIN_LOW);

    /* 按键0引脚为输入模式 */
    rt_pin_mode(KEY1_PIN_NUM, PIN_MODE_INPUT_PULLUP);
    /* 绑定中断，下降沿模式，回调函数名为beep_on */
    rt_pin_attach_irq(KEY1_PIN_NUM, PIN_IRQ_MODE_FALLING, Mode_Change, RT_NULL);
    /* 使能中断 */
    rt_pin_irq_enable(KEY1_PIN_NUM, PIN_IRQ_ENABLE);

    /* 按键1引脚为输入模式 */
    rt_pin_mode(KEY2_PIN_NUM, PIN_MODE_INPUT_PULLUP);
    /* 绑定中断，下降沿模式，回调函数名为beep_off */
    rt_pin_attach_irq(KEY2_PIN_NUM, PIN_IRQ_MODE_FALLING, RGB_B_TEST, RT_NULL);
    /* 使能中断 */
    rt_pin_irq_enable(KEY2_PIN_NUM, PIN_IRQ_ENABLE);
}
