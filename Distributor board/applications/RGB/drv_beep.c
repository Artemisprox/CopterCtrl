#include <rtthread.h>
#include "drv_beep.h"
#include "drv_gpio.h"

void BEEP_init(void)
{
		/* 蜂鸣器引脚为输出模式 */
    rt_pin_mode(BEEP_PIN_NUM, PIN_MODE_OUTPUT);
    /* 默认低电平 */
    rt_pin_write(BEEP_PIN_NUM, PIN_LOW);
}

void beep_set_high(void)
{
	rt_pin_write(BEEP_PIN_NUM, PIN_HIGH);
}

void beep_set_low(void)
{
	rt_pin_write(BEEP_PIN_NUM, PIN_LOW);
}