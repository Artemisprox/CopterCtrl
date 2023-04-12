#include "func_ESC_cali.h"
#include <rtthread.h>
#include "drv_PWM_motor.h"
#include <rtdevice.h>
#include "drv_gpio.h"
#include "func_motor.h"
#include "func_state.h"
#include "drv_dataserve.h"

static rt_uint8_t status_ID;
static status copter_status;
/* 引脚编号，通过查看设备驱动文件drv_gpio.c确定 */
#define KEY1_PIN_NUM GET_PIN(B, 7)
#define KEY2_PIN_NUM GET_PIN(B, 6)
#define BEEP_PIN_NUM GET_PIN(B, 9)

static void ESC_set_high(void)
{
    MX_TIM_DUTY(TIM1,TIM_CHANNEL_ALL,MAX_DUTY);
}

static void ESC_set_low(void)
{
    MX_TIM_DUTY(TIM1,TIM_CHANNEL_ALL,MIN_DUTY);
}


static void RGB_R_TEST(void *args)
{
//    static rt_uint32_t a = 1;
	rt_thread_mdelay(10);
	uint8_t p;
	p = rt_pin_read(KEY1_PIN_NUM);
	
    if (!p)
    {
		//pulse_R = (++a % 2) > 0 ? 0 : 1000;
    }
}

static void RGB_B_TEST(void *args)
{
		static int p;
    static rt_uint32_t a = 1;
		rt_thread_mdelay(10);
	p = rt_pin_read(KEY2_PIN_NUM);
	if (!p)
    {
			//更新无人机实时状态
      status *p_2 =  Package_Pionter_Single(status_ID,status);
	    copter_status = *p_2 ;
	    Package_Write_Pionter_End(status_ID,status);
			
			//仅有在电池未接入的情况下允许进行电调校准
      //第一次按下置PWM于最大占空比，蜂鸣器响起。第二次按下PWM置于最低占空比，蜂鸣器关闭。
        if((!copter_status.power_ready)&&(a == 1))
        {
           ESC_set_high();
					 a = 2;
					 rt_pin_write(BEEP_PIN_NUM, PIN_HIGH);
				}else if(copter_status.power_ready && (a == 2))
        {
           ESC_set_low();
					 a = 1;
					 rt_pin_write(BEEP_PIN_NUM, PIN_LOW);
        }           
    }
}

void ESC_cali_Init(void)
{
    /* 蜂鸣器引脚为输出模式 */
    rt_pin_mode(BEEP_PIN_NUM, PIN_MODE_OUTPUT);
    /* 默认低电平 */
    rt_pin_write(BEEP_PIN_NUM, PIN_LOW);

    /* 按键0引脚为输入模式 */
    rt_pin_mode(KEY1_PIN_NUM, PIN_MODE_INPUT_PULLUP);
    /* 绑定中断，下降沿模式，回调函数名为beep_on */
    rt_pin_attach_irq(KEY1_PIN_NUM, PIN_IRQ_MODE_FALLING, RGB_R_TEST, RT_NULL);
    /* 使能中断 */
    rt_pin_irq_enable(KEY1_PIN_NUM, PIN_IRQ_ENABLE);

    /* 按键1引脚为输入模式 */
    rt_pin_mode(KEY2_PIN_NUM, PIN_MODE_INPUT_PULLUP);
    /* 绑定中断，下降沿模式，回调函数名为beep_off */
    rt_pin_attach_irq(KEY2_PIN_NUM, PIN_IRQ_MODE_FALLING, RGB_B_TEST, RT_NULL);
    /* 使能中断 */
    rt_pin_irq_enable(KEY2_PIN_NUM, PIN_IRQ_ENABLE);

    status_ID = Package_Find_Num("status");
}
