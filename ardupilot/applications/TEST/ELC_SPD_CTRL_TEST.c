#include "ELC_SPD_CTRL_TEST.h"
#include "KEY_TEST.h"
#include <rtdevice.h>
#include "drv_gpio.h"
#include "drv_PWM_motor.h"
#include "roboselect.h"
#include "func_motor.h"
#include "INS_FLOW.h"

static int finish_flag = 1;

static void PWM_MIN(void *parameter)
{
	static int first_flag = 1;
	if(first_flag)
	{
		first_flag = 0;

		MX_TIM_DUTY(TIM1,COPTER_MOTOR_1,MIN_DUTY);
		MX_TIM_DUTY(TIM1,COPTER_MOTOR_2,MIN_DUTY);
		MX_TIM_DUTY(TIM1,COPTER_MOTOR_3,MIN_DUTY);
		MX_TIM_DUTY(TIM1,COPTER_MOTOR_4,MIN_DUTY);		
	}	
	else
	{
		finish_flag = 0;
	}
}

static void PWM_MAX(void *parameter)
{
	static int first_flag = 1;
	if(first_flag)
	{
		first_flag = 0;
		MX_TIM_DUTY(TIM1,COPTER_MOTOR_1,MAX_DUTY);
		MX_TIM_DUTY(TIM1,COPTER_MOTOR_2,MAX_DUTY);
		MX_TIM_DUTY(TIM1,COPTER_MOTOR_3,MAX_DUTY);
		MX_TIM_DUTY(TIM1,COPTER_MOTOR_4,MAX_DUTY);		
	}

}

static void key_Init(void)
{
     /* 按键0引脚为输入模式 */
    rt_pin_mode(KEY1_PIN_NUM, PIN_MODE_INPUT_PULLUP);
    /* 绑定中断，下降沿模式，回调函数名为PWM_MIN */
    rt_pin_attach_irq(KEY1_PIN_NUM, PIN_IRQ_MODE_FALLING, PWM_MIN, RT_NULL);
    /* 使能中断 */
    rt_pin_irq_enable(KEY1_PIN_NUM, PIN_IRQ_ENABLE);

    /* 按键1引脚为输入模式 */
    rt_pin_mode(KEY2_PIN_NUM, PIN_MODE_INPUT_PULLUP);
    /* 绑定中断，下降沿模式，回调函数名为PWM_MAX */
    rt_pin_attach_irq(KEY2_PIN_NUM, PIN_IRQ_MODE_FALLING, PWM_MAX, RT_NULL);
    /* 使能中断 */
    rt_pin_irq_enable(KEY2_PIN_NUM, PIN_IRQ_ENABLE);
}

void elc_spd_ctrl_Init(void)
{
    MX_TIM1_PWM_Init();
    key_Init();
		while(finish_flag);
}



