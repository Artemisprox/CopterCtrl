#include "drv_RGB.h"
#include "drv_HW_pwm.h"

RGB_set rgb;

/**
 * @brief 设置RGB的颜色（最大值为1000）
 *
 * @param RGB_R_Pulse R的亮度
 * @param RGB_G_Pulse G的亮度
 * @param RGB_B_Pulse B的亮度
 */
void RGB_Colour_Set(RGB_set rgb_ctrl , rt_int16_t RGB_R_Pulse, rt_int16_t RGB_G_Pulse, rt_int16_t RGB_B_Pulse)
{
	switch(rgb_ctrl)
	{
		case RGB_A:
			HW_PWM_Pulse_Set(TIM2 ,A_RGB_G_Channel, 1000 - RGB_G_Pulse);
			HW_PWM_Pulse_Set(TIM2 ,A_RGB_R_Channel, 1000 - RGB_R_Pulse);
			HW_PWM_Pulse_Set(TIM2 ,A_RGB_B_Channel, 1000 - RGB_B_Pulse);
			break;
		case RGB_B:
			HW_PWM_Pulse_Set(TIM2 ,B_RGB_R_Channel, 1000 - RGB_R_Pulse);
			HW_PWM_Pulse_Set(TIM1 ,B_RGB_G_Channel, 1000 - RGB_G_Pulse);
			HW_PWM_Pulse_Set(TIM1 ,B_RGB_B_Channel, 1000 - RGB_B_Pulse);
			break;
		case RGB_C:
			HW_PWM_Pulse_Set(TIM1 ,C_RGB_R_Channel, 1000 - RGB_R_Pulse);
			HW_PWM_Pulse_Set(TIM1 ,C_RGB_G_Channel, 1000 - RGB_G_Pulse);
			rt_pin_write(C_RGB_B_PIN, PIN_HIGH);
			break;
		case RGB_D:
			if(RGB_R_Pulse != 0)
				rt_pin_write(D_RGB_R_PIN,PIN_HIGH);
			if(RGB_G_Pulse != 0)
				rt_pin_write(D_RGB_G_PIN,PIN_HIGH);
			if(RGB_B_Pulse != 0)
				rt_pin_write(D_RGB_B_PIN,PIN_HIGH);
			break;
		case RGB_onboard:
			HW_PWM_Pulse_Set(TIM3 ,onboard_RGB_R_Channel, RGB_R_Pulse);
			HW_PWM_Pulse_Set(TIM3 ,onboard_RGB_G_Channel, RGB_G_Pulse);
			HW_PWM_Pulse_Set(TIM3 ,onboard_RGB_B_Channel, RGB_B_Pulse);
			break;
	}	
}

// 初始化
void RGB_Init(void)
{ // 初始PWM均为0
  HW_PWM_Init();
	
  rt_pin_mode(C_RGB_B_PIN, PIN_MODE_OUTPUT);
  rt_pin_write(C_RGB_B_PIN, PIN_LOW);
	rt_pin_mode(D_RGB_R_PIN, PIN_MODE_OUTPUT);
  rt_pin_write(D_RGB_R_PIN, PIN_LOW);
	rt_pin_mode(D_RGB_G_PIN, PIN_MODE_OUTPUT);
  rt_pin_write(D_RGB_G_PIN, PIN_LOW);
	rt_pin_mode(D_RGB_B_PIN, PIN_MODE_OUTPUT);
  rt_pin_write(D_RGB_B_PIN, PIN_LOW);
	
  RGB_Colour_Set(RGB_A, 0, 0, 0);
	RGB_Colour_Set(RGB_B, 0, 0, 0);
	RGB_Colour_Set(RGB_C, 0, 0, 0);
	RGB_Colour_Set(RGB_D, 0, 0, 0);
	
}
