/************************************** Copyright ****************************** 
  *                 (C) Copyright 2020,China, HITwh.
  *                            All Rights Reserved
  *
  *                     HITwh Excellent Robot Organization
  *                     https://github.com/HERO-ECG
  *                     https://gitee.com/HIT-718LC
  *
  * FileName   : drv_rgblight.c
  * Version    : v1.0
  * Author     : mqy
  * Date       : 2020-02-14
  * Description:
********************************************************************************/

#include "drv_rgblight.h"

//RGB颜色控制PWM设备句柄与通道号
struct rt_device_pwm* red_light 		= RT_NULL;

struct rt_device_pwm* green_light 	= RT_NULL;

struct rt_device_pwm* blue_light 	= RT_NULL;

/**
* @brief：该函数初始化核心板上的RGB灯
* @param [in] 无
* @return：	true：初始化成功
			false：初始化失败
* @author：	mqy
*/
int RGB_init(void)
{
    //尝试查找设备，查找失败时返回
    red_light 	= (struct rt_device_pwm *)rt_device_find("pwm5");
    if(red_light == RT_NULL)
    {
        return 0;
    }

    green_light	= (struct rt_device_pwm *)rt_device_find("pwm5");
    if(green_light == RT_NULL)
    {
        return 0;
    }

    blue_light	= (struct rt_device_pwm *)rt_device_find("pwm5");
    if(blue_light == RT_NULL)
    {
        return 0;
    }
    rt_pwm_set(red_light,RED_PWMCHANNEL,2550000,2550000);
    rt_pwm_set(green_light,GREEN_PWMCHANNEL,2550000,2550000);
    rt_pwm_set(blue_light,BLUE_PWMCHANNEL,2550000,2550000);

    rt_pwm_enable(red_light,RED_PWMCHANNEL);
    rt_pwm_enable(green_light,GREEN_PWMCHANNEL);
    rt_pwm_enable(blue_light,BLUE_PWMCHANNEL);
	
	return 1;
}
INIT_APP_EXPORT(RGB_init);
/**
* @brief：该函数设置灯的亮度
* @param[in] red：红色亮度
			green：绿色亮度
			blue：蓝色亮度
* @return：	无
* @author:	mqy
*/
void set_RGB(rt_uint8_t red,rt_uint8_t green,rt_uint8_t blue)
{
    static rt_uint8_t red_last, green_last, blue_last;
    if((red!=red_last)||(green!=green_last)||(blue!=blue_last))
    {
        rt_pwm_set(red_light, RED_PWMCHANNEL, 2550000, red * 10000);
        rt_pwm_set(green_light, GREEN_PWMCHANNEL, 2550000, green * 10000);
        rt_pwm_set(blue_light, BLUE_PWMCHANNEL, 2550000, blue * 10000);
    }
    red_last = red;
    green_last = green;
    blue_last = blue;
}


