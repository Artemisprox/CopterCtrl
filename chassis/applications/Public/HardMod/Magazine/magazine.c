#include "Magazine.h"

static struct rt_device_pwm *servo_dev;   /* 弹仓PWM设备名称 */


/**
 * @brief  弹仓舵机角度设定
 * @param  percent 百分比 0-100，对应角度0-180°
 * @return RT_EOK or RT_ERROR(成功或失败)
 */
static void Maga_Servo_Set(rt_uint32_t percent)
{
	static rt_uint32_t last_percent = 0;
	
	/*只有设定值改变时，重新设置脉宽*/
	if(last_percent != percent)
	{
		/*输入限幅*/
		if(percent > 100) 
			percent = 100;
	
		/*单位换算*/
		percent = percent*40000+1000000;
	
		/*设置周期和脉冲宽度*/
		rt_pwm_set(servo_dev, PWM_DEV_CHANNEL, 40000000,percent);
	
		last_percent = percent;
	}
}


/**
 * @brief  舵机（弹仓门开关）初始化
 * @note   设置频率为100hz，则实际输出200hz。(RT-THread高级定时器BUG)
 * @param  percent 占空比（0-1000对于0%-%100）
 * @return RT_EOK or RT_ERROR(成功或失败)
 */
int Maga_Servo_Init(void)
{
    rt_err_t res = RT_EOK;

	servo_dev = (struct rt_device_pwm *)rt_device_find(PWM_DEV_NAME);

	if(!servo_dev)
		{return RT_ERROR;}

	/*默认关弹仓盖*/
	Maga_Servo_Set(PER_CLOSE);

	/* 使能设备 */
	res = rt_pwm_enable(servo_dev, PWM_DEV_CHANNEL);
	if ( res != RT_EOK)
		return res;

	return RT_EOK;
}
INIT_APP_EXPORT(Maga_Servo_Init);


/**
 * @brief  打开弹仓
 * @param  None
 * @return None
 */
void Maga_Open(void)
{
	Maga_Servo_Set(PER_OPEN);
}


/**
 * @brief  关闭弹仓
 * @param  None
 * @return None
 */
void Maga_Close(void)
{
	Maga_Servo_Set(PER_CLOSE);
}
