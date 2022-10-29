#include "drv_CustomUI_AuxAiming.h"
#include <math.h>
#include "drv_queue.h"
#include "mod_RefSystem.h"
#include "drv_IMU.h"
#include "app_GetGim.h"
#include "app_GetRef.h"
#include "drv_GimMotor.h"


#define MOTION_MODE

#define PI 3.14159265f
#define g 979.85f			//重力加速度
#ifdef CORE_USING_HERO
#define mass_1 41.0f //大弹丸质�?
#else
#define mass_1 3.2f //小弹丸质�?	
#endif

#define BASEMENT_HEIGHT		42.0f
#define SENTRY_HEIGHT 130.0f
#define OUTPOST_HEIGHT 120.0f
#ifdef 	CORE_USING_HERO
#define AMMOR_HEIGHT 27.0f			//装甲板高�?
#else
#define AMMOR_HEIGHT 20.0f //装甲板高�?
#endif

#define h_max 540.0f		//UI最大的vision_y
#define angle_max 35.574f //图传最大的张�??

// #define speed0  1500.0f						//子弹初速度(cm/s)


#define offset  -36.0f

#define FUNC(a, b, c, y, x) \
	(a * x + b * log(1 - c * x) - y)

#define DIFF_FUNC(a, b, c, y, x) \
	(a - (b * c) / (1 - c * x))

/***
* @brief 	对�?�判系统弹速和云台设定的弹速取平均
* @param
* @retval	滤波后弹�?,单位：cm/s
* @author dxy
***/
rt_uint16_t Get_Bullet_Speed(void)
{
    static SqQueue bullet_speed_buff[3];      //三�?�弹速�?�应三个�?�?队列
    static rt_uint8_t bullet_speed_mode = 0;
    static rt_uint8_t is_init = 0;
    static rt_uint16_t bullet_speed_ref_last = 0;        ////上一次�?�判系统弹速返回�?
#ifdef CORE_USING_HERO
	static rt_uint16_t bullet_speed_limit[3] = {1000, 1600, 1600};
#else
	static rt_uint16_t bullet_speed_limit[3] = {1500, 1800, 3000};
#endif
    rt_uint16_t len;
    rt_uint16_t bullet_speed_filter;
	rt_uint16_t bullet_speed_ref = 0; //裁判系统弹速返回�?
	if (!is_init)
    {
        InitSqQueue(&bullet_speed_buff[0]);
        InitSqQueue(&bullet_speed_buff[1]);
		InitSqQueue(&bullet_speed_buff[2]);
		is_init = 1;
    }


	bullet_speed_mode = Ref_Bullet_Speed_Mode();
	bullet_speed_ref = Ref_Bullet_Speed();	   //裁判系统弹速返回�?
	if(bullet_speed_ref != 0)
	{

		if (bullet_speed_ref_last != bullet_speed_ref) //裁判系统弹速更�?
		{
			bullet_speed_ref_last = bullet_speed_ref;
			if (EnSqQueue(&bullet_speed_buff[bullet_speed_mode], bullet_speed_ref))
			{
				len = SqQueueLength(bullet_speed_buff[bullet_speed_mode]);
				bullet_speed_filter = SqQueueSum(bullet_speed_buff[bullet_speed_mode]) / len;
				if (bullet_speed_filter == 0)
				{
					return bullet_speed_limit[bullet_speed_mode];
				}
				return bullet_speed_filter;
			}
			else
			{
				DeSqQueue(&bullet_speed_buff[bullet_speed_mode], NULL);
				EnSqQueue(&bullet_speed_buff[bullet_speed_mode], bullet_speed_ref);
				len = SqQueueLength(bullet_speed_buff[bullet_speed_mode]);
				return SqQueueSum(bullet_speed_buff[bullet_speed_mode]) / len;
			}
		}
		else //速度�?更新
		{
			len = SqQueueLength(bullet_speed_buff[bullet_speed_mode]);
			bullet_speed_filter = SqQueueSum(bullet_speed_buff[bullet_speed_mode]) / len;
			if (bullet_speed_filter == 0)
			{
				return bullet_speed_limit[bullet_speed_mode];
			}
			return bullet_speed_filter;
		}
	}
	else
	{
		return bullet_speed_limit[bullet_speed_mode];
	}
}
