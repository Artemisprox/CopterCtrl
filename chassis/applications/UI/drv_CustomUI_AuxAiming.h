#ifndef __DRV_CUSTOMUI_AUXAIMING_H
#define __DRV_CUSTOMUI_AUXAIMING_H

#include <rtthread.h>
#include <rtdevice.h>
#include <board.h>

#define SHAFT_HEIGHT	35.0f		//�??轴高�??

#define VERT_DIS_CAMERA_SHAFT 9.0f	 //图传与转轴的垂直距�??
#define HORI_DIS_CAMERA_SHAFT 15.36f //图传镜头与转轴的水平距�??
#define VERT_DIS_GUN_SHAFT 0.0f		 //�??口与�??轴的垂直距�??
#define HORI_DIS_GUN_SHAFT 16.0f	 //�??口与�??轴的水平距�??
#define CAMERA_ANGLE_ERROR 0.f	 //图传安�?��??�??夹�??1.5f
#define k_resist 0.2175f			 //空气阻力系数(暂定)
#define PITCH_OFFSET 0.f	 //IMU安�?��??�??



struct AuxAim_t
{
	rt_uint8_t mode;
	float yaw;
	float  pitch;
	float bullet_speed;
};

int16_t get_AuxAim_y(rt_int16_t g_pitch) ;
rt_uint16_t Get_Bullet_Speed(void);

#endif

