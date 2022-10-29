#ifndef __HCANID_DATA_H__
#define __HCANID_DATA_H__
#include <rtthread.h>
#include <rtdevice.h>
#include "HChassis_Data.h"
///////////////////////////////////////CAN ID///////////////////////////////////////
/* 底盘CAN1设备ID */
typedef enum
{
	//底盘四轮电机反馈 ID
	RIGHT_FRONT = RIGHT_FRONT_MOTOR_ID, 
	LEFT_FRONT  = LEFT_FRONT_MOTOR_ID,
	LEFT_BACK   = LEFT_BACK_MOTOR_ID,
	RIGHT_BACK  = RIGHT_BACK_MOTOR_ID,

	//底盘IMU回传报文 ID
	CHASSIS_IMU_DATA1_RX = 0x403, 
	CHASSIS_IMU_DATA2_RX = 0x404, 

} drv_can1ID_e;


/* 底盘CAN2设备ID */
typedef enum
{
	//云台电机反馈 ID
	YAW_ID 	 = YAW_MOTOR_ID,
	PITCH_ID = PITCH_MOTOR_ID,

	//超级电容通信报文 ID
	SCPR_RX = 0x096, 	//从超级电容端接收
	SCPR_TX = 0x095, 	//向超级电容端发送

	//云台通信报文 ID
	GIMBAL_SHOOT_TX	 	= 0x105, 	//裁判系统枪管信息
	GIMBAL_DATA_TX 		= 0x106, 	//向云台发送各类数据
	GIMBAL_CCTRL_RX 	= 0x100, 	//底盘控制相关
	GIMBAL_DATA_RX 		= 0x401,	//从云台接收的各类数据
	GIMBAL_IMU_DATA_RX	= 0x402,	//从云台接收的IMU数据
		
} drv_can2ID_e;

///////////////////////////////////////CAN 过滤表设置///////////////////////////////////////
/*当增加原过滤表没有的can接收ID时，需在下面宏内加上相对应的过滤表项和数量*/
/*从原drv_canthread.c中单独拉出来，便于修改以及尽量不对底层驱动做修改*/
#define MYCAN1_FILTER_ITEM	struct rt_can_filter_item items[MYCAN1_FILTER_ITEM_NUM] = \
							{ \
								RT_CAN_FILTER_ITEM_INIT(RIGHT_FRONT   		, 0, 0, 1, 0x7F8, RT_NULL, RT_NULL), /* std,match ID:0x200~0x207，hdr 为 - 1，设置默认过滤表 */\
								RT_CAN_FILTER_ITEM_INIT(CHASSIS_IMU_DATA1_RX, 0, 0, 1, 0x7F8, RT_NULL, RT_NULL), /* std,match ID:0x400~0x407，hdr 为 - 1，设置默认过滤表 */\
							}; \

#define MYCAN1_FILTER_ITEM_NUM 2	//can1硬件过滤表的数量								
	
#define MYCAN2_FILTER_ITEM  struct rt_can_filter_item items[MYCAN2_FILTER_ITEM_NUM] = \
							{ \
								RT_CAN_FILTER_ITEM_INIT(YAW_ID			, 0, 0, 1, 0x7F8, RT_NULL, RT_NULL), /* std,match ID:0x200~0x207，hdr 为 - 1，设置默认过滤表 */\
								RT_CAN_FILTER_ITEM_INIT(SCPR_RX			, 0, 0, 1, 0x7F8, RT_NULL, RT_NULL), /* std,match ID:0x090~0x097，hdr 为 - 1，设置默认过滤表 */\
								RT_CAN_FILTER_ITEM_INIT(GIMBAL_CCTRL_RX , 0, 0, 1, 0x7F8, RT_NULL, RT_NULL), /* std,match ID:0x100~0x107，hdr 为 - 1，设置默认过滤表 */\
								RT_CAN_FILTER_ITEM_INIT(GIMBAL_DATA_RX  , 0, 0, 1, 0x7F8, RT_NULL, RT_NULL), /* std,match ID:0x400~0x407，hdr 为 - 1，设置默认过滤表 */\
							}; \

#define MYCAN2_FILTER_ITEM_NUM 4	//can2硬件过滤表的数量
//////////////////////////////////////////////////////////////////////////////////////

							
#endif
