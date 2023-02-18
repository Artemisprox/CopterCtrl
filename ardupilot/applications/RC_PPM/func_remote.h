#include <rtthread.h>

#define f_max            30//最大推力
#define pitch_max_degree 10//最大角度
#define roll_max_degree  10//最大角度
#define yaw_max_spe      30//最大旋转速度

#define rocker_min       36.0f  //最小占空比
#define rocker_max       72.0f  //最大占空比
#define rocker_inter     2.0f		//在相同位置下摇杆的信号误差

#define stabilization    1   //自稳模式
#define height           2   //定高模式
#define position         3   //定点模式

#define ready                 1   //准备
#define armed                 0   //起飞
#define emergency_stop        2   //紧急停止

typedef struct
{
	float f;
	float pitch_deg;
	float roll_deg;
	float yaw_spe;
}remote_data;

typedef struct 
{
	uint8_t mode;
	uint8_t arm;
	uint8_t emergency;
}switch_state;

extern remote_data copter_remote ;
extern switch_state copter_state ;
