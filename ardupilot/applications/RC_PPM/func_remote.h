#include <rtthread.h>

#define f_max            30
#define pitch_max_degree 10
#define roll_max_degree  10
#define yaw_max_spe      30

#define rocker_min       36  //摇杆最小占空比
#define rocker_max       72  //摇杆最大占空比

#define stabilization    1   //自稳
#define height           2   //定高
#define position          3  //定点

#define ready                 1   //等待起飞
#define armed                 0   //已经起飞
#define emergency_stop        2   //紧急暂停

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

