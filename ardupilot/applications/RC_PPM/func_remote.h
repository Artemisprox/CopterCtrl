#include <rtthread.h>

#define HEIGHT_MAX_V     2.0f 	//最大上升速度
#define POS_X_MAX_V      2.0f	//最大X轴速度
#define POS_Y_MAX_V      2.0f 	//最大Y轴速度

#define F_MAX            28.06f //最大升力
#define PITCH_MAX_DEG    20 	//最大俯仰角
#define ROLL_MAX_DEG     20 	//最大翻滚角
#define YAW_MAX_SPE      60 	//最大偏航角速度

#define PITCH_MAX_SPE    200//最大俯仰角速度
#define ROLL_MAX_SPE		 200//最大翻滚角速度

#define rocker_min       1107 	//遥控器摇杆低位
#define rocker_max       1930  	//遥控器摇杆高位
#define rocker_middle    1520  	//遥控器摇杆中位
#define rocker_width     823.0f //遥控器信号占空比总宽度
#define rocker_inter     10		//遥控器信号


#define STABILIZATION    1   
#define HEIGHT           2  
#define POSITION         3  

#define READY_T                 1   
#define ARMED_T                 0   
#define EMERGENCY_STOP_T        2   

#define LPF_k                0.5f
#define PEAK_LPF_k						0.1f



typedef struct
{
	float throttle;
	float pitch;
	float roll;
	float yaw;

	uint8_t pitch_middle_flag;
	uint8_t roll_middle_flag;
	uint8_t yaw_middle_flag;
	uint8_t throttle_low_flag;
	uint8_t throttle_middle_flag;

	uint8_t switch_arm;
	uint8_t switch_mode;
	uint8_t switch_arm_change;
	uint8_t switch_mode_change;

	uint32_t fresh_time;
}remote_data;


extern int RC_init(void);
