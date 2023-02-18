#include "drv_RC_PPM.h"
#include "func_remote.h"
#include "func_motor.h"

remote_data copter_remote ={0};
switch_state copter_state = {0};
switch_state copter_state_last = {0};
uint8_t armed_flag = 1;
uint8_t offboard_flag = 0;
uint8_t board_flag = 0; 
uint8_t first_flag = 1;

static void remote_data_process(void)
{
	
	copter_remote.f = (copter_rec_data.RC_throttle/2000.0f - rocker_min)/(rocker_max - rocker_min)*f_max;
	copter_remote.pitch_deg = (copter_rec_data.RC_pitch/2000.0f - rocker_min)/(rocker_max - rocker_min)*pitch_max_degree;
	copter_remote.roll_deg  = (copter_rec_data.RC_roll/2000.0f - rocker_min)/(rocker_max - rocker_min)*roll_max_degree;
	copter_remote.yaw_spe   = (copter_rec_data.RC_yaw/2000.0f - rocker_min)/(rocker_max - rocker_min)*yaw_max_spe;
	
	switch(copter_rec_data.RC_switch_left)
		{
			case 500:   copter_state.mode = stabilization;break;
			case 1000:  copter_state.mode = height;break;
			case 1500:  copter_state.mode = position;break;
		}

	switch(copter_rec_data.RC_switch_right)
		{
			case 500:   armed_flag = armed; break;
			case 1000:  armed_flag = emergency_stop;break;
			case 1500:  armed_flag = ready;break;
		}

	if(!first_flag)
	{
		if( !copter_state.arm && 
				(copter_rec_data.RC_throttle  >= rocker_min - rocker_inter || copter_rec_data.RC_throttle  <= rocker_min + rocker_inter))			
			offboard_flag  = 1;//已经升空
	    else if (offboard_flag != 0 &&
				(copter_rec_data.RC_throttle  >= rocker_min - rocker_inter || copter_rec_data.RC_throttle  <= rocker_min + rocker_inter))
			board_flag = 1;//准备降落

		//满足切换到起飞模式和油门杆在最低位置时进入起飞状态
		if( copter_state_last.arm != copter_state.arm && copter_state.arm == armed 
			&& (copter_rec_data.RC_throttle  >= rocker_min - rocker_inter || copter_rec_data.RC_throttle  <= rocker_min + rocker_inter))
			copter_state.arm = 0;

/*如果无法进入armed状态LED闪红灯*/
/*
	code
*/

		if(board_flag == 1)
			copter_state.arm = 2;//进入降落模式

		if(armed_flag)
			copter_state.arm = 1;//紧急停止
	}else{
		first_flag = 0;
	}

		copter_state_last.arm = armed_flag;
		copter_state_last.mode = copter_state.mode;

	Remote_Routine_Set(&remote_data_process);
}

