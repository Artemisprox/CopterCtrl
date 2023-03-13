#include "drv_RC_PPM.h"
#include "func_remote.h"
#include "drv_utils.h"

remote_data copter_remote ={0};
RC_PPM_data copter_rec_data_last = {0};

uint8_t first_flag = 1;

/*低通滤波*/
static int16_t low_pass_filter(int16_t data_now , int16_t data_last , float param)
{  
	return data_now*param + data_last*(1-param);
}

static void remote_data_process(void)
{	
	/*遥控器摇杆的返回值先过一遍低通滤波消除抖动*/
	if(!first_flag)
	{
		copter_rec_data.RC_throttle = low_pass_filter(copter_rec_data.RC_throttle , copter_rec_data_last.RC_throttle , LPF_k);
		copter_rec_data.RC_roll = low_pass_filter(copter_rec_data.RC_roll , copter_rec_data_last.RC_roll , LPF_k);
		copter_rec_data.RC_pitch = low_pass_filter(copter_rec_data.RC_pitch , copter_rec_data_last.RC_pitch , LPF_k);
		copter_rec_data.RC_yaw = low_pass_filter(copter_rec_data.RC_yaw , copter_rec_data_last.RC_yaw , LPF_k);
	}

	/*将摇杆的值转化为控制设定值*/
	copter_remote.f = (copter_rec_data.RC_throttle/2000.0f - rocker_min)/(rocker_max - rocker_min)*f_max;
	copter_remote.pitch_deg = (copter_rec_data.RC_pitch/2000.0f - rocker_min)/(rocker_max - rocker_min)*pitch_max_degree;
	copter_remote.roll_deg  = (copter_rec_data.RC_roll/2000.0f - rocker_min)/(rocker_max - rocker_min)*roll_max_degree;
	copter_remote.yaw_spe   = (copter_rec_data.RC_yaw/2000.0f - rocker_min)/(rocker_max - rocker_min)*yaw_max_spe;
	
	/*左摇杆用于控制飞行模式，右摇杆用于飞机起飞与急停*/
	switch(copter_rec_data.RC_switch_left)
		{
			case 500:   copter_remote.switch_mode = stabilization;break;
			case 1000:  copter_remote.switch_mode = height;break;
			case 1500:  copter_remote.switch_mode = position;break;
		}

	switch(copter_rec_data.RC_switch_right)
		{
			case 500:   copter_remote.switch_arm = armed; break;
			case 1000:  copter_remote.switch_arm = ready;break;//起飞与急停中间间隔一个位置防止误操作
			case 1500:  copter_remote.switch_arm = emergency_stop;break;
		}

	/*遥控器摇杆改变、特殊位置检测*/
	if(!first_flag)
	{
		/*油门低位*/
		if( (copter_rec_data.RC_throttle  >= rocker_min - rocker_inter || copter_rec_data.RC_throttle  <= rocker_min + rocker_inter))			
			copter_remote.throttle_low_flag = 1;
	    else copter_remote.throttle_low_flag = 0;
		/*pitch轴角度中间位置*/
		if( (copter_rec_data.RC_pitch  >= rocker_middle - rocker_inter || copter_rec_data.RC_pitch  <= rocker_middle + rocker_inter))
			copter_remote.pitch_middle_flag = 1;
		else copter_remote.pitch_middle_flag = 0;
		/*roll轴角度中间位置*/
		if( (copter_rec_data.RC_roll  >= rocker_middle - rocker_inter || copter_rec_data.RC_roll  <= rocker_middle + rocker_inter))
			copter_remote.roll_middle_flag = 1;
		else copter_remote.roll_middle_flag = 0;
		/*yaw轴角度中间位置*/
		if( (copter_rec_data.RC_yaw  >= rocker_middle - rocker_inter || copter_rec_data.RC_yaw  <= rocker_middle + rocker_inter))
			copter_remote.yaw_middle_flag = 1;
		else copter_remote.yaw_middle_flag = 0;
		/*左开关发生改变*/
		if(copter_rec_data.RC_switch_left != copter_rec_data_last.RC_switch_left)
			copter_remote.switch_mode_change = 1;
		else copter_remote.switch_mode_change = 0;
		/*右开关发生改变*/
		if(copter_rec_data.RC_switch_right != copter_rec_data_last.RC_switch_right)
			copter_remote.switch_arm_change = 1;
		else copter_remote.switch_mode_change = 0;

	}

	first_flag = 0;

	/*记录上次遥控器数据*/
	copter_rec_data_last.RC_throttle = copter_rec_data.RC_throttle;
	copter_rec_data_last.RC_roll = copter_rec_data.RC_roll;
	copter_rec_data_last.RC_pitch = copter_rec_data_last.RC_pitch;
	copter_rec_data_last.RC_yaw = copter_rec_data_last.RC_yaw;
	copter_rec_data_last.RC_switch_left = copter_rec_data.RC_switch_left;
	copter_rec_data_last.RC_switch_left = copter_rec_data.RC_switch_left;
	copter_rec_data_last.RC_roller = copter_rec_data.RC_roller;
	
/*以下为数据服务器位置*/

//函数指针，在遥控器读取线程进行
	Remote_Routine_Set(&remote_data_process);
}
