#include "drv_RC_PPM.h"
#include "func_remote.h"
#include "drv_utils.h"
#include "drv_dataserve.h"
#include "func_state.h"

static remote_data copter_remote ={0};
RC_PPM_data copter_rec_data_last = {0};
uint8_t Package_ID;
uint8_t first_flag = 1;
rt_int8_t status_ID;
static status copter_status;
/*低通滤波*/
static int16_t low_pass_filter(int16_t data_now , int16_t data_last , float param)
{  
	return data_now*param + data_last*(1-param);
}

/*将遥控器数据转化为控制数据*/
static void remote_setpoint(uint8_t mode , RC_PPM_data *data)
{
	
	switch (mode)
	{
		case POSITION ://定点模式：高度速度、x轴速度、y轴速度、yaw轴角速度
			copter_remote.throttle = (data->RC_throttle - rocker_middle)/rocker_width*2.0f*HEIGHT_MAX_V;
			copter_remote.pitch = (data->RC_pitch - rocker_middle)/rocker_width*2.0f*POS_X_MAX_V;
			copter_remote.roll  = (data->RC_roll - rocker_middle)/rocker_width*2.0f*POS_Y_MAX_V;
			copter_remote.yaw   = (data->RC_yaw - rocker_middle)/rocker_width*2.0f*YAW_MAX_SPE;
			break;
		case HEIGHT ://定高模式：高度速度、pitch轴角度、roll轴角度、yaw轴角速度
			copter_remote.throttle = (data->RC_throttle - rocker_middle)/rocker_width*2.0f*HEIGHT_MAX_V;
			copter_remote.pitch = (data->RC_pitch - rocker_middle)/rocker_width*2.0f*PITCH_MAX_DEG;
			copter_remote.roll  = (data->RC_roll - rocker_middle)/rocker_width*2.0f*ROLL_MAX_DEG;
			copter_remote.yaw   = (data->RC_yaw - rocker_middle)/rocker_width*2.0f*YAW_MAX_SPE;
			break;
		case STABILIZATION ://自稳模式：推力、pitch轴角度、roll轴角度、yaw轴角速度
			copter_remote.throttle = (data->RC_throttle - rocker_min)/rocker_width*F_MAX;
			copter_remote.pitch = (data->RC_pitch - rocker_middle)/rocker_width*2.0f*PITCH_MAX_DEG;
			copter_remote.roll  = (data->RC_roll - rocker_middle)/rocker_width*2.0f*ROLL_MAX_DEG;
			copter_remote.yaw   = -(data->RC_yaw - rocker_middle)/rocker_width*2.0f*YAW_MAX_SPE;
			break;
	}
}

static void remote_data_process(void)
{	
	/*摇杆数据先过一遍低通滤波*/
	if(!first_flag)
	{
		copter_rec_data.RC_throttle = low_pass_filter(copter_rec_data.RC_throttle , copter_rec_data_last.RC_throttle , LPF_k);
		copter_rec_data.RC_roll = low_pass_filter(copter_rec_data.RC_roll , copter_rec_data_last.RC_roll , LPF_k);
		copter_rec_data.RC_pitch = low_pass_filter(copter_rec_data.RC_pitch , copter_rec_data_last.RC_pitch , LPF_k);
		copter_rec_data.RC_yaw = low_pass_filter(copter_rec_data.RC_yaw , copter_rec_data_last.RC_yaw , LPF_k);
	}

	/*数据服务器接收*/
	status *p_1 =  Package_Pionter_Single(status_ID,status);
	copter_status = *p_1 ;
	Package_Write_Pionter_End(status_ID,status);
		copter_status.mode = STABILIZATION;
		copter_status.rc_status = 1;

	if(!copter_status.rc_status)//遥控器离线，保持当前状态不动
	{
		copter_rec_data.RC_throttle = 1519;
		copter_rec_data.RC_roll = 1519;
		copter_rec_data.RC_pitch = 1519;
		copter_rec_data.RC_yaw = 1519;
	}
	
	//根据当前飞行模式设置遥控数据
	remote_setpoint(copter_status.mode , &copter_rec_data );
			
	/*三档开关*/
	switch(copter_rec_data.RC_switch_left)
		{
			case 500:   copter_remote.switch_mode = STABILIZATION;break;
			case 1000:  copter_remote.switch_mode = HEIGHT;break;
			case 1500:  copter_remote.switch_mode = POSITION;break;
		}

	switch(copter_rec_data.RC_switch_right)
		{
			case 500:   copter_remote.switch_arm = ARMED_T; break;
			case 1000:  copter_remote.switch_arm = READY_T;break;
			case 1500:  copter_remote.switch_arm = EMERGENCY_STOP_T;break;
		}

	/*进行遥控器摇杆特殊位置检测、开关切换检测*/
	if(!first_flag)
	{
		/*油门低位*/
		if( (copter_rec_data.RC_throttle  >= rocker_min - rocker_inter) && (copter_rec_data.RC_throttle  <= rocker_min + rocker_inter))			
			copter_remote.throttle_low_flag = 1;
	    else copter_remote.throttle_low_flag = 0;
		/*pitch中位*/
		if( (copter_rec_data.RC_pitch  >= rocker_middle - rocker_inter) && (copter_rec_data.RC_pitch  <= rocker_middle + rocker_inter))
			copter_remote.pitch_middle_flag = 1;
		else copter_remote.pitch_middle_flag = 0;
		/*roll中位*/
		if( (copter_rec_data.RC_roll  >= rocker_middle - rocker_inter) && (copter_rec_data.RC_roll  <= rocker_middle + rocker_inter))
			copter_remote.roll_middle_flag = 1;
		else copter_remote.roll_middle_flag = 0;
		/*yaw中位*/
		if( (copter_rec_data.RC_yaw  >= rocker_middle - rocker_inter) && (copter_rec_data.RC_yaw  <= rocker_middle + rocker_inter))
			copter_remote.yaw_middle_flag = 1;
		else copter_remote.yaw_middle_flag = 0;
		/*左开关切换*/
		if(copter_rec_data.RC_switch_left != copter_rec_data_last.RC_switch_left)
			copter_remote.switch_mode_change = 1;
		else copter_remote.switch_mode_change = 0;
		/*右开关切换*/
		if(copter_rec_data.RC_switch_right != copter_rec_data_last.RC_switch_right)
			copter_remote.switch_arm_change = 1;
		else copter_remote.switch_arm_change = 0;

	}

	first_flag = 0;
	copter_remote.fresh_time = rt_tick_get();//更新数据更新时间

	/*进行数据存储*/
	copter_rec_data_last.RC_throttle = copter_rec_data.RC_throttle;
	copter_rec_data_last.RC_roll = copter_rec_data.RC_roll;
	copter_rec_data_last.RC_pitch = copter_rec_data.RC_pitch;
	copter_rec_data_last.RC_yaw = copter_rec_data.RC_yaw;
	copter_rec_data_last.RC_switch_left = copter_rec_data.RC_switch_left;
	copter_rec_data_last.RC_switch_right = copter_rec_data.RC_switch_right;
	copter_rec_data_last.RC_roller = copter_rec_data.RC_roller;

	/*数据服务器写入*/
	remote_data *p =  Package_Pionter_Single(Package_ID,remote_data);
	*p = copter_remote;
	Package_Write_Pionter_End(Package_ID,remote_data);

}

int RC_init(void)
{
	//通过函数指针转移至遥控器数据接收线程中进行处理
	Remote_Routine_Set(&remote_data_process);
	RC_PPM_Init();
  Package_Pionter_Add("remote", remote_data);
	Package_ID = Package_Find_Num("remote");
	status_ID = Package_Find_Num("status");
	return RT_EOK;
}
