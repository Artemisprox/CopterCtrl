#include "drv_RC_PPM.h"
#include "func_remote.h"
#include "drv_utils.h"

remote_data copter_remote ={0};
RC_PPM_data copter_rec_data_last = {0};

uint8_t first_flag = 1;

/*��ͨ�˲�*/
static int16_t low_pass_filter(int16_t data_now , int16_t data_last , float param)
{  
	return data_now*param + data_last*(1-param);
}

static void 

static void remote_setpoint(uint8_t mode , RC_PPM_data *data)
{
	switch (mode)
	{
		case POSITION :
			copter_remote.throttle = (data->RC_throttle/2000.0f - rocker_middle)/rocker_width/2.0f*HEIGHT_MAX_V;
			copter_remote.pitch = (data->RC_pitch/2000.0f - rocker_middle)/rocker_width/2.0f*POS_X_MAX_V;
			copter_remote.roll  = (data->RC_roll/2000.0f - rocker_middle)/rocker_width/2.0f*POS_Y_MAX_V;
			copter_remote.yaw   = (data->RC_yaw/2000.0f - rocker_middle)/rocker_width/2.0f*YAW_MAX_SPE;
			break;
		case HEIGHT :
			copter_remote.throttle = (data->RC_throttle/2000.0f - rocker_middle)/rocker_width/2.0f*HEIGHT_MAX_V;
			copter_remote.pitch = (data->RC_pitch/2000.0f - rocker_middle)/rocker_width/2.0f*PITCH_MAX_DEG;
			copter_remote.roll  = (data->RC_roll/2000.0f - rocker_middle)/rocker_width/2.0f*ROLL_MAX_DEG;
			copter_remote.yaw   = (data->RC_yaw/2000.0f - rocker_middle)/rocker_width/2.0f*YAW_MAX_SPE;
			break;
		case STABILIZATION :
			copter_remote.throttle = (data->RC_throttle/2000.0f - rocker_min)/rocker_width/2.0f*F_MAX;
			copter_remote.pitch = (data->RC_pitch/2000.0f - rocker_middle)/rocker_width/2.0f*PITCH_MAX_DEG;
			copter_remote.roll  = (data->RC_roll/2000.0f - rocker_middle)/rocker_width/2.0f*ROLL_MAX_DEG;
			copter_remote.yaw   = (data->RC_yaw/2000.0f - rocker_middle)/rocker_width/2.0f*YAW_MAX_SPE;
			break;
	}
}

static void remote_data_process(void)
{	
	/*ң����ҡ�˵ķ���ֵ�ȹ�һ���ͨ�˲���������*/
	if(!first_flag)
	{
		copter_rec_data.RC_throttle = low_pass_filter(copter_rec_data.RC_throttle , copter_rec_data_last.RC_throttle , LPF_k);
		copter_rec_data.RC_roll = low_pass_filter(copter_rec_data.RC_roll , copter_rec_data_last.RC_roll , LPF_k);
		copter_rec_data.RC_pitch = low_pass_filter(copter_rec_data.RC_pitch , copter_rec_data_last.RC_pitch , LPF_k);
		copter_rec_data.RC_yaw = low_pass_filter(copter_rec_data.RC_yaw , copter_rec_data_last.RC_yaw , LPF_k);
	}

	/*��ȡ��ǰ����ģʽ��ȷ���趨ֵ*/
	uint8_t flight_mode;
	uint8_t rc_status;

	if(!rc_status)//遥控器离线，保持当前状态不动
	{
		copter_rec_data.RC_throttle = rocker_middle;
		copter_rec_data.RC_roll = rocker_middle;
		copter_rec_data.RC_pitch = rocker_middle;
		copter_rec_data.RC_yaw = rocker_middle;
	}
	
	remote_setpoint(flight_mode , &copter_rec_data );
			
	/*��ҡ�����ڿ��Ʒ���ģʽ����ҡ�����ڷɻ�����뼱ͣ*/
	switch(copter_rec_data.RC_switch_left)
		{
			case 500:   copter_remote.switch_mode = STABILIZATION;break;
			case 1000:  copter_remote.switch_mode = HEIGHT;break;
			case 1500:  copter_remote.switch_mode = POSITION;break;
		}

	switch(copter_rec_data.RC_switch_right)
		{
			case 500:   copter_remote.switch_arm = ARMED; break;
			case 1000:  copter_remote.switch_arm = READY;break;//����뼱ͣ�м���һ��λ�÷�ֹ�����
			case 1500:  copter_remote.switch_arm = EMERGENCY_STOP;break;
		}

	/*ң����ҡ�˸ı䡢����λ�ü��*/
	if(!first_flag)
	{
		/*���ŵ�λ*/
		if( (copter_rec_data.RC_throttle  >= rocker_min - rocker_inter || copter_rec_data.RC_throttle  <= rocker_min + rocker_inter))			
			copter_remote.throttle_low_flag = 1;
	    else copter_remote.throttle_low_flag = 0;
		/*pitch��Ƕ��м�λ��*/
		if( (copter_rec_data.RC_pitch  >= rocker_middle - rocker_inter || copter_rec_data.RC_pitch  <= rocker_middle + rocker_inter))
			copter_remote.pitch_middle_flag = 1;
		else copter_remote.pitch_middle_flag = 0;
		/*roll��Ƕ��м�λ��*/
		if( (copter_rec_data.RC_roll  >= rocker_middle - rocker_inter || copter_rec_data.RC_roll  <= rocker_middle + rocker_inter))
			copter_remote.roll_middle_flag = 1;
		else copter_remote.roll_middle_flag = 0;
		/*yaw��Ƕ��м�λ��*/
		if( (copter_rec_data.RC_yaw  >= rocker_middle - rocker_inter || copter_rec_data.RC_yaw  <= rocker_middle + rocker_inter))
			copter_remote.yaw_middle_flag = 1;
		else copter_remote.yaw_middle_flag = 0;
		/*�󿪹ط����ı�*/
		if(copter_rec_data.RC_switch_left != copter_rec_data_last.RC_switch_left)
			copter_remote.switch_mode_change = 1;
		else copter_remote.switch_mode_change = 0;
		/*�ҿ��ط����ı�*/
		if(copter_rec_data.RC_switch_right != copter_rec_data_last.RC_switch_right)
			copter_remote.switch_arm_change = 1;
		else copter_remote.switch_mode_change = 0;

	}

	first_flag = 0;
	copter_remote.fresh_time = rt_tick_get();//��¼��ǰʱ��

	/*��¼�ϴ�ң��������*/
	copter_rec_data_last.RC_throttle = copter_rec_data.RC_throttle;
	copter_rec_data_last.RC_roll = copter_rec_data.RC_roll;
	copter_rec_data_last.RC_pitch = copter_rec_data_last.RC_pitch;
	copter_rec_data_last.RC_yaw = copter_rec_data_last.RC_yaw;
	copter_rec_data_last.RC_switch_left = copter_rec_data.RC_switch_left;
	copter_rec_data_last.RC_switch_left = copter_rec_data.RC_switch_left;
	copter_rec_data_last.RC_roller = copter_rec_data.RC_roller;

	/*д�����ݷ�����*/

//����ָ�룬��ң������ȡ�߳̽���
	Remote_Routine_Set(&remote_data_process);
}
