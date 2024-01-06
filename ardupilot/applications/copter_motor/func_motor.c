#include "func_motor.h"
#include "drv_PWM_motor.h"
#include "pid.h"
#include "roboselect.h"
#include "drv_thread.h"
#include "drv_IMU.h"
#include "arm_math.h"
#include "func_remote.h"
#include "mod_recoil_force_compensate.h"
#include "func_sensor.h"
#include "func_state.h"
#include "drv_dataserve.h"
#include "drv_utils.h"
#include "drv_IMUPosCali.h"
#include "drv_SetPlanning.h"
#include "drv_ExactSmooth.h"
#include "TEST.h"
#include "Filter.h"

//电机模型:W0 + k*duty
#define propeller_spe_base 1201 //基础转速
#define propeller_spe_gain 7309 //转速增益
#define spe_k 1.0
static struct rt_semaphore atti_5ms_sem; /* 姿态环 */
static struct rt_semaphore pos_20ms_sem; /* 位置环 */
static struct rt_timer atti_5ms_tim;         /* 姿态环定时器 */
static struct rt_timer pos_20ms_tim;         /* 位置环定时器 */

static recoil_data copter_gimbal;
static copter_ctrl HERO_copter;
static remote_data control_data;
static status copter_status;
pos_sensor local_pos;

float gyro_raw[3]={0};//记录角速度

static uint8_t remote_ID,IMU_ID,sensor_ID,status_ID,recoil_ID;

SetPlanning_Str HeightCtrl ;
ExactSmth_CTRL_S SmoothHeight;

//机体系变换到大地系
static void earth_body_tranfer(void)
{	
	HERO_copter.copter_pitch.ang.set = ((1 - cosf(HERO_IMU.yaw)*cosf(HERO_IMU.yaw))/sinf(HERO_IMU.yaw))*HERO_copter.copter_x.vec.out
										- cosf(HERO_IMU.yaw)*HERO_copter.copter_y.vec.out;
	HERO_copter.copter_roll.ang.set = cosf(HERO_IMU.yaw)*HERO_copter.copter_x.vec.out 
										+ sinf(HERO_IMU.yaw)*HERO_copter.copter_y.vec.out;
}

static mixer_spe temp;
//混控器
float temp_data[20];
static void matrix_control(void)
{
	static float duty[4] ={0};
//	temp_data[0] = 0.5*HERO_copter.copter_mixer.f/Ct;
//	temp_data[1] = 0.5*HERO_copter.copter_mixer.tau_x/COPTER_ARM_LENGTH/Ct ;
//	temp_data[2] = 0.5*HERO_copter.copter_mixer.tau_y/COPTER_ARM_LENGTH/Ct ;
//	temp_data[3] = 0.5*HERO_copter.copter_mixer.tau_z/COPTER_ARM_LENGTH/Cm;
	temp.motor_spe1 = 0.25f*HERO_copter.copter_mixer.f/Ct
					- 0.25f*HERO_copter.copter_mixer.tau_x/COPTER_ARM_LENGTH/Ct 
					- 0.25f*HERO_copter.copter_mixer.tau_y/COPTER_ARM_LENGTH/Ct
					- 0.25f*HERO_copter.copter_mixer.tau_z/COPTER_ARM_LENGTH/Cm;
	if(temp.motor_spe1 < 0)
		temp.motor_spe1 = 0;
	
	temp.motor_spe2 = 0.25f*HERO_copter.copter_mixer.f/Ct
					+ 0.25f*HERO_copter.copter_mixer.tau_x/COPTER_ARM_LENGTH/Ct
					+ 0.25f*HERO_copter.copter_mixer.tau_y/COPTER_ARM_LENGTH/Ct
					- 0.25f*HERO_copter.copter_mixer.tau_z/COPTER_ARM_LENGTH/Cm;
	if(temp.motor_spe2 < 0)
		temp.motor_spe2 = 0;

    temp.motor_spe3 = 0.25f*HERO_copter.copter_mixer.f/Ct
					+ 0.25f*HERO_copter.copter_mixer.tau_x/COPTER_ARM_LENGTH/Ct
					- 0.25f*HERO_copter.copter_mixer.tau_y/COPTER_ARM_LENGTH/Ct
					+ 0.25f*HERO_copter.copter_mixer.tau_z/COPTER_ARM_LENGTH/Cm;
	if(temp.motor_spe3 < 0)
		temp.motor_spe3 = 0;

	temp.motor_spe4 = 0.25f*HERO_copter.copter_mixer.f/Ct
					- 0.25f*HERO_copter.copter_mixer.tau_x/COPTER_ARM_LENGTH/Ct
					+ 0.25f*HERO_copter.copter_mixer.tau_y/COPTER_ARM_LENGTH/Ct
					+ 0.25f*HERO_copter.copter_mixer.tau_z/COPTER_ARM_LENGTH/Cm;
	if(temp.motor_spe4 < 0)
		temp.motor_spe4 = 0;
	
	HERO_copter.copter_mixer.tau_x = 0;
	HERO_copter.copter_mixer.tau_y = 0;
	HERO_copter.copter_mixer.tau_z = 0;
	
	duty[0] = (sqrtf(temp.motor_spe1) - propeller_spe_base)/propeller_spe_gain;//最终占空比
	utils_truncate_number(&duty[0],CTRL_LIMIT_DOWN,CTRL_LIMIT_UP);
	HERO_copter.copter_mixer.motor_duty1 = utils_map(duty[0],0.0f,1.0f,MIN_DUTY,MAX_DUTY);
	
	duty[1] = (sqrtf(temp.motor_spe2) - propeller_spe_base)/propeller_spe_gain;//最终占空比
	utils_truncate_number(&duty[1],CTRL_LIMIT_DOWN,CTRL_LIMIT_UP);
	HERO_copter.copter_mixer.motor_duty2 = utils_map(duty[1],0.0f,1.0f,MIN_DUTY,MAX_DUTY);	
	
	duty[2] = (sqrtf(temp.motor_spe3) - propeller_spe_base)/propeller_spe_gain;//最终占空比
	utils_truncate_number(&duty[2],CTRL_LIMIT_DOWN,CTRL_LIMIT_UP);
	HERO_copter.copter_mixer.motor_duty3 = utils_map(duty[2],0.0f,1.0f,MIN_DUTY,MAX_DUTY);	
	
	duty[3] = (sqrtf(temp.motor_spe4) - propeller_spe_base)/propeller_spe_gain;//最终占空比
	utils_truncate_number(&duty[3],CTRL_LIMIT_DOWN,CTRL_LIMIT_UP);
	HERO_copter.copter_mixer.motor_duty4 = utils_map(duty[3],0.0f,1.0f,MIN_DUTY,MAX_DUTY);
	

}

//速度环控制
static void velocity_control(float error_x, float error_y, pid_t* x, pid_t* y)
{
	PID_Calculate(x,error_x);
	PID_Calculate(y,error_y);
}

//姿态环线程
static void atti_5ms_entry(void *parameter)
{
	static int first_flag = 1;
	while(1)
	{
	rt_sem_take(&atti_5ms_sem,RT_WAITING_FOREVER);

	/*从数据服务器更新数据*/
	IMU_t *p_2 =  Package_Pionter_Single(IMU_ID,IMU_t);
	HERO_IMU = *p_2 ;
	Package_Write_Pionter_End(IMU_ID,IMU_t);
	
	if(first_flag)
	{	
		first_flag = 0;
		gyro_raw[0] = HERO_IMU.yaw_speed;
		gyro_raw[1] = HERO_IMU.pitch_speed;
		gyro_raw[2] = HERO_IMU.roll_speed;
	}
	//对角速度滤波
		gyro_raw[0] = low_pass_filter_f(HERO_IMU.yaw_speed ,gyro_raw[0],spe_k);
		gyro_raw[1] = low_pass_filter_f(HERO_IMU.pitch_speed ,gyro_raw[1],spe_k);
		gyro_raw[2] = low_pass_filter_f(HERO_IMU.roll_speed ,gyro_raw[2],spe_k);

	float error_p = 0,error_r = 0,error_y = 0;
	if((!copter_status.emergency) && (copter_status.flight_status != READY))
	{
		switch(copter_status.mode)
		{
			case POSITION:
				error_p = HERO_copter.copter_x.vec.out - HERO_IMU.pitch;
				error_r = HERO_copter.copter_y.vec.out - HERO_IMU.roll;
				break;
			default:
				error_p = control_data.pitch - HERO_IMU.pitch;
				error_r = control_data.roll - HERO_IMU.roll;
		}
	
//		if( copter_gimbal.en_flag )
//		{
//			error_p += copter_gimbal.pitch_angle;
//			error_r += copter_gimbal.roll_angle;
//		}
		
		PID_Calculate(&HERO_copter.copter_pitch.ang,error_p);
		error_p = HERO_copter.copter_pitch.ang.out - gyro_raw[1];
//		error_p = control_data.pitch - HERO_IMU.pitch_speed;
		PID_Calculate(&HERO_copter.copter_pitch.spe,error_p);
		HERO_copter.copter_mixer.tau_y = HERO_copter.copter_pitch.spe.out;

		PID_Calculate(&HERO_copter.copter_roll.ang,error_r);
		error_r = HERO_copter.copter_roll.ang.out - HERO_IMU.roll_speed;
//		ins_flow_data.flo[6] = HERO_copter.copter_roll.ang.out;
//		ins_flow_data.flo[7] = HERO_IMU.roll_speed;
//		error_r = control_data.roll - HERO_IMU.roll_speed;
		PID_Calculate(&HERO_copter.copter_roll.spe,error_r);
		HERO_copter.copter_mixer.tau_x = HERO_copter.copter_roll.spe.out;
 
		error_y = control_data.yaw - HERO_IMU.yaw_speed;
		PID_Calculate(&HERO_copter.copter_yaw.spe,error_y);
		HERO_copter.copter_mixer.tau_z = HERO_copter.copter_yaw.spe.out;

//		if( copter_gimbal.en_flag )
//		{
//			HERO_copter.copter_mixer.tau_x += copter_gimbal.x_torque;
//			HERO_copter.copter_mixer.tau_y += copter_gimbal.y_torque;
//		} 

		matrix_control();//混控器
	}else
	{
		HERO_copter.copter_mixer.motor_duty1 = MIN_DUTY;
		HERO_copter.copter_mixer.motor_duty2 = MIN_DUTY;
		HERO_copter.copter_mixer.motor_duty3 = MIN_DUTY;
		HERO_copter.copter_mixer.motor_duty4 = MIN_DUTY;
		//状态清零
		//姿态环
		pid_clear(&HERO_copter.copter_pitch.ang);
		pid_clear(&HERO_copter.copter_pitch.spe);
		pid_clear(&HERO_copter.copter_roll.ang);
		pid_clear(&HERO_copter.copter_roll.spe);
		pid_clear(&HERO_copter.copter_yaw.ang);
		pid_clear(&HERO_copter.copter_yaw.spe);
		//位置环
		pid_clear(&HERO_copter.copter_h.pos);
		pid_clear(&HERO_copter.copter_h.vec);
		pid_clear(&HERO_copter.copter_x.pos);
		pid_clear(&HERO_copter.copter_x.vec);
		pid_clear(&HERO_copter.copter_y.pos);
		pid_clear(&HERO_copter.copter_y.vec);
	}
	 MX_TIM_DUTY(TIM1,COPTER_MOTOR_1,HERO_copter.copter_mixer.motor_duty1);
	 MX_TIM_DUTY(TIM1,COPTER_MOTOR_2,HERO_copter.copter_mixer.motor_duty2);
	 MX_TIM_DUTY(TIM1,COPTER_MOTOR_3,HERO_copter.copter_mixer.motor_duty3);
	 MX_TIM_DUTY(TIM1,COPTER_MOTOR_4,HERO_copter.copter_mixer.motor_duty4);

	}
}

uint8_t pos_control_check(void)
{
	if( fabs(control_data.throttle) < 0.05f && fabs(local_pos.V_height) < 0.25f )
		return 1;
	else 
		return 0;
}

//位置线程
static void pos_20ms_entry(void *parameter)
{
	static int pos_h_flag = 0 , pos_x_flag = 0,pos_y_flag = 0;
	static float pos_h_set = 0, pos_x_set = 0,pos_y_set = 0;
	static int flag = 5;
	while(1)
	{
	rt_sem_take(&pos_20ms_sem,RT_WAITING_FOREVER);

	/*数据服务器数据更新*/
	remote_data *p_1 =  Package_Pionter_Single(remote_ID,remote_data);
	control_data = *p_1 ;
	Package_Write_Pionter_End(remote_ID,remote_data);

	status *p_2 =  Package_Pionter_Single(status_ID,status);
	copter_status = *p_2 ;
	Package_Write_Pionter_End(status_ID,status);

  pos_sensor *p_3 =  Package_Pionter_Single(sensor_ID,pos_sensor);
	local_pos = *p_3 ;
	Package_Write_Pionter_End(sensor_ID,pos_sensor);
	
	// recoil_data *p_4 =  Package_Pionter_Single(recoil_ID,recoil_data);
	// copter_gimbal = *p_4 ;
	// Package_Write_Pionter_End(recoil_ID,recoil_data);
	
	float error_x,error_y,error_h = 0;
	
	static uint8_t height_ctrl_first = 1;
//	if(copter_status.mode == HEIGHT || copter_status.mode == POSITION)
//	{	
//		if(height_ctrl_first)
//		{
//			HeightCtrl.Input.Set.pos = get_height();
//			height_ctrl_first = 0;
//		}
//	}

	if(!copter_status.emergency && copter_status.flight_status != READY)
		switch (copter_status.mode)
		{
		case POSITION:
		{
			HERO_copter.copter_x.pos.out = 0;
			if(fabs(control_data.pitch) < 0.05f)
			{
				if(fabs(local_pos.V_pos_x) < 0.15f)
				{
					if(pos_x_flag == 1)
					{
						pos_x_set = local_pos.pos_x;
						pos_x_flag = 0;
					}
					HERO_copter.copter_x.pos.set = pos_x_set;
					control_data.pitch = 0;
					error_x = HERO_copter.copter_x.pos.set - local_pos.pos_x;
					//PID_Calculate(&HERO_copter.copter_x.pos,error_x);
				}
			}
			else
			{
				pos_x_flag = 1;
			} 			
			
			/*速度闭环*/
			error_x = HERO_copter.copter_x.pos.out + control_data.pitch -  local_pos.V_pos_x;
			PID_Calculate(&HERO_copter.copter_x.vec,error_x);

			HERO_copter.copter_y.pos.out = 0;
			if(fabs(control_data.roll) < 0.05f)
			{
				if(fabs(local_pos.V_pos_y) < 0.15f)
				{
					if(pos_y_flag == 1)
					{
						pos_y_set = local_pos.pos_y;
						pos_y_flag = 0;
					}
					flag=1;
					HERO_copter.copter_y.pos.set = pos_y_set;
					control_data.roll = 0;
					error_y = HERO_copter.copter_y.pos.set - local_pos.pos_y;
					//PID_Calculate(&HERO_copter.copter_y.pos,error_y);
				}
			}
			else
			{
				flag=2;
				pos_y_flag = 1;
			} 			
			error_y = HERO_copter.copter_y.pos.out + control_data.roll - local_pos.V_pos_y;
			PID_Calculate(&HERO_copter.copter_y.vec,error_y);
		}
		//位置闭环包含速度闭环，故没有break
		case HEIGHT:
		{
			/*在正常定高模式下采用位置闭环，在进行人为控制时采用速度闭环，以提高响应速度*/
			/*模式切换的依据为 控制输入与当前速度 ， 当摇杆归中且速度降低至可接受范围内时，转入位置闭环*/
				HERO_copter.copter_h.pos.out = 0;
				if(fabs(control_data.throttle) < 0.05f)
				{
					if(fabs(local_pos.V_height) < 0.25f)
					{
						if(pos_h_flag == 1)
						{
							pos_h_set = get_height();
							pos_h_flag = 0;
						}
						HERO_copter.copter_h.pos.set = pos_h_set;
						control_data.throttle = 0;
						error_h = HERO_copter.copter_h.pos.set - local_pos.distance;
						PID_Calculate(&HERO_copter.copter_h.pos,error_h);
					}
				}
				else
				{
					pos_h_flag = 1;
				} 

				error_h = HERO_copter.copter_h.pos.out + control_data.throttle - local_pos.V_height;
				PID_Calculate(&HERO_copter.copter_h.vec,error_h);
				HERO_copter.copter_mixer.f = HERO_copter.copter_h.vec.out + MASS*g/arm_cos_f32(HERO_IMU.pitch/360.0f*2*3.1415926f)/arm_cos_f32(HERO_IMU.roll/360.0f*2*3.1415926f) ;
			break;
		}
		default:
			HERO_copter.copter_mixer.f =  control_data.throttle + (MASS - 0.4)*g/arm_cos_f32(HERO_IMU.pitch/360.0f*2*3.1415926f)/arm_cos_f32(HERO_IMU.roll/360.0f*2*3.1415926f);
//			HERO_copter.copter_mixer.f =  control_data.throttle;
		}
		ins_flow_data.flo[0] = pos_y_set;
		ins_flow_data.flo[1] = local_pos.distance;
		ins_flow_data.flo[2] = control_data.roll;
		ins_flow_data.flo[3] = local_pos.V_height;
		ins_flow_data.flo[6] = local_pos.V_pos_y;
		ins_flow_data.flo[7] = local_pos.pos_x;
		ins_flow_data.flo[8] = local_pos.pos_y;

	}
}

static void atti_5ms_IRQHandler(void *parameter)
{
	while (rt_sem_trytake(&atti_5ms_sem) == RT_EOK)
        continue; // 取完多余的信号量
    rt_sem_release(&atti_5ms_sem);
}

static void pos_20ms_IRQHandler(void *parameter)
{
	while (rt_sem_trytake(&pos_20ms_sem) == RT_EOK)
        continue; // 取完多余的信号量
    rt_sem_release(&pos_20ms_sem);
}

static void motor_start(void)
{
	/*数据服务器ID查找*/
    remote_ID = Package_Find_Num("remote");
    IMU_ID = Package_Find_Num("IMU");
    sensor_ID = Package_Find_Num("pos_sensor");
	status_ID = Package_Find_Num("status");
	recoil_ID = Package_Find_Num("compensate");


    /*线程初始化*/
    rt_thread_t thread;
    rt_sem_init(&atti_5ms_sem, "copter_atti", 0, RT_IPC_FLAG_FIFO);
	rt_sem_init(&pos_20ms_sem, "copter_pos", 0, RT_IPC_FLAG_FIFO);
    thread = rt_thread_create("atti_ctrl", atti_5ms_entry, RT_NULL, 2048, THREAD_PRIO_MOTOR_ATTI_CONTROL, 1);
	if (thread != RT_NULL)
        rt_thread_startup(thread);
	thread = rt_thread_create("pos_ctrl", pos_20ms_entry, RT_NULL, 2048, THREAD_PRIO_MOTOR_POS_CONTROL, 1);
    if (thread != RT_NULL)
        rt_thread_startup(thread);
	
	Smooth_Init(&SmoothHeight, 0.0f , 20);
	// 初始化设定值规划模块
    SetPlanSettings_Str Setting;
    Setting.dt = 0.02f;
    Setting.POS_Error_Max = 2.0f;
    Setting.Accl_Max = 4.0f;
    Setting.Speed_Max = 4.0f;
    SetPlanning_Init(&HeightCtrl, &Setting);

    /*定时线程*/
  	rt_timer_init(&atti_5ms_tim, "atti_Tim", atti_5ms_IRQHandler, RT_NULL, 5,
                  RT_TIMER_FLAG_PERIODIC | RT_TIMER_FLAG_SOFT_TIMER);
	rt_timer_init(&pos_20ms_tim, "pos_Tim", pos_20ms_IRQHandler, RT_NULL, 20,
                  RT_TIMER_FLAG_PERIODIC | RT_TIMER_FLAG_SOFT_TIMER);
    /* 开启定时器 */
 	rt_timer_start(&atti_5ms_tim);
	rt_timer_start(&pos_20ms_tim);
}

void Motor_init(void)
{
	IMU_PosCali_Init();

	MX_TIM1_PWM_Init(); 
	//姿态环
	pid_init(&HERO_copter.copter_pitch.ang,PITCHANG_PID);
	pid_init(&HERO_copter.copter_pitch.spe,PITCHSPE_PID);
	pid_init(&HERO_copter.copter_roll.ang,ROLLANG_PID);
	pid_init(&HERO_copter.copter_roll.spe,ROLLSPE_PID);
//	pid_init(&HERO_copter.copter_yaw.ang,YAWANG_PID);
	pid_init(&HERO_copter.copter_yaw.spe,YAWSPE_PID);
//	//位置环
	pid_init(&HERO_copter.copter_h.pos,POS_H_PID);
	pid_init(&HERO_copter.copter_h.vec,VEC_H_PID);
	pid_init(&HERO_copter.copter_x.pos,POS_X_PID);
	pid_init(&HERO_copter.copter_x.vec,VEC_X_PID);
	pid_init(&HERO_copter.copter_y.pos,POS_Y_PID);
	pid_init(&HERO_copter.copter_y.vec,VEC_Y_PID);
				  
//	elc_spd_ctrl_Init();
	motor_start();
}
