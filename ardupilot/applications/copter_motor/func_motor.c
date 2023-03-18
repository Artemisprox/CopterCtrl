#include "func_motor.h"
#include "drv_PWM_motor.h"
#include "pid.h"
#include "roboselect.h"
#include "drv_thread.h"
#include "drv_IMU.h"
#include "arm_math.h"
#include "func_remote.h"
#include "func_sensor.h"
#include "func_state.h"
#include "drv_dataserve.h"

//电机模型:W0 + k*duty
#define propeller_spe_base 1000 //基础转速
#define propeller_spe_gain 6374 //转速增益

static struct rt_semaphore atti_1ms_sem; /* 姿态环 */
static struct rt_semaphore pos_20ms_sem; /* 位置环 */
static struct rt_timer atti_1ms_tim;         /* 姿态环定时器 */
static struct rt_timer pos_20ms_tim;         /* 位置环定时器 */

copter_ctrl HERO_copter;
remote_data control_data;
status copter_status;
static uint8_t remote_ID,IMU_ID,sensor_ID,status_ID;

static void status_check(status* p_status , remote_data* p_data)
{

}

//机体系变换到大地系
static void earth_body_tranfer(void)
{	
	HERO_copter.copter_pitch.ang.set = ((1 - cosf(HERO_IMU.yaw)*cosf(HERO_IMU.yaw))/sinf(HERO_IMU.yaw))*HERO_copter.copter_x.vec.out
										- cosf(HERO_IMU.yaw)*HERO_copter.copter_y.vec.out;
	HERO_copter.copter_roll.ang.set = cosf(HERO_IMU.yaw)*HERO_copter.copter_x.vec.out 
										+ sinf(HERO_IMU.yaw)*HERO_copter.copter_y.vec.out;
}

//混控器
static void matrix_control(void)
{
	mixer_spe temp;
	temp.motor_spe1 = HERO_copter.copter_mixer.f/Ct
					- HERO_copter.copter_mixer.tau_x*COPTER_ARM_LENGTH/Ct 
					- HERO_copter.copter_mixer.tau_y*COPTER_ARM_LENGTH/Ct
					- HERO_copter.copter_mixer.tau_z*COPTER_ARM_LENGTH/Cm;
	HERO_copter.copter_mixer.motor_duty1 = (__sqrtf(temp.motor_spe1) - propeller_spe_base)/propeller_spe_gain;//最终占空比
	utils_truncate_number(&HERO_copter.copter_mixer.motor_duty1,MIN_DUTY,MAX_DUTY);

	temp.motor_spe2 = HERO_copter.copter_mixer.f/Ct
					+ HERO_copter.copter_mixer.tau_x*COPTER_ARM_LENGTH/Ct 
					+ HERO_copter.copter_mixer.tau_y*COPTER_ARM_LENGTH/Ct
					- HERO_copter.copter_mixer.tau_z*COPTER_ARM_LENGTH/Cm;
	HERO_copter.copter_mixer.motor_duty2 = (__sqrtf(temp.motor_spe2) - propeller_spe_base)/propeller_spe_gain;//最终占空比
	utils_truncate_number(&HERO_copter.copter_mixer.motor_duty2,MIN_DUTY,MAX_DUTY);

    temp.motor_spe3 = HERO_copter.copter_mixer.f/Ct
					+ HERO_copter.copter_mixer.tau_x*COPTER_ARM_LENGTH/Ct 
					- HERO_copter.copter_mixer.tau_y*COPTER_ARM_LENGTH/Ct
					+ HERO_copter.copter_mixer.tau_z*COPTER_ARM_LENGTH/Cm;
	HERO_copter.copter_mixer.motor_duty3 = (__sqrtf(temp.motor_spe3) - propeller_spe_base)/propeller_spe_gain;//最终占空比
	utils_truncate_number(&HERO_copter.copter_mixer.motor_duty3,MIN_DUTY,MAX_DUTY);
	
	temp.motor_spe4 = HERO_copter.copter_mixer.f/Ct
					- HERO_copter.copter_mixer.tau_x*COPTER_ARM_LENGTH/Ct 
					+ HERO_copter.copter_mixer.tau_y*COPTER_ARM_LENGTH/Ct
					+ HERO_copter.copter_mixer.tau_z*COPTER_ARM_LENGTH/Cm;
	HERO_copter.copter_mixer.motor_duty4 = (__sqrtf(temp.motor_spe4) - propeller_spe_base)/propeller_spe_gain;//最终占空比
	utils_truncate_number(&HERO_copter.copter_mixer.motor_duty4,MIN_DUTY,MAX_DUTY);
}

//速度环控制
static void velocity_control(float error_x, float error_y, pid_t* x, pid_t* y)
{
	PID_Calculate(x,error_x);
	PID_Calculate(y,error_y);
}


//姿态环线程
static void atti_1ms_entry(void *parameter)
{
	rt_sem_take(&atti_1ms_sem,RT_WAITING_FOREVER);

	/*从数据服务器更新数据*/
	IMU_t *p_2 =  Package_Pionter_Add(IMU_ID,HERO_IMU);
	HERO_IMU = *p_2 ;
	Package_Write_Pionter_End(IMU_ID,HERO_IMU);

	float error_p = 0,error_r = 0,error_y = 0;
	if(!copter_status.emergency && copter_status.flight_status != READY)
	{
		switch(copter_status.mode)
		{
			case POSITION:
				error_p = HERO_copter.copter_pitch.ang.set - HERO_IMU.pitch ;
				error_r = HERO_copter.copter_roll.ang.set - HERO_IMU.roll ;
			default:
				error_p = control_data.pitch - HERO_IMU.pitch ;
				error_r = control_data.roll - HERO_IMU.roll ;
		}

		PID_Calculate(&HERO_copter.copter_pitch.ang,error_p);
		error_p = HERO_copter.copter_pitch.ang.out - HERO_IMU.pitch_speed;
		PID_Calculate(&HERO_copter.copter_pitch.spe,error_p);
		HERO_copter.copter_mixer.tau_y = HERO_copter.copter_pitch.ang.out;

		PID_Calculate(&HERO_copter.copter_roll.ang,error_r);
		error_r = HERO_copter.copter_roll.ang.out - HERO_IMU.roll_speed;
		PID_Calculate(&HERO_copter.copter_roll.spe,error_r);
		HERO_copter.copter_mixer.tau_x = HERO_copter.copter_roll.ang.out;

		error_y = control_data.yaw - HERO_IMU.yaw_speed;
		PID_Calculate(&HERO_copter.copter_yaw.spe,error_y);
		HERO_copter.copter_mixer.tau_z = HERO_copter.copter_yaw.ang.out;

		matrix_control();//混控器
	}else
	{
		HERO_copter.copter_mixer.motor_duty1 = 0;
		HERO_copter.copter_mixer.motor_duty2 = 0;
		HERO_copter.copter_mixer.motor_duty3 = 0;
		HERO_copter.copter_mixer.motor_duty4 = 0;
	}
	MX_TIM_DUTY(TIM1,COPTER_MOTOR_1,HERO_copter.copter_mixer.motor_duty1);
	MX_TIM_DUTY(TIM1,COPTER_MOTOR_2,HERO_copter.copter_mixer.motor_duty2);
	MX_TIM_DUTY(TIM1,COPTER_MOTOR_3,HERO_copter.copter_mixer.motor_duty3);
	MX_TIM_DUTY(TIM1,COPTER_MOTOR_4,HERO_copter.copter_mixer.motor_duty4);

}

//位置线程
static void pos_20ms_entry(void *parameter)
{
	rt_sem_take(&pos_20ms_sem,RT_WAITING_FOREVER);

	/*数据服务器数据更新*/
	remote_data *p_1 =  Package_Pionter_Add(remote_ID,control_data);
	control_data = *p_1 ;
	Package_Write_Pionter_End(remote_ID,control_data);

	status *p_2 =  Package_Pionter_Add(status_ID,copter_status);
	copter_status = *p_2 ;
	Package_Write_Pionter_End(status_ID,copter_status);

	pos_sensor local_pos;
    pos_sensor *p_3 =  Package_Pionter_Add(sensor_ID,local_pos);
	local_pos = *p_3 ;
	Package_Write_Pionter_End(sensor_ID,local_pos);

	status_check(&copter_status , &control_data);
	float error_x,error_y,error_h = 0;
	
	if(!copter_status.emergency && copter_status.flight_status != READY)
		switch (copter_status.mode)
		{
		case POSITION:
		{
			/*速度闭环*/
			error_x = control_data.pitch -  local_pos.V_pos_x;
			error_y = control_data.roll -  local_pos.V_pos_y;
			velocity_control(error_x,error_y,&HERO_copter.copter_x.vec , &HERO_copter.copter_y.vec );
		}
		case HEIGHT:
		{
			error_h = control_data.throttle - local_pos.V_height;
			PID_Calculate(&HERO_copter.copter_h.vec,error_h);
		}
			break;
		default:
			HERO_copter.copter_mixer.f =  control_data.throttle;
		}
}

static void atti_1ms_IRQHandler(void *parameter)
{
	while (rt_sem_trytake(&atti_1ms_sem) == RT_EOK)
        continue; // 取完多余的信号量
    rt_sem_release(&atti_1ms_sem);
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
    /*线程初始化*/
    rt_thread_t thread;
    rt_sem_init(&atti_1ms_sem, "copter_atti", 0, RT_IPC_FLAG_FIFO);
		rt_sem_init(&pos_20ms_sem, "copter_pos", 0, RT_IPC_FLAG_FIFO);
    thread = rt_thread_create("atti_ctrl", atti_1ms_entry, RT_NULL, 2048, THREAD_PRIO_STRIKEPID, 1);
		thread = rt_thread_create("pos_ctrl", pos_20ms_entry, RT_NULL, 2048, THREAD_PRIO_STRIKEPID, 1);
    if (thread != RT_NULL)
        rt_thread_startup(thread);

    /*定时线程*/
    rt_timer_init(&atti_1ms_tim, "atti_Tim", atti_1ms_IRQHandler, RT_NULL, 1,
                  RT_TIMER_FLAG_PERIODIC | RT_TIMER_FLAG_SOFT_TIMER);
		 rt_timer_init(&pos_20ms_tim, "pos_Tim", pos_20ms_IRQHandler, RT_NULL, 20,
                  RT_TIMER_FLAG_PERIODIC | RT_TIMER_FLAG_SOFT_TIMER);
    /* 开启定时器 */
    rt_timer_start(&atti_1ms_tim);
		rt_timer_start(&pos_20ms_tim);
}

void Motor_init(void)
{
	
	MX_TIM1_PWM_Init();
	//姿态环
	pid_init(&HERO_copter.copter_pitch.ang,PITCHANG_PID);
	pid_init(&HERO_copter.copter_pitch.spe,PITCHSPE_PID);
	pid_init(&HERO_copter.copter_roll.ang,ROLLANG_PID);
	pid_init(&HERO_copter.copter_roll.spe,ROLLSPE_PID);
	pid_init(&HERO_copter.copter_yaw.ang,YAWANG_PID);
	pid_init(&HERO_copter.copter_yaw.spe,YAWSPE_PID);
	//位置环
	pid_init(&HERO_copter.copter_h.pos,POS_H_PID);
	pid_init(&HERO_copter.copter_h.pos,VEC_H_PID);
	pid_init(&HERO_copter.copter_x.pos,POS_X_PID);
	pid_init(&HERO_copter.copter_x.pos,VEC_X_PID);
	pid_init(&HERO_copter.copter_y.pos,POS_Y_PID);
	pid_init(&HERO_copter.copter_y.pos,VEC_Y_PID);
	motor_start();
}
