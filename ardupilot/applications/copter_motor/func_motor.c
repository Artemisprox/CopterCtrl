#include "func_motor.h"
#include "drv_PWM_motor.h"
#include "pid.h"
#include "roboselect.h"
#include "drv_thread.h"
#include "drv_IMU.h"
#include "drv_NimingFlow.h"
#include "arm_math.h"
#include "func_remote.h"

//电机的线性化模型:W0 + k*duty
#define propeller_spe_base 1000 //基础转速
#define propeller_spe_gain 6374 //占空比的转速增益

static struct rt_semaphore atti_1ms_sem; /* 用于接收消息的信号量 */
static struct rt_semaphore pos_20ms_sem; /* 用于接收消息的信号量 */
static struct rt_timer atti_1ms_tim;         /* 闭环线程定时器 */
static struct rt_timer pos_20ms_tim;         /* 闭环线程定时器 */

copter_ctrl HERO_copter;
state HERO_state;

//x,y大地坐标系变换到机体坐标系
static void earth_body_tranfer(void)
{	
	HERO_copter.copter_pitch.ang.set = ((1 - cosf(HERO_IMU.yaw)*cosf(HERO_IMU.yaw))/sinf(HERO_IMU.yaw))*HERO_copter.copter_x.vec.out
										- cosf(HERO_IMU.yaw)*HERO_copter.copter_y.vec.out;
	HERO_copter.copter_roll.ang.set = cosf(HERO_IMU.yaw)*HERO_copter.copter_x.vec.out 
										+ sinf(HERO_IMU.yaw)*HERO_copter.copter_y.vec.out;
}

//混控器进行动力分配
static void matrix_control(void)
{
	mixer_spe temp;
	temp.motor_spe1 = HERO_copter.copter_mixer.f/Ct
					- HERO_copter.copter_mixer.tau_x*COPTER_ARM_LENGTH/Ct 
					- HERO_copter.copter_mixer.tau_y*COPTER_ARM_LENGTH/Ct
					- HERO_copter.copter_mixer.tau_z*COPTER_ARM_LENGTH/Cm;
	if(temp.motor_spe1 >= propeller_spe_base)
		{
			HERO_copter.copter_mixer.motor_duty1 = (__sqrtf(temp.motor_spe1) - propeller_spe_base)/propeller_spe_gain;//计算得到占空比
		}
	else 
		HERO_copter.copter_mixer.motor_duty1 = 0;

	temp.motor_spe2 = HERO_copter.copter_mixer.f/Ct
					+ HERO_copter.copter_mixer.tau_x*COPTER_ARM_LENGTH/Ct 
					+ HERO_copter.copter_mixer.tau_y*COPTER_ARM_LENGTH/Ct
					- HERO_copter.copter_mixer.tau_z*COPTER_ARM_LENGTH/Cm;
    if(temp.motor_spe2 >= propeller_spe_base)
		{
			HERO_copter.copter_mixer.motor_duty2 = (__sqrtf(temp.motor_spe2) - propeller_spe_base)/propeller_spe_gain;//计算得到占空比
		}
	else 
		HERO_copter.copter_mixer.motor_duty2 = 0;

    temp.motor_spe3 = HERO_copter.copter_mixer.f/Ct
					+ HERO_copter.copter_mixer.tau_x*COPTER_ARM_LENGTH/Ct 
					- HERO_copter.copter_mixer.tau_y*COPTER_ARM_LENGTH/Ct
					+ HERO_copter.copter_mixer.tau_z*COPTER_ARM_LENGTH/Cm;
	if(temp.motor_spe3 >= propeller_spe_base)
		{
			HERO_copter.copter_mixer.motor_duty3 = (__sqrtf(temp.motor_spe3) - propeller_spe_base)/propeller_spe_gain;//计算得到占空比
		}
	else 
		HERO_copter.copter_mixer.motor_duty3 = 0;
	
	temp.motor_spe4 = HERO_copter.copter_mixer.f/Ct
					- HERO_copter.copter_mixer.tau_x*COPTER_ARM_LENGTH/Ct 
					+ HERO_copter.copter_mixer.tau_y*COPTER_ARM_LENGTH/Ct
					+ HERO_copter.copter_mixer.tau_z*COPTER_ARM_LENGTH/Cm;
	if(temp.motor_spe4 >= propeller_spe_base)
		{
			HERO_copter.copter_mixer.motor_duty4 = (__sqrtf(temp.motor_spe4) - propeller_spe_base)/propeller_spe_gain;//计算得到占空比
		}
	else 
		HERO_copter.copter_mixer.motor_duty4 = 0;
}

//姿态控制
static void atti_1ms_entry(void *parameter)
{
	rt_sem_take(&atti_1ms_sem,RT_WAITING_FOREVER);

	float error1 = 0,error2 = 0,error3 = 0;
	if(copter_state.mode == position)
	{
		error1 = HERO_copter.copter_pitch.ang.set - HERO_IMU.pitch ;
		error2 = HERO_copter.copter_roll.ang.set - HERO_IMU.roll ;
	}else if( copter_state.mode == height || copter_state.mode == stabilization)
	{
		error1 = copter_remote.pitch_deg - HERO_IMU.pitch ;
		error2 = copter_remote.roll_deg - HERO_IMU.roll ;
	}

	PID_Calculate(&HERO_copter.copter_pitch.ang,error1);
	error1 = HERO_copter.copter_pitch.ang.out - HERO_IMU.pitch_speed;
	PID_Calculate(&HERO_copter.copter_pitch.spe,error1);
	HERO_copter.copter_mixer.tau_y = HERO_copter.copter_pitch.ang.out;

	PID_Calculate(&HERO_copter.copter_roll.ang,error2);
	error2 = HERO_copter.copter_roll.ang.out - HERO_IMU.roll_speed;
	PID_Calculate(&HERO_copter.copter_roll.spe,error2);
	HERO_copter.copter_mixer.tau_x = HERO_copter.copter_roll.ang.out;

	error3 = 0 - HERO_IMU.yaw ;
	PID_Calculate(&HERO_copter.copter_yaw.ang,error3);
	error3 = HERO_copter.copter_yaw.ang.out - HERO_IMU.yaw_speed;
	PID_Calculate(&HERO_copter.copter_yaw.spe,error3);
	HERO_copter.copter_mixer.tau_z = HERO_copter.copter_yaw.ang.out;

	matrix_control();//混控器进行动力分配

	MX_TIM_DUTY(TIM1,COPTER_MOTOR_1,HERO_copter.copter_mixer.motor_duty1);
	MX_TIM_DUTY(TIM1,COPTER_MOTOR_2,HERO_copter.copter_mixer.motor_duty2);
	MX_TIM_DUTY(TIM1,COPTER_MOTOR_3,HERO_copter.copter_mixer.motor_duty3);
	MX_TIM_DUTY(TIM1,COPTER_MOTOR_4,HERO_copter.copter_mixer.motor_duty4);

}

//位置控制
static void pos_20ms_entry(void *parameter)
{
	rt_sem_take(&pos_20ms_sem,RT_WAITING_FOREVER);

	float error = 0;
	
	if(copter_state.mode == position )
	{
		error = 0 - NiMingFlow_data.pos_x;
		PID_Calculate(&HERO_copter.copter_x.pos,error);
		error = HERO_copter.copter_x.pos.out - NiMingFlow_data.Vx_Flow;
		PID_Calculate(&HERO_copter.copter_x.vec,error);
		
		error = 0 - NiMingFlow_data.pos_y;
		PID_Calculate(&HERO_copter.copter_y.pos,error);
		error = HERO_copter.copter_y.pos.out - NiMingFlow_data.Vy_Flow;
		PID_Calculate(&HERO_copter.copter_y.vec,error);
		
		error = 0 - NiMingFlow_data.distance;
		PID_Calculate(&HERO_copter.copter_h.pos,error);
		error = HERO_copter.copter_h.pos.out - - NiMingFlow_data.distance_v;
		PID_Calculate(&HERO_copter.copter_h.vec,error);
		HERO_copter.copter_mixer.f =  HERO_copter.copter_h.vec.out + MASS*g;

		earth_body_tranfer();//坐标变换
	}else if(copter_state.mode == height)
	{

		error = 0 - NiMingFlow_data.distance;
		PID_Calculate(&HERO_copter.copter_h.pos,error);
		error = HERO_copter.copter_h.pos.out - NiMingFlow_data.distance_v;
		PID_Calculate(&HERO_copter.copter_h.vec,error);
		HERO_copter.copter_mixer.f =  HERO_copter.copter_h.vec.out + MASS*g;

	}else if(copter_state.mode == stabilization)
	{
		HERO_copter.copter_mixer.f =  copter_remote.f ;
	}

}

static void atti_1ms_IRQHandler(void *parameter)
{
	while (rt_sem_trytake(&atti_1ms_sem) == RT_EOK)
        continue; // 清空多余的信号量
    rt_sem_release(&atti_1ms_sem);
}

static void pos_20ms_IRQHandler(void *parameter)
{
	while (rt_sem_trytake(&pos_20ms_sem) == RT_EOK)
        continue; // 清空多余的信号量
    rt_sem_release(&pos_20ms_sem);
}

static void motor_start(void)
{
    /*定时器处理线程*/
    rt_thread_t thread;
    rt_sem_init(&atti_1ms_sem, "copter_atti", 0, RT_IPC_FLAG_FIFO);
		rt_sem_init(&pos_20ms_sem, "copter_pos", 0, RT_IPC_FLAG_FIFO);
    thread = rt_thread_create("atti_ctrl", atti_1ms_entry, RT_NULL, 2048, THREAD_PRIO_STRIKEPID, 1);
		thread = rt_thread_create("pos_ctrl", pos_20ms_entry, RT_NULL, 2048, THREAD_PRIO_STRIKEPID, 1);
    if (thread != RT_NULL)
        rt_thread_startup(thread);

    /*定时器中断*/
    rt_timer_init(&atti_1ms_tim, "atti_Tim", atti_1ms_IRQHandler, RT_NULL, 1,
                  RT_TIMER_FLAG_PERIODIC | RT_TIMER_FLAG_SOFT_TIMER);
		 rt_timer_init(&pos_20ms_tim, "pos_Tim", pos_20ms_IRQHandler, RT_NULL, 20,
                  RT_TIMER_FLAG_PERIODIC | RT_TIMER_FLAG_SOFT_TIMER);
    /* 启动定时器 */
    rt_timer_start(&atti_1ms_tim);
		rt_timer_start(&pos_20ms_tim);
}

void Motor_init(void)
{
	
	MX_TIM1_PWM_Init();
	//姿态
	pid_init(&HERO_copter.copter_pitch.ang,PITCHANG_PID);
	pid_init(&HERO_copter.copter_pitch.spe,PITCHSPE_PID);
	pid_init(&HERO_copter.copter_roll.ang,ROLLANG_PID);
	pid_init(&HERO_copter.copter_roll.spe,ROLLSPE_PID);
	pid_init(&HERO_copter.copter_yaw.ang,YAWANG_PID);
	pid_init(&HERO_copter.copter_yaw.spe,YAWSPE_PID);
	//位置
	pid_init(&HERO_copter.copter_h.pos,POS_H_PID);
	pid_init(&HERO_copter.copter_h.pos,VEC_H_PID);
	pid_init(&HERO_copter.copter_x.pos,POS_X_PID);
	pid_init(&HERO_copter.copter_x.pos,VEC_X_PID);
	pid_init(&HERO_copter.copter_y.pos,POS_Y_PID);
	pid_init(&HERO_copter.copter_y.pos,VEC_Y_PID);
	motor_start();
}
