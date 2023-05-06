#include "drv_PWM_motor.h"
#include "pid.h"

#define sinf(a)  arm_sin_f32(a)
#define cosf(a)  arm_cos_f32(a)
//#define sqrt(a,b) arm_sqrt_f32_t(a,b)

#define CTRL_LIMIT_UP 0.90f
#define CTRL_LIMIT_DOWN 0.15f

#define MAX_DUTY   0.8f
#define MIN_DUTY   0.36f

typedef struct
{
	pid_t pos;
	pid_t vec;
}pos_channel;

typedef struct
{
	pid_t ang;
	pid_t spe;
}atti_channel;

typedef struct 
{
	float f;
	float tau_x;
	float tau_y;
	float tau_z;
	
	float motor_duty1;
	float motor_duty2;
	float motor_duty3;
	float motor_duty4;
}mixer;

typedef struct 
{
	float motor_spe1;
	float motor_spe2;
	float motor_spe3;
	float motor_spe4;
}mixer_spe;


typedef struct
{
	pos_channel copter_x;
	pos_channel copter_y;
	pos_channel copter_h;
	
	atti_channel copter_pitch;
	atti_channel copter_roll;
	atti_channel copter_yaw;
	
	mixer copter_mixer;
}copter_ctrl;

extern void Motor_init(void);
