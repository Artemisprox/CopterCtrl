#include "drv_PWM_motor.h"
#include "pid.h"

#define sinf(a)  arm_sin_f32(a)
#define cosf(a)  arm_cos_f32(a)

#define MAX_DUTY   0.8
#define MIN_DUTY   0.2

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
	uint8_t arm;
	uint8_t mode;
	uint8_t heath;
}state;

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
