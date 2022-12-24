#include "stm32f4xx_hal.h"
#include <rtthread.h>

#define PPM_PIN GET_PIN(C,6)
#define s1_low			800
#define s1_high		1200
#define s2_low			800
#define s2_high		1200

void pulse_process(void *args);
void RC_PPM_REC_Thread(void *Para);
rt_err_t RC_PPM_Init(void);

typedef struct
{
	int RC_throttle;
	int RC_yaw;
	int RC_pitch;
	int RC_roll;
	int RC_switch_left;
	int RC_switch_right;
	int RC_roller;
}RC_PPM_data;

extern RC_PPM_data copter_ctrl;



