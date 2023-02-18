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
	int16_t RC_throttle;
	int16_t RC_yaw;
	int16_t RC_pitch;
	int16_t RC_roll;
	int16_t RC_switch_left;
	int16_t RC_switch_right;
	int16_t RC_roller;
}RC_PPM_data;

void Remote_Routine_Set(void (*Func)(void));

extern RC_PPM_data copter_rec_data;

