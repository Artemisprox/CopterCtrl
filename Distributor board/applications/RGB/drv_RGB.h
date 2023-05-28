#ifndef _DRV_RGB_H_
#define _DRV_RGB_H_

#include <rtthread.h>
#include <rtdevice.h>
#include <board.h>

#define A_RGB_R_Channel 1//TIM2
#define A_RGB_G_Channel 2
#define A_RGB_B_Channel 3

#define B_RGB_R_Channel 4//TIM2
#define B_RGB_G_Channel 1//TIM1
#define B_RGB_B_Channel 2

#define C_RGB_R_Channel 3
#define C_RGB_G_Channel 4
#define C_RGB_B_PIN GET_PIN(A,7)

#define D_RGB_R_PIN GET_PIN(B,14)
#define D_RGB_G_PIN GET_PIN(B,12)
#define D_RGB_B_PIN GET_PIN(B,13)

#define onboard_RGB_R_Channel 2//TIM3
#define onboard_RGB_G_Channel 3
#define onboard_RGB_B_Channel 4

typedef enum
{
	RGB_A = 1,
	RGB_B,
	RGB_C,
	RGB_D,
	RGB_onboard
}RGB_set;

extern void RGB_Colour_Set(RGB_set rgb_ctrl , rt_int16_t RGB_R_Pulse, rt_int16_t RGB_G_Pulse, rt_int16_t RGB_B_Pulse);
extern void RGB_Init(void);

#endif /* _DRV_RGB_H_ */
