#ifndef _DRV_HW_PWN_H_
#define _DRV_HW_PWN_H_

#include "stm32f1xx_hal.h"

void HW_PWM_Init(void);
void HW_PWM_Pulse_Set(TIM_TypeDef *TIMx , uint32_t Channel, int16_t Pulse);

#endif /* _DRV_HW_PWN_H_ */
