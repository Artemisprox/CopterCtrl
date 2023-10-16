#ifndef CAN_TEST_H
#define CAN_TEST_H

#include <rtdevice.h>
#include "drv_motor.h"

Motor_t *Read_Gun_Motor(void);

rt_err_t CAN_Init(void);

#endif /* CAN_TEST_H */
