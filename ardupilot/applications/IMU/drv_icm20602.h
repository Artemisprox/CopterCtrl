

#ifndef __ICM20602_H__
#define __ICM20602_H__

#include <stdint.h>
#include <rtthread.h>
#include <rtdevice.h>
#include <board.h>

extern int icm20602_get_gyro(float *gyro);
extern int icm20602_get_accel(float *accel);
extern int icm20602_get_temper(float *temper);

extern void ICM_init(void);

#endif

