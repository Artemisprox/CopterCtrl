

#ifndef __ICM20602_H__
#define __ICM20602_H__

#include <stdint.h>
#include <rtthread.h>
#include <rtdevice.h>
#include <board.h>

extern int icm20602_get_gyro_IMU1(float *gyro);
extern int icm20602_get_accel_IMU1(float *accel);
extern int icm20602_get_temper_IMU1(float *temper);

extern int icm20602_get_gyro_IMU2(float *gyro);
extern int icm20602_get_accel_IMU2(float *accel);
extern int icm20602_get_temper_IMU2(float *temper);

extern void ICM_init(void);

#endif
