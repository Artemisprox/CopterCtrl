#ifndef __FUNC_TEMPCTR_H__
#define __FUNC_TEMPCTR_H__
/*
        IMU温度控制
*/
#include <rtthread.h>
#include "pid.h"

typedef struct
{
    pid_t TempCTR_pid;
}TempCTR_t;
//电机PID闭环结构体

extern TempCTR_t IMU1_TPctr;
extern TempCTR_t IMU2_TPctr;
extern struct rt_semaphore imu1_temp_pid_sem;
extern struct rt_semaphore imu2_temp_pid_sem;
extern int IMU1_IfTempOK;
extern int IMU2_IfTempOK;

extern int IMU1_TempCTR_init(void);
extern int IMU2_TempCTR_init(void);

#endif
