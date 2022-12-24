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

extern TempCTR_t HERO_TPctr;
extern struct rt_semaphore temp_pid_sem;
extern int IfTempOK;

extern int TempCTR_init(void);

#endif

