#include <rtthread.h>
#include "func_SensorRAW.h"

typedef enum
{
    IMU1_set = 1,
    IMU2_set
}IMU_set_e;

typedef struct 
{
   int IMU1_fresh_last;
   int IMU2_fresh_last;
   IMU_set_e IMU_using;
   rt_int8_t IMU1_state;
   rt_int8_t IMU2_state;
}IMU_redun;

extern void IMU_WaitForRawData(void);
extern Sensor_RAW_t copter_IMU_RAW;
extern void IMU_redundancy_init(void);
