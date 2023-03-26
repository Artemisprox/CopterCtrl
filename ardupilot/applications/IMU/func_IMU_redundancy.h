<<<<<<< Updated upstream
=======
<<<<<<< HEAD
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
=======
>>>>>>> Stashed changes
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

extern Sensor_RAW_t copter_IMU_RAW;
<<<<<<< Updated upstream
=======
>>>>>>> 1fd7eb8ca870c0bbf1d38229d317ead000e81871
>>>>>>> Stashed changes
