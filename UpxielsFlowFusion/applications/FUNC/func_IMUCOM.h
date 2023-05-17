#include <rtthread.h>

#define IMU_ANGLE_ID 0x001
#define IMU_SPE_ID   0x002
#define IMU_ACC_ID   0x003
#define IMU_Q_ID     0x004


typedef struct 
{
    float Pitch;
    float Roll;
    float Yaw;
    float PitchSpe;
    float RollSpe;
    float YawSpe;
}IMU_data;

extern void  IMU_angle_readmsg(rt_uint8_t rxmsg[], IMU_data *imu);
extern void  IMU_spe_readmsg(rt_uint8_t rxmsg[], IMU_data *imu);

extern IMU_data IMU_RawData;
