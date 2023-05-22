#include "func_IMUCOM.h"

IMU_data IMU_RawData = {0};

void  IMU_angle_readmsg(rt_uint8_t rxmsg[], IMU_data *imu)
{
    imu->Roll = ((rt_int16_t)(rxmsg[0] << 8 | rxmsg[1]))/100.0f;
    imu->Yaw = ((rt_int16_t)(rxmsg[2] << 8 | rxmsg[3]))/100.0f;
    imu->Pitch = ((rt_int16_t)(rxmsg[4] << 8 | rxmsg[5]))/100.0f;
}

void  IMU_spe_readmsg(rt_uint8_t rxmsg[], IMU_data *imu)
{
    imu->RollSpe = ((rt_int16_t)(rxmsg[0] << 8 | rxmsg[1]))/300.0f;
    imu->YawSpe = ((rt_int16_t)(rxmsg[2] << 8 | rxmsg[3]))/300.0f;
    imu->PitchSpe = ((rt_int16_t)(rxmsg[4] << 8 | rxmsg[5]))/300.0f;
		
//		if(imu->YawSpe > 30)
//			imu->YawSpe = 0;
}
