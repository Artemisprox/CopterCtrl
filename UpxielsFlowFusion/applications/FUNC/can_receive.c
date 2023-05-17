#include "can_receive.h"
#include "func_IMUCOM.h"

volatile int Last_CANID[2]; // 用于检查收到的无效报文的ID

void can_rec(struct rt_can_msg *msg)
{
    switch (msg->id)
    {
    case IMU_ANGLE_ID:
        IMU_angle_readmsg(msg->data, &IMU_RawData);
    case IMU_SPE_ID:
        IMU_spe_readmsg(msg->data, &IMU_RawData);
    default:
        Last_CANID[0] = msg->id;
        return;
    }
}


