#include <rtthread.h>
#include "drv_motor.h"

#define YAW_ID       0x201
#define PITCH_ID     0x202
#define GIMBAL_ID 	 0x101

typedef struct 
{

    uint8_t shooting_flag;//是否正在射击
    uint8_t speed;       //弹速
    uint8_t frequency;   //射频
    uint16_t quantity;   //已发射子弹数量
    uint8_t data_valid;  //云台数据是否可用

    Motor_t Yaw;
    uint32_t yaw_fresh_time;
    Motor_t Pitch;
    uint32_t pitch_fresh_time;

    uint32_t fresh_time;//数据更新时间
}gun_data;


extern gun_data copter_gun;
