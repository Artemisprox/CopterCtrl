#include <rtthread.h>
#include "drv_motor.h"
#include "arm_math.h"

#define sinf(a)  arm_sin_f32(a)
#define cosf(a)  arm_cos_f32(a)

#define YAW_ID       0x201
#define PITCH_ID     0x202
#define GIMBAL_ID 	 0x206

#define BULLET_MASS 0.0017f
#define BULLET_INITAL_NUM 1000
#define GIMBAL_DIS  0.3f

#define COMPENSATE_OPEN 0
typedef struct 
{

    uint8_t shooting_flag;//是否正在射击
    uint8_t speed;       //弹速
    uint8_t frequency;   //射频
    uint16_t quantity;   //已发射子弹数量
    uint8_t data_valid;  //云台数据是否可用

    Motor_t Yaw;
    Motor_t Pitch;

    uint32_t fresh_time;//数据更新时间
}gun_data;

typedef struct 
{
    float bullet_num;//当前弹丸数量
    float pitch_angle;//pitch角度补偿
    float x_torque;//x轴力矩补偿
    float roll_angle;//roll角度补偿
    float y_torque;//y轴力矩补偿

    uint32_t fresh_time;//数据更新时间
    rt_uint8_t en_flag;//根据发弹指示判断是否执行后坐力补偿 
}recoil_data;

extern void recoil_compensate_init(void);
extern void gun_readmsg(rt_uint8_t rxmsg[]);
extern gun_data copter_gun;
