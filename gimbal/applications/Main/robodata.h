#ifndef __ROBODATA_H__
#define __ROBODATA_H__

#include <rtthread.h>
#include <rtdevice.h>

// 用于调试，写1清空所有PID参数
#define TEST_CLEAR_PID (0)

#define PID_CLEAR 0, 0, 0, 0, 0, -0 // 空 PID

/********************************* 机器人通用操作设置 *********************************/
#define USE_SHOOT_LIMIT (0)           // 是否使用自动开火
#define AIMBOT_FIX_GAIN 0.2f          // 自瞄模式下操作手移动鼠标对云台的微调程度
#define SHIFT_UP 2.f                  // 操作手按下 Shift 以后的移动速度增益
#define CTRL_LOW 0.5f                 // 操作手按下 Ctrl 以后的移动速度衰减
#define MOUSE_SPEED_K 15.0f           // 补弹模式下的鼠标移动速度增益
#define MOUSE_SPEED_GAIN_PITCH 0.006f // 正常控制时鼠标移动速度对 Pitch 轴设定值的增益
#define MOUSE_SPEED_GAIN_YAW 0.018f   // 正常控制时鼠标移动速度对 Yaw 轴设定值的增益
#if defined CORE_USING_INFANTRY
#define SMALLGYRO_ROTATE_SPEED 900      // 小陀螺状态下的旋转速度
#define FASTGYRO_ROTATE_SPEED 2700      // 快陀螺下的旋转速度
#define FASTGYRO_ROTATE_CHANGE_A 400    // 快陀螺旋转速度变化的振幅
#define FASTGYRO_RATATE_CHANGE_W 0.009f // 快陀螺旋转速度变化的角频率
#elif defined CORE_USING_HERO
#define MOUSE_SPEED_EXTRAGAIN_DANGLING 0.012f // 吊射模式下鼠标移动数据增益
#define SMALLGYRO_ROTATE_SPEED 900            // 小陀螺状态下的旋转速度
#define FASTGYRO_ROTATE_SPEED 1800            // 快陀螺下的旋转速度
#define FASTGYRO_ROTATE_CHANGE_A 420          // 快陀螺旋转速度变化的振幅
#define FASTGYRO_RATATE_CHANGE_W 0.0085f      // 快陀螺旋转速度变化的角频率
#endif

#include "roboselect.h"

typedef enum
{
    //发送
    CHASSIS_CTL = 0x100,       // 底盘：底盘通信的云台发送运动相关数据的ID
    CHASSIS_DATA = 0x401,      // 底盘：底盘通信的云台发送弹速等其他数据的ID
    CHASSIS_IMU_DATA = 0x402,  // 底盘：底盘通信的云台发送弹速等其他数据的ID
    IMU_CTL = 0x300,           // 云台imu：向陀螺仪发送数据ID（切换短时积分用）
    GIMBAL_CTL = 0x1FF,        // 云台电机：云台PITCH和YAW电机写电流用的公共ID
    PITCH_ID = PITCH_MOTOR_ID, // pitch电机：云台pitch电机数据更新ID
    YAW_ID = YAW_MOTOR_ID,     // yaw电机：云台yaw电机数据更新ID
                               // DUAL_PITCH_ID = 0x207,    // pitch电机：英雄的第二个pitch轴电机数据更新ID

    //接收
    CHASSIS_REC = 0x105,          // 底盘回传数据ID
    CHASSIS_POSITION_REC = 0x106, // 底盘回传自身位置增量数据ID
    GYRO_ANGLE_ID = 0x001,        // 云台imu：云台接收陀螺仪角度的CAN数据ID
    GYRO_SPEED_ID = 0x002,        // 云台imu：云台接收陀螺仪角速度的CAN数据ID
    LAUNCH_ID = LAUNCH_MOTOR_ID,  // 拨弹：拨弹电机的发送电流用的ID
} drv_can1ID_e;
// CAN1设备ID

typedef enum
{
    // 发送
    STRIKE_ID = 0x200,             // 发送摩擦轮电机用的报文
    ID_VISUAL_ATTI_SEND = 0x012,   // 视觉：电控发送报文
    ID_VISUAL_TIMING_SEND = 0x011, // 向视觉发送对时信息

    // 接收
    ID_VISUALDATA_AIMFLAGS = 0x021,
    ID_VISUALDATA_GIMBALSET = 0x022,

    ID_RUB_LEFT = RUB_LEFT_MOTOR_ID,
    ID_RUB_RIGHT = RUB_RIGHT_MOTOR_ID,
} drv_can2ID_e;
//云台CAN2设备ID

#endif
