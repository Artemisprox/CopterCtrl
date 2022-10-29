/**
 * @file roboselect.h
 * @brief 本文件用于记录一些机器人个体参数
 * @author fwlh
 * @version 1.0
 * @date 2022-07-21
 *
 * @copyright Copyright (c) 2022  哈尔滨工业大学(威海)HERO战队
 */
#ifndef __ROBOSELECT_H__
#define __ROBOSELECT_H__

#include <rtconfig.h>

// IMU 坐标系相关说明
// 以云台为例：枪口方向为前方，垂直地面向上为上方，类似于第一视角
// X为 后方(枪管反方向轴), Y为 右方, Z为 上方
// GYRO_X_SOURCE -GYRO_RAWDATA(y)表示 云台后方数据=(-1*原始y轴数据)
// 角速度方向遵循右手螺旋

/****************************************************************************
 *                        步兵机器人
 * *************************************************************************/
#if defined CORE_USING_INFANTRY
#define SERVO_CTRL_EN (1) // 步兵机器人全部需要电动弹舱
/********************************* 绿头步兵 *********************************/
#if defined CORE_USING_GREENHEAD_INFANTRY
// IMU 坐标系相关设置
#define GYRO_X_SOURCE +GYRO_RAWDATA(x)
#define ACCL_X_SOURCE +ACCL_RAWDATA(x)
#define GYRO_Y_SOURCE -GYRO_RAWDATA(y)
#define ACCL_Y_SOURCE -ACCL_RAWDATA(y)
#define GYRO_Z_SOURCE -GYRO_RAWDATA(z)
#define ACCL_Z_SOURCE -ACCL_RAWDATA(z)
// 云台控制相关设置
#define PITCH_MOTOR_ID 0x206                            // Pitch 轴电机 ID
#define YAW_MOTOR_ID 0x205                              // Yaw 轴电机 ID
#undef CAR_USING_LINK                                   // Pitch 轴电机传动没有采用连杆结构
#define GIMBAL_BIAS_SET (-1800)                         // 云台重心电流补偿
#define YAW_ZERO_ANGLE 0                                // 正常跟随时 Yaw 电机编码器数值
#define PITCH_ZERO_ANGLE 0x505                          // Pitch 轴水平时 Pitch 电机编码器数值
#define PITCH_START_ANGLE 0                             // pitch上电后的初始角度设定值 单位°
#define PITCH_MOTOR_MIRROR 0                            // 0 表示 Pitch 使用默认转动方向, 即 Pitch 轴低头时电机编码器原始数据减小
#define YAW_MOTOR_MIRROR 0                              // 0 表示 Yaw 使用默认转动方向, 即 Yaw 轴向左旋转时电机编码器原始数据增大
#define PITCH_MIN_ANGLE (-25.0f)                        // pitch轴限幅 单位°
#define PITCH_MAX_ANGLE (27.0f)                         // pitch轴限幅 单位°
#define YAWSPE_PID 750, 3, 0, 1500, 25000, -25000       // IMU 闭环时的 Yaw 轴角速度环 PID
#define YAWANG_PID 38, 0.1, 100, 1.5, 300, -300         // IMU 闭环时的 Yaw 轴角度环 PID
#define PITSPE_PID 475, 5, 0, 2200, 25000, -25000       // IMU 闭环时的 Pitch 轴角速度环 PID
#define PITANG_PID 40, 0.1, 38, 1.5, 180, -180          // IMU 闭环时的 Pitch 轴角度环 PID
#define YAWSPE_ENCOD_PID 100, 0, 0, 0, 28000, -28000    // 编码器闭环时的 Yaw 轴角速度环 PID
#define YAWANG_ENCOD_PID 40, 1, 400, 10, 800, -800      // 编码器闭环时的 Yaw 轴角度环 PID
#define PITSPE_ENCOD_PID 450, 4, 0, 4000, 28000, -28000 // 编码器闭环时的 Pitch 轴角速度环 PID
#define PITANG_ENCOD_PID 20, 1, 20, 15, 300, -300       // 编码器闭环时的 Pitch 轴角度环 PID
// 发射机构相关设置
#define LAUNCH_MOTOR_ID 0x203                        // 播弹盘电机 ID
#define RUB_LEFT_MOTOR_ID 0x201                      // 左侧摩擦轮电机 ID
#define RUB_RIGHT_MOTOR_ID 0x202                     // 右侧摩擦轮电机 ID
#define GUN_RUB_TOGGLE (0)                           // 0 表示发射摩擦轮使用默认转动方向, 即左侧摩擦轮原始转速大于 0 时弹丸可以被发射出去
#define GUN_LAUNCH_TOGGLE (0)                        // 0 表示发射播弹盘使用默认转动方向, 即播弹盘原始转速大于 0 时弹丸可以被发射出去
#define GUN_SPEED_SET_15 3500                        // 实际设定弹速 15 时的摩擦轮转速设定值
#define GUN_SPEED_REAL_15 (14.3f)                    // SET15 对应的实际弹速
#define GUN_SPEED_SET_18 3950                        // 实际设定弹速 18 时的摩擦轮转速设定值
#define GUN_SPEED_REAL_18 (16.5f)                    // SET18 对应的实际弹速
#define GUN_SPEED_SET_30 4950                        // 实际设定弹速 30 时的摩擦轮转速设定值
#define GUN_SPEED_REAL_30 (19.8f)                    // SET30 对应的实际弹速
#define SET_BOOSTER_ADDRATIO (0.05f)                 // 拨弹叉每次运动到发射点之后继续转动的角度 单位：弹丸
#define LAUNCH_ANG_PID 10, 2, 40, 0.5, 500, -500     // 播弹盘电机角度环 PID
#define LAUNCH_SPE_PID 40, 10, 0, 600, 9000, -9000   // 播弹盘电机角速度环 PID
#define RUB_SPE_PID 20, 0.10, 30, 300, 14000, -14000 // 摩擦轮电机角速度环 PID
// 弹舱相关设置
#define MAGAZINE_SERVO_CLOSE_PULSE 2120 // 弹舱盖关闭时的脉宽
#define MAGAZINE_SERVO_OPEN_PULSE -980  // 弹舱盖从关闭到打开的脉宽行程
// 自瞄相关设置
#define CAMERA_PITCH_FIX (-4.85f)      // 弹丸出射速度绝对水平时，相机的Pitch姿态角, 填写数字越大，瞄准位置越靠上
#define CAMERA_YAW_FIX (0.0f)          // 枪口Yaw=0时，相机Yaw姿态角，填写数字越大，瞄准位置越靠左
#define IMU_PITCH_FIX (0.0f)           // 弹丸出射速度绝对水平时，陀螺仪发回的Pitch角度 单位 ° 用于修正机械安装误差
#define AIMBOT_CIMMUNICATION_USING_CAN // 与视觉通信使用 CAN 通信
/********************************* 碳板步兵 *********************************/
#elif defined CORE_USING_CARBONPLATE_INFANTRY
// IMU 坐标系相关设置
#define GYRO_X_SOURCE -GYRO_RAWDATA(y)
#define ACCL_X_SOURCE -ACCL_RAWDATA(y)
#define GYRO_Y_SOURCE +GYRO_RAWDATA(x)
#define ACCL_Y_SOURCE +ACCL_RAWDATA(x)
#define GYRO_Z_SOURCE +GYRO_RAWDATA(z)
#define ACCL_Z_SOURCE +ACCL_RAWDATA(z)
// 云台控制相关设置
#define PITCH_MOTOR_ID 0x206                            // Pitch 轴电机 ID
#define YAW_MOTOR_ID 0x205                              // Yaw 轴电机 ID
#undef CAR_USING_LINK                                   // Pitch 轴电机传动没有采用连杆结构
#define GIMBAL_BIAS_SET (2300)                          // 云台重心电流补偿
#define YAW_ZERO_ANGLE 0                                // 正常跟随时 Yaw 电机编码器数值
#define PITCH_ZERO_ANGLE 0x1D20                         // Pitch 轴水平时 Pitch 电机编码器数值
#define PITCH_START_ANGLE (-0.03f)                      // pitch上电后的初始角度设定值 单位°
#define PITCH_MOTOR_MIRROR 0                            // 0 表示 Pitch 使用默认转动方向, 即 Pitch 轴低头时电机编码器原始数据减小
#define YAW_MOTOR_MIRROR 0                              // 0 表示 Yaw 使用默认转动方向, 即 Yaw 轴向左旋转时电机编码器原始数据增大
#define PITCH_MIN_ANGLE (-29.0f)                        // pitch轴限幅 单位°
#define PITCH_MAX_ANGLE (32.0f)                         // pitch轴限幅 单位°
#define YAWSPE_PID 730, 25, 0, 2500, 25000, -25000      // IMU 闭环时的 Yaw 轴角速度环 PID
#define YAWANG_PID 40, 0.12, 30, 0.5, 400, -400         // IMU 闭环时的 Yaw 轴角度环 PID
#define PITSPE_PID 310, 0.65, 0, 4500, 25000, -25000    // IMU 闭环时的 Pitch 轴角速度环 PID
#define PITANG_PID 31, 0.2, 50, 1.5, 200, -200          // IMU 闭环时的 Pitch 轴角度环 PID
#define YAWSPE_ENCOD_PID 100, 0, 0, 0, 28000, -28000    // 编码器闭环时的 Yaw 轴角速度环 PID
#define YAWANG_ENCOD_PID 40, 1, 400, 10, 800, -800      // 编码器闭环时的 Yaw 轴角度环 PID
#define PITSPE_ENCOD_PID 350, 4, 0, 4200, 28000, -28000 // 编码器闭环时的 Pitch 轴角速度环 PID
#define PITANG_ENCOD_PID 20, 1, 20, 15, 300, -300       // 编码器闭环时的 Pitch 轴角度环 PID
// 发射机构相关设置
#define LAUNCH_MOTOR_ID 0x201                           // 播弹盘电机 ID
#define RUB_LEFT_MOTOR_ID 0x204                         // 左侧摩擦轮电机 ID
#define RUB_RIGHT_MOTOR_ID 0x203                        // 右侧摩擦轮电机 ID
#define GUN_RUB_TOGGLE (0)                              // 0 表示发射摩擦轮使用默认转动方向, 即左侧摩擦轮原始转速大于 0 时弹丸可以被发射出去
#define GUN_LAUNCH_TOGGLE (0)                           // 0 表示发射播弹盘使用默认转动方向, 即播弹盘原始转速大于 0 时弹丸可以被发射出去
#define GUN_SPEED_SET_15 3550                           // 实际设定弹速 15 时的摩擦轮转速设定值
#define GUN_SPEED_REAL_15 (14.1f)                       // SET15 对应的实际弹速
#define GUN_SPEED_SET_18 4000                           // 实际设定弹速 18 时的摩擦轮转速设定值
#define GUN_SPEED_REAL_18 (16.5f)                       // SET18 对应的实际弹速
#define GUN_SPEED_SET_30 5000                           // 实际设定弹速 30 时的摩擦轮转速设定值
#define GUN_SPEED_REAL_30 (19.8f)                       // SET30 对应的实际弹速
#define SET_BOOSTER_ADDRATIO (0.05f)                    // 拨弹叉每次运动到发射点之后继续转动的角度 单位：弹丸
#define LAUNCH_ANG_PID 10, 2, 40, 0.5, 500, -500        // 播弹盘电机角度环 PID
#define LAUNCH_SPE_PID 40, 10, 0, 600, 9000, -9000      // 播弹盘电机角速度环 PID
#define RUB_SPE_PID 20, 0.10, 30, 300, 14000, -14000    // 摩擦轮电机角速度环 PID
// 弹舱相关设置
#define MAGAZINE_SERVO_CLOSE_PULSE 991                  // 弹舱盖关闭时的脉宽
#define MAGAZINE_SERVO_OPEN_PULSE 931                   // 弹舱盖从关闭到打开的脉宽行程
// 自瞄相关设置
#define CAMERA_PITCH_FIX (-4.85f)                       // 弹丸出射速度绝对水平时，相机的Pitch姿态角, 填写数字越大，瞄准位置越靠上
#define CAMERA_YAW_FIX (0.0f)                           // 枪口Yaw=0时，相机Yaw姿态角，填写数字越大，瞄准位置越靠左
#define IMU_PITCH_FIX (0.0f)                            // 弹丸出射速度绝对水平时，陀螺仪发回的Pitch角度 单位 ° 用于修正机械安装误差
#define AIMBOT_CIMMUNICATION_USING_CAN                  // 与视觉通信使用 CAN 通信
/********************************* 富贵步兵 *********************************/
#elif defined CORE_USING_RICH_INFANTRY
// IMU 坐标系相关设置
#define GYRO_X_SOURCE -GYRO_RAWDATA(y)
#define ACCL_X_SOURCE -ACCL_RAWDATA(y)
#define GYRO_Y_SOURCE +GYRO_RAWDATA(x)
#define ACCL_Y_SOURCE +ACCL_RAWDATA(x)
#define GYRO_Z_SOURCE +GYRO_RAWDATA(z)
#define ACCL_Z_SOURCE +ACCL_RAWDATA(z)
// 云台控制相关设置
#define PITCH_MOTOR_ID 0x206                         // Pitch 轴电机 ID
#define YAW_MOTOR_ID 0x208                           // Yaw 轴电机 ID
#define CAR_USING_LINK                               // Pitch 轴电机传动采用连杆结构
#define CONNECT_ANGLE 90                             // 云台水平时连杆角度
#define PITCH_ZEROCURRENT_ANG (9.f)                  // 当Pitch = 3.0f 时，达到机械平衡点
#define PTICH_F0_CURRENT (-2500)                     // 重力补偿参数
#define YAW_ZERO_ANGLE 0                             // 正常跟随时 Yaw 电机编码器数值
#define PITCH_ZERO_ANGLE 4821                        // Pitch 轴水平时 Pitch 电机编码器数值
#define PITCH_START_ANGLE 0                          // pitch上电后的初始角度设定值 单位°
#define PITCH_MOTOR_MIRROR 1                         // 0 表示 Pitch 使用默认转动方向, 即 Pitch 轴低头时电机编码器原始数据减小
#define YAW_MOTOR_MIRROR 0                           // 0 表示 Yaw 使用默认转动方向, 即 Yaw 轴向左旋转时电机编码器原始数据增大
#define PITCH_MIN_ANGLE (-28.0f)                     // pitch轴限幅 单位°
#define PITCH_MAX_ANGLE (25.0f)                      // pitch轴限幅 单位°
#define YAWSPE_PID 430, 3, 0, 1500, 25000, -25000    // IMU 闭环时的 Yaw 轴角速度环 PID
#define YAWANG_PID 28, 0.1, 80, 5, 600, -600         // IMU 闭环时的 Yaw 轴角度环 PID
#define PITSPE_PID 420, 0.6, 0, 3500, 25000, -25000  // IMU 闭环时的 Pitch 轴角速度环 PID
#define PITANG_PID 41, 0.2, 60, 5, 200, -200         // IMU 闭环时的 Pitch 轴角度环 PID
#define YAWSPE_ENCOD_PID 200, 0, 0, 0, 28000, -28000 // 编码器闭环时的 Yaw 轴角速度环 PID
#define YAWANG_ENCOD_PID 4, 0, 60, 0, 800, -800      // 编码器闭环时的 Yaw 轴角度环 PID
#define PITSPE_ENCOD_PID 250, 0, 0, 0, 28000, -28000 // 编码器闭环时的 Pitch 轴角速度环 PID
#define PITANG_ENCOD_PID 20, 1, 25, 15, 300, -300    // 编码器闭环时的 Pitch 轴角度环 PID
// 发射机构相关设置
#define LAUNCH_MOTOR_ID 0x201                        // 播弹盘电机 ID
#define RUB_LEFT_MOTOR_ID 0x204                      // 左侧摩擦轮电机 ID
#define RUB_RIGHT_MOTOR_ID 0x203                     // 右侧摩擦轮电机 ID
#define GUN_RUB_TOGGLE (0)                           // 0 表示发射摩擦轮使用默认转动方向, 即左侧摩擦轮原始转速大于 0 时弹丸可以被发射出去
#define GUN_LAUNCH_TOGGLE (0)                        // 0 表示发射播弹盘使用默认转动方向, 即播弹盘原始转速大于 0 时弹丸可以被发射出去
#define GUN_SPEED_SET_15 3600                        // 实际设定弹速 15 时的摩擦轮转速设定值
#define GUN_SPEED_REAL_15 (14.4f)                    // SET15 对应的实际弹速
#define GUN_SPEED_SET_18 4150                        // 实际设定弹速 18 时的摩擦轮转速设定值
#define GUN_SPEED_REAL_18 (17.4f)                    // SET18 对应的实际弹速
#define GUN_SPEED_SET_30 5300                        // 实际设定弹速 30 时的摩擦轮转速设定值
#define GUN_SPEED_REAL_30 (21.5f)                    // SET30 对应的实际弹速
#define SET_BOOSTER_ADDRATIO (0.05f)                 // 拨弹叉每次运动到发射点之后继续转动的角度 单位：弹丸
#define LAUNCH_ANG_PID 10, 2, 40, 0.5, 500, -500     // 播弹盘电机角度环 PID
#define LAUNCH_SPE_PID 40, 10, 0, 600, 9000, -9000   // 播弹盘电机角速度环 PID
#define RUB_SPE_PID 20, 0.10, 30, 300, 14000, -14000 // 摩擦轮电机角速度环 PID
// 弹舱相关设置
#define MAGAZINE_SERVO_CLOSE_PULSE 1500              // 弹舱盖关闭时的脉宽
#define MAGAZINE_SERVO_OPEN_PULSE 980                // 弹舱盖从关闭到打开的脉宽行程
// 自瞄相关设置
#define CAMERA_PITCH_FIX (-4.85f)                    // 弹丸出射速度绝对水平时，相机的Pitch姿态角, 填写数字越大，瞄准位置越靠上
#define CAMERA_YAW_FIX (0.0f)                        // 枪口Yaw=0时，相机Yaw姿态角，填写数字越大，瞄准位置越靠左
#define IMU_PITCH_FIX (0.0f)                         // 弹丸出射速度绝对水平时，陀螺仪发回的Pitch角度 单位 ° 用于修正机械安装误差
#undef AIMBOT_CIMMUNICATION_USING_CAN                // 与视觉通信使用 串口 通信
/********************************* 玻头步兵 *********************************/
#elif defined CORE_USING_GLASS_INFANTRY
// IMU 坐标系相关设置
#define GYRO_X_SOURCE -GYRO_RAWDATA(y)
#define ACCL_X_SOURCE -ACCL_RAWDATA(y)
#define GYRO_Y_SOURCE +GYRO_RAWDATA(x)
#define ACCL_Y_SOURCE +ACCL_RAWDATA(x)
#define GYRO_Z_SOURCE +GYRO_RAWDATA(z)
#define ACCL_Z_SOURCE +ACCL_RAWDATA(z)
// 云台控制相关设置
#define PITCH_MOTOR_ID 0x206                            // Pitch 轴电机 ID
#define YAW_MOTOR_ID 0x205                              // Yaw 轴电机 ID
#undef CAR_USING_LINK                                   // Pitch 轴电机传动不采用连杆结构
#define GIMBAL_BIAS_SET (1300)                          // 云台重心电流补偿
#define YAW_ZERO_ANGLE 0                                // 正常跟随时 Yaw 电机编码器数值
#define PITCH_ZERO_ANGLE (4764)                         // Pitch 轴水平时 Pitch 电机编码器数值
#define PITCH_START_ANGLE 0                             // pitch上电后的初始角度设定值 单位°
#define PITCH_MOTOR_MIRROR 0                            // 0 表示 Pitch 使用默认转动方向, 即 Pitch 轴低头时电机编码器原始数据减小
#define YAW_MOTOR_MIRROR 0                              // 0 表示 Yaw 使用默认转动方向, 即 Yaw 轴向左旋转时电机编码器原始数据增大
#define PITCH_MIN_ANGLE (-19.0f)                        // pitch轴限幅 单位°
#define PITCH_MAX_ANGLE (25.0f)                         // pitch轴限幅 单位°
#define YAWSPE_PID 580, 2.5, 0, 1600, 25000, -25000     // IMU 闭环时的 Yaw 轴角速度环 PID
#define YAWANG_PID 43, 0.2, 105, 5, 600, -600           // IMU 闭环时的 Yaw 轴角度环 PID
#define PITSPE_PID 490, 0.6, 0, 3500, 25000, -25000     // IMU 闭环时的 Pitch 轴角速度环 PID
#define PITANG_PID 42, 0.2, 60, 5, 200, -200            // IMU 闭环时的 Pitch 轴角度环 PID
#define YAWSPE_ENCOD_PID 200, 0, 0, 0, 28000, -28000    // 编码器闭环时的 Yaw 轴角速度环 PID
#define YAWANG_ENCOD_PID 4, 0, 60, 0, 800, -800         // 编码器闭环时的 Yaw 轴角度环 PID
#define PITSPE_ENCOD_PID 300, 0, 0, 0, 28000, -28000    // 编码器闭环时的 Pitch 轴角速度环 PID
#define PITANG_ENCOD_PID 20, 1, 25, 15, 300, -300       // 编码器闭环时的 Pitch 轴角度环 PID
// 发射机构相关设置
#define LAUNCH_MOTOR_ID 0x201                           // 播弹盘电机 ID
#define RUB_LEFT_MOTOR_ID 0x204                         // 左侧摩擦轮电机 ID
#define RUB_RIGHT_MOTOR_ID 0x203                        // 右侧摩擦轮电机 ID
#define GUN_RUB_TOGGLE (1)                              // 0 表示发射摩擦轮使用默认转动方向, 即左侧摩擦轮原始转速大于 0 时弹丸可以被发射出去
#define GUN_LAUNCH_TOGGLE (0)                           // 0 表示发射播弹盘使用默认转动方向, 即播弹盘原始转速大于 0 时弹丸可以被发射出去
#define GUN_SPEED_SET_15 3730                           // 实际设定弹速 15 时的摩擦轮转速设定值
#define GUN_SPEED_REAL_15 (14.4f)                       // SET15 对应的实际弹速
#define GUN_SPEED_SET_18 4350                           // 实际设定弹速 18 时的摩擦轮转速设定值
#define GUN_SPEED_REAL_18 (17.5f)                       // SET18 对应的实际弹速
#define GUN_SPEED_SET_30 5800                           // 实际设定弹速 30 时的摩擦轮转速设定值
#define GUN_SPEED_REAL_30 (21.7f)                       // SET30 对应的实际弹速
#define SET_BOOSTER_ADDRATIO (0.01f)                    // 拨弹叉每次运动到发射点之后继续转动的角度 单位：弹丸
#define LAUNCH_ANG_PID 13, 1, 70, 1, 650, -650          // 播弹盘电机角度环 PID
#define LAUNCH_SPE_PID 160, 0.14, 300, 800, 9500, -9500 // 播弹盘电机角速度环 PID
#define RUB_SPE_PID 20, 0.10, 30, 300, 14000, -14000    // 摩擦轮电机角速度环 PID
// 弹舱相关设置
#define MAGAZINE_SERVO_CLOSE_PULSE 460                  // 弹舱盖关闭时的脉宽
#define MAGAZINE_SERVO_OPEN_PULSE 1230                  // 弹舱盖从关闭到打开的脉宽行程
// 自瞄相关设置
#define CAMERA_PITCH_FIX (-4.85f)                       // 弹丸出射速度绝对水平时，相机的Pitch姿态角, 填写数字越大，瞄准位置越靠上
#define CAMERA_YAW_FIX (0.0f)                           // 枪口Yaw=0时，相机Yaw姿态角，填写数字越大，瞄准位置越靠左
#define IMU_PITCH_FIX (0.0f)                            // 弹丸出射速度绝对水平时，陀螺仪发回的Pitch角度 单位 ° 用于修正机械安装误差
#undef AIMBOT_CIMMUNICATION_USING_CAN                   // 与视觉通信使用 串口 通信
#endif

/****************************************************************************
 *                        英雄机器人
 * *************************************************************************/
#elif defined CORE_USING_HERO
#define CAR_USING_LINK    // 英雄机器人 Pitch 全都使用连杆结构
#define SERVO_CTRL_EN (0) // 英雄机器人全部不需要电动弹舱
/********************************* 国赛老英雄 *********************************/
#if defined CORE_USING_NATION_HERO
// IMU 坐标系相关设置
#define GYRO_X_SOURCE -GYRO_RAWDATA(y)
#define ACCL_X_SOURCE -ACCL_RAWDATA(y)
#define GYRO_Y_SOURCE +GYRO_RAWDATA(x)
#define ACCL_Y_SOURCE +ACCL_RAWDATA(x)
#define GYRO_Z_SOURCE +GYRO_RAWDATA(z)
#define ACCL_Z_SOURCE +ACCL_RAWDATA(z)
// 云台控制相关设置
#define PITCH_MOTOR_ID 0x206                               // Pitch 轴电机 ID
#define YAW_MOTOR_ID 0x205                                 // Yaw 轴电机 ID
#define CONNECT_ANGLE 75                                   // 云台水平时连杆角度
#define PITCH_ZEROCURRENT_ANG (3.5f)                       // 当Pitch = 3.0f 时，达到机械平衡点
#define PTICH_F0_CURRENT (12000)                           // 重力补偿参数
#define YAW_ZERO_ANGLE 0x10A0                              // 正常跟随时 Yaw 电机编码器数值
#define PITCH_ZERO_ANGLE 4321                              // Pitch 轴水平时 Pitch 电机编码器数值
#define PITCH_START_ANGLE (0.f)                            // pitch上电后的初始角度设定值 单位°
#define PITCH_MOTOR_MIRROR 1                               // 0 表示 Pitch 使用默认转动方向, 即 Pitch 轴低头时电机编码器原始数据减小
#define YAW_MOTOR_MIRROR 1                                 // 0 表示 Yaw 使用默认转动方向, 即 Yaw 轴向左旋转时电机编码器原始数据增大
#define PITCH_MIN_ANGLE (-16.5f)                           // pitch轴限幅 单位°
#define PITCH_MAX_ANGLE (35.5f)                            // pitch轴限幅 单位°
#define YAWSPE_PID 400, 5, 0, 5000, 28000, -28000          // IMU 闭环时的 Yaw 轴角速度环 PID
#define YAWANG_PID 28, 0.3, 30, 0.2, 800, -800             // IMU 闭环时的 Yaw 轴角度环 PID
#define PITSPE_PID 350, 0, 0, 0, 28000, -28000             // IMU 闭环时的 Pitch 轴角速度环 PID
#define PITANG_PID 25, 0, 35, 0, 500, -500                 // IMU 闭环时的 Pitch 轴角度环 PID
#define YAWSPE_ENCOD_PID 450, 5, 0, 1500, 28000, -28000    // 编码器闭环时的 Yaw 轴角速度环 PID
#define YAWANG_ENCOD_PID 32, 1, 0, 10, 200, -200           // 编码器闭环时的 Yaw 轴角度环 PID
#define PITSPE_ENCOD_PID 450, 5, 0, 1000, 28000, -28000    // 编码器闭环时的 Pitch 轴角速度环 PID
#define PITANG_ENCOD_PID 30, 0, 0, 0, 150, -150            // 编码器闭环时的 Pitch 轴角度环 PID
#define PITSPE_DANGLING_PID 350, 5, 0, 5000, 28000, -28000 // 吊射模式下 Pitch 轴角速度环 PID
#define PITANG_DANGLING_PID 25, 0, 35, 0, 500, -500        // 吊射模式下 Pitch 轴角度环 PID
// 发射机构相关设置
#define LAUNCH_MOTOR_ID 0x208                              // 播弹盘电机 ID
#define RUB_LEFT_MOTOR_ID 0x201                            // 左侧摩擦轮电机 ID
#define RUB_RIGHT_MOTOR_ID 0x202                           // 右侧摩擦轮电机 ID
#define GUN_RUB_TOGGLE (1)                                 // 0 表示发射摩擦轮使用默认转动方向, 即左侧摩擦轮原始转速大于 0 时弹丸可以被发射出去
#define GUN_LAUNCH_TOGGLE (1)                              // 0 表示发射播弹盘使用默认转动方向, 即播弹盘原始转速大于 0 时弹丸可以被发射出去
#define GUN_SPEED_SET_10 3945                              // 实际设定弹速 10 时的摩擦轮转速设定值
#define GUN_SPEED_REAL_10 (9.1f)                           // SET10 对应的实际弹速
#define GUN_SPEED_SET_16 5050                              // 实际设定弹速 16 时的摩擦轮转速设定值
#define GUN_SPEED_REAL_16 (14.4f)                          // SET16 对应的实际弹速
#define SET_BOOSTER_ADDRATIO (0.32f)                       // 播弹叉对位完成以后向发弹方向转动的角度 单位：弹丸
#define LAUNCH_ANG_PID 12, 2, 20, 0.5, 80, -80             // 播弹盘电机角度环 PID
#define LAUNCH_SPE_PID 300, 0.1, 0, 3000, 16000, -16000    // 播弹盘电机角速度环 PID
#define RUB_SPE_PID 20, 0.10, 30, 300, 14000, -14000       // 摩擦轮电机角速度环 PID
// 自瞄相关设置
#define CAMERA_PITCH_FIX (-3.8f)                           // 弹丸出射速度绝对水平时，相机的Pitch姿态角, 填写数字越大，瞄准位置越靠上
#define CAMERA_YAW_FIX (0.0f)                              // 枪口Yaw=0时，相机Yaw姿态角，填写数字越大，瞄准位置越靠左
#define IMU_PITCH_FIX (0.0f)                               // 弹丸出射速度绝对水平时，陀螺仪发回的Pitch角度 单位 ° 用于修正机械安装误差
#define AIMBOT_CIMMUNICATION_USING_CAN                     // 与视觉通信使用 CAN 通信
/********************************* 双马尾英雄 *********************************/
#elif defined CORE_USING_BUNCHES_HERO
// IMU 坐标系相关设置
#define GYRO_X_SOURCE -GYRO_RAWDATA(y)
#define ACCL_X_SOURCE -ACCL_RAWDATA(y)
#define GYRO_Y_SOURCE +GYRO_RAWDATA(x)
#define ACCL_Y_SOURCE +ACCL_RAWDATA(x)
#define GYRO_Z_SOURCE +GYRO_RAWDATA(z)
#define ACCL_Z_SOURCE +ACCL_RAWDATA(z)
// 云台控制相关设置
#define PITCH_MOTOR_ID 0x206                               // Pitch 轴电机 ID
#define YAW_MOTOR_ID 0x205                                 // Yaw 轴电机 ID
#define CONNECT_ANGLE 80                                   // 云台水平时连杆角度
#define PITCH_ZEROCURRENT_ANG (-6.4f)                      // 当Pitch = 3.0f 时，达到机械平衡点
#define PTICH_F0_CURRENT (15000)                           // 重力补偿参数
#define YAW_ZERO_ANGLE 0x00AB                              // 正常跟随时 Yaw 电机编码器数值
#define PITCH_ZERO_ANGLE 0x1BF0                            // Pitch 轴水平时 Pitch 电机编码器数值
#define PITCH_START_ANGLE (0.f)                            // pitch上电后的初始角度设定值 单位°
#define PITCH_MOTOR_MIRROR 1                               // 0 表示 Pitch 使用默认转动方向, 即 Pitch 轴低头时电机编码器原始数据减小
#define YAW_MOTOR_MIRROR 1                                 // 0 表示 Yaw 使用默认转动方向, 即 Yaw 轴向左旋转时电机编码器原始数据增大
#define PITCH_MIN_ANGLE (-23.0f)                           // pitch轴限幅 单位°
#define PITCH_MAX_ANGLE (33.0f)                            // pitch轴限幅 单位°
#define YAWSPE_PID 400, 5, 0, 5000, 28000, -28000          // IMU 闭环时的 Yaw 轴角速度环 PID
#define YAWANG_PID 28, 0.3, 30, 0.2, 800, -800             // IMU 闭环时的 Yaw 轴角度环 PID
#define PITSPE_PID 400, 1, 0, 800, 28000, -28000           // IMU 闭环时的 Pitch 轴角速度环 PID
#define PITANG_PID 25, 1, 35, 0.5, 500, -500               // IMU 闭环时的 Pitch 轴角度环 PID
#define YAWSPE_ENCOD_PID 450, 1, 0, 200, 28000, -28000     // 编码器闭环时的 Yaw 轴角速度环 PID
#define YAWANG_ENCOD_PID 30, 1, 0, 10, 200, -200           // 编码器闭环时的 Yaw 轴角度环 PID
#define PITSPE_ENCOD_PID 450, 5, 0, 1000, 28000, -28000    // 编码器闭环时的 Pitch 轴角速度环 PID
#define PITANG_ENCOD_PID 30, 0, 0, 0, 150, -150            // 编码器闭环时的 Pitch 轴角度环 PID
#define PITSPE_DANGLING_PID 350, 5, 0, 5000, 28000, -28000 // 吊射模式下 Pitch 轴角速度环 PID
#define PITANG_DANGLING_PID 25, 0, 35, 0, 500, -500        // 吊射模式下 Pitch 轴角度环 PID
// 发射机构相关设置
#define LAUNCH_MOTOR_ID 0x208                              // 播弹盘电机 ID
#define RUB_LEFT_MOTOR_ID 0x202                            // 左侧摩擦轮电机 ID
#define RUB_RIGHT_MOTOR_ID 0x201                           // 右侧摩擦轮电机 ID
#define GUN_RUB_TOGGLE (0)                                 // 0 表示发射摩擦轮使用默认转动方向, 即左侧摩擦轮原始转速大于 0 时弹丸可以被发射出去
#define GUN_LAUNCH_TOGGLE (1)                              // 0 表示发射播弹盘使用默认转动方向, 即播弹盘原始转速大于 0 时弹丸可以被发射出去
#define GUN_SPEED_SET_10 2740                              // 实际设定弹速 10 时的摩擦轮转速设定值
#define GUN_SPEED_REAL_10 (9.4f)                           // SET10 对应的实际弹速
#define GUN_SPEED_SET_16 3480                              // 实际设定弹速 16 时的摩擦轮转速设定值
#define GUN_SPEED_REAL_16 (15.3f)                          // SET16 对应的实际弹速
#define SET_BOOSTER_ADDRATIO (0.48f)                       // 播弹叉对位完成以后向发弹方向转动的角度 单位：弹丸
#define LAUNCH_ANG_PID 12, 2, 20, 0.5, 80, -80             // 播弹盘电机角度环 PID
#define LAUNCH_SPE_PID 300, 0.1, 0, 3000, 16000, -16000    // 播弹盘电机角速度环 PID
#define RUB_SPE_PID 20, 0.10, 30, 300, 14000, -14000       // 摩擦轮电机角速度环 PID
// 自瞄相关设置
#define CAMERA_PITCH_FIX (-3.8f)                           // 弹丸出射速度绝对水平时，相机的Pitch姿态角, 填写数字越大，瞄准位置越靠上
#define CAMERA_YAW_FIX (0.0f)                              // 枪口Yaw=0时，相机Yaw姿态角，填写数字越大，瞄准位置越靠左
#define IMU_PITCH_FIX (0.0f)                               // 弹丸出射速度绝对水平时，陀螺仪发回的Pitch角度 单位 ° 用于修正机械安装误差
#undef AIMBOT_CIMMUNICATION_USING_CAN                      // 与视觉通信使用串口通信
#endif
#endif

#endif /* __ROBOSELECT_H__ */
