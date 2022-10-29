#ifndef __HCHASSIS_DATA_H__
#define __HCHASSIS_DATA_H__

#include <rtthread.h>

#ifdef CORE_USING_GREENHEAD_INFANTRY //绿头步兵

//前后轮间距，单位mm
#define VEHICLE_LONG 325 //单位毫米
//左右轮间距，单位mm
#define VEHICLE_WIDTH 370 //单位毫米

//麦轮半径
#define WHEEL_RADIUS 70 //单位毫米
//云台电机零位
#define YAW_ZERO_ANGLE 8186
#define PITCH_ZERO_ANGLE 1444
// 云台电机反馈报文 ID
#define YAW_MOTOR_ID 0x205
#define PITCH_MOTOR_ID 0x206

//对yaw轴电机编码器角度值的变化方向进行矫正
#define YAW_ANGLE_REVERSAL RT_TRUE    //俯视图下顺时针转动云台，yaw轴电机编码器值从小变大，为RT_FALSE，反之为RT_TRUE
#define PITCH_ANGLE_REVERSAL RT_FALSE //枪口往上抬，pitch轴电机编码器值从小变大，为RT_FALSE，反之为RT_TRUE

// 底盘电机电调 ID
#define LEFT_FRONT_MOTOR_ID 0x202
#define RIGHT_FRONT_MOTOR_ID 0x201
#define LEFT_BACK_MOTOR_ID 0x203
#define RIGHT_BACK_MOTOR_ID 0x204

#endif

#ifdef CORE_USING_OMNI_WHEEL_INFANTRY //全向轮步兵

//全向轮旋转直径
#define VEHICLE_DIAMETER 350 //单位毫米
//全向轮半径
#define WHEEL_RADIUS 70 //单位毫米//待测
//云台电机零位
#define YAW_ZERO_ANGLE 4116
#define PITCH_ZERO_ANGLE 4116 //待测
// 云台电机反馈报文 ID
#define YAW_MOTOR_ID 0x205
#define PITCH_MOTOR_ID 0x206

//对yaw轴电机编码器角度值的变化方向进行矫正
#define YAW_ANGLE_REVERSAL RT_FALSE   //俯视图下顺时针转动云台，yaw轴电机编码器值从小变大，为RT_FALSE，反之为RT_TRUE
#define PITCH_ANGLE_REVERSAL RT_FALSE //枪口往上抬，pitch轴电机编码器值从小变大，为RT_FALSE，反之为RT_TRUE

// 底盘电机电调 ID
#define LEFT_FRONT_MOTOR_ID 0x202
#define RIGHT_FRONT_MOTOR_ID 0x201
#define LEFT_BACK_MOTOR_ID 0x203
#define RIGHT_BACK_MOTOR_ID 0x204

#endif

#ifdef CORE_USING_BLACKHEAD_INFANTRY //黑头步兵

//前后轮间距，单位mm
#define VEHICLE_LONG 325 //单位毫米
//左右轮间距，单位mm
#define VEHICLE_WIDTH 370 //单位毫米

//麦轮半径
#define WHEEL_RADIUS 70 //单位毫米
//云台电机零位
#define YAW_ZERO_ANGLE 4065
#define PITCH_ZERO_ANGLE 148
// 云台电机反馈报文 ID
#define YAW_MOTOR_ID 0x205
#define PITCH_MOTOR_ID 0x206

//对yaw轴电机编码器角度值的变化方向进行矫正
#define YAW_ANGLE_REVERSAL RT_TRUE   //
#define PITCH_ANGLE_REVERSAL RT_TRUE //枪口往上抬，pitch轴电机编码器值从小变大，为RT_FALSE，反之为RT_TRUE

// 底盘电机电调 ID
#define LEFT_FRONT_MOTOR_ID 0x202
#define RIGHT_FRONT_MOTOR_ID 0x201
#define LEFT_BACK_MOTOR_ID 0x203
#define RIGHT_BACK_MOTOR_ID 0x204

#endif

#ifdef CORE_USING_SMALL_INFANTRY //小步兵

//前后轮间距，单位mm
#define VEHICLE_LONG 238 //单位毫米
//左右轮间距，单位mm
#define VEHICLE_WIDTH 297 //单位毫米

//麦轮半径
#define WHEEL_RADIUS 57 //单位毫米
//云台电机零位
#define YAW_ZERO_ANGLE 7046
#define PITCH_ZERO_ANGLE 728
// 云台电机反馈报文 ID
#define YAW_MOTOR_ID 0x205
#define PITCH_MOTOR_ID 0x206

//对yaw轴电机编码器角度值的变化方向进行矫正
#define YAW_ANGLE_REVERSAL RT_TRUE   //俯视图下顺时针转动云台，yaw轴电机编码器值从小变大，为RT_FALSE，反之为RT_TRUE
#define PITCH_ANGLE_REVERSAL RT_TRUE //枪口往上抬，pitch轴电机编码器值从小变大，为RT_FALSE，反之为RT_TRUE

// 底盘电机电调 ID
#define LEFT_FRONT_MOTOR_ID 0x202
#define RIGHT_FRONT_MOTOR_ID 0x201
#define LEFT_BACK_MOTOR_ID 0x203
#define RIGHT_BACK_MOTOR_ID 0x204

#endif

#ifdef CORE_USING_CLOUD_INFANTRY1 //国赛祥云步兵1，先出来(碳板步兵)

//前后轮间距，单位mm
#define VEHICLE_LONG 390 //单位毫米
//左右轮间距，单位mm
#define VEHICLE_WIDTH 400 //单位毫米

//麦轮半径
#define WHEEL_RADIUS 70 //单位毫米
//云台电机零位
#define YAW_ZERO_ANGLE 0xFDA
#define PITCH_ZERO_ANGLE 7525
// 云台电机反馈报文 ID
#define YAW_MOTOR_ID 0x205
#define PITCH_MOTOR_ID 0x206

//对yaw轴电机编码器角度值的变化方向进行矫正，用于motor_speed_set函数
#define YAW_ANGLE_REVERSAL RT_TRUE    //俯视图下顺时针转动云台，yaw轴电机编码器值从小变大，为RT_FALSE，反之为RT_TRUE
#define PITCH_ANGLE_REVERSAL RT_FALSE //枪口往上抬，pitch轴电机编码器值从小变大，为RT_FALSE，反之为RT_TRUE

// 底盘电机电调 ID
#define LEFT_FRONT_MOTOR_ID 0x202
#define RIGHT_FRONT_MOTOR_ID 0x201
#define LEFT_BACK_MOTOR_ID 0x203
#define RIGHT_BACK_MOTOR_ID 0x204

#endif

#ifdef CORE_USING_CLOUD_INFANTRY2 //国赛祥云步兵2，后出来

//前后轮间距，单位mm
#define VEHICLE_LONG 390 //单位毫米
//左右轮间距，单位mm
#define VEHICLE_WIDTH 400 //单位毫米

//麦轮半径
#define WHEEL_RADIUS 70 //单位毫米
//云台电机零位
#define YAW_ZERO_ANGLE 0104
#define PITCH_ZERO_ANGLE 7513
// 云台电机反馈报文 ID
#define YAW_MOTOR_ID 0x205
#define PITCH_MOTOR_ID 0x206

//对yaw轴电机编码器角度值的变化方向进行矫正，用于motor_speed_set函数
#define YAW_ANGLE_REVERSAL RT_TRUE    //俯视图下顺时针转动云台，yaw轴电机编码器值从小变大，为RT_FALSE，反之为RT_TRUE
#define PITCH_ANGLE_REVERSAL RT_FALSE //枪口往上抬，pitch轴电机编码器值从小变大，为RT_FALSE，反之为RT_TRUE

// 底盘电机电调 ID
#define LEFT_FRONT_MOTOR_ID 0x202
#define RIGHT_FRONT_MOTOR_ID 0x201
#define LEFT_BACK_MOTOR_ID 0x203
#define RIGHT_BACK_MOTOR_ID 0x204

#endif

#ifdef CORE_USING_RICH_INFANTRY //富贵步兵

//前后轮间距，单位mm
#define VEHICLE_LONG 370 //单位毫米
//左右轮间距，单位mm
#define VEHICLE_WIDTH 410 //单位毫米

//麦轮半径
#define WHEEL_RADIUS 70 //单位毫米
//云台电机零位
#define YAW_ZERO_ANGLE 0xD4B
#define PITCH_ZERO_ANGLE 0xD25
// 云台电机反馈报文 ID
#define YAW_MOTOR_ID 0x208
#define PITCH_MOTOR_ID 0x206

//对yaw轴电机编码器角度值的变化方向进行矫正，用于motor_speed_set函数
#define YAW_ANGLE_REVERSAL RT_TRUE    //俯视图下顺时针转动云台，yaw轴电机编码器值从小变大，为RT_FALSE，反之为RT_TRUE
#define PITCH_ANGLE_REVERSAL RT_FALSE //枪口往上抬，pitch轴电机编码器值从小变大，为RT_FALSE，反之为RT_TRUE

// 底盘电机电调 ID
#define LEFT_FRONT_MOTOR_ID 0x204
#define RIGHT_FRONT_MOTOR_ID 0x202
#define LEFT_BACK_MOTOR_ID 0x201
#define RIGHT_BACK_MOTOR_ID 0x203

#endif

#ifdef CORE_USING_GLASS_INFANTRY //玻头步兵

//前后轮间距，单位mm
#define VEHICLE_LONG 390 //单位毫米
//左右轮间距，单位mm
#define VEHICLE_WIDTH 400 //单位毫米

//麦轮半径
#define WHEEL_RADIUS 70 //单位毫米
//云台电机零位
#define YAW_ZERO_ANGLE (0x0FDD)
#define PITCH_ZERO_ANGLE 0xD25
// 云台电机反馈报文 ID
#define YAW_MOTOR_ID 0x205
#define PITCH_MOTOR_ID 0x206

//对yaw轴电机编码器角度值的变化方向进行矫正，用于motor_speed_set函数
#define YAW_ANGLE_REVERSAL RT_TRUE    //俯视图下顺时针转动云台，yaw轴电机编码器值从小变大，为RT_FALSE，反之为RT_TRUE
#define PITCH_ANGLE_REVERSAL RT_FALSE //枪口往上抬，pitch轴电机编码器值从小变大，为RT_FALSE，反之为RT_TRUE

// 底盘电机电调 ID
#define LEFT_FRONT_MOTOR_ID 0x203
#define RIGHT_FRONT_MOTOR_ID 0x201
#define LEFT_BACK_MOTOR_ID 0x202
#define RIGHT_BACK_MOTOR_ID 0x204

#endif

#ifdef CORE_USING_BLACKHEAD_HERO //黑头英雄

//前后轮间距，单位mm
#define VEHICLE_LONG 421 //单位毫米
//左右轮间距，单位mm
#define VEHICLE_WIDTH 444 //单位毫米

//麦轮半径
#define WHEEL_RADIUS 76 //单位毫米
//云台电机零位
#define YAW_ZERO_ANGLE 5432
#define PITCH_ZERO_ANGLE 5374
// 云台电机反馈报文 ID
#define YAW_MOTOR_ID 0x205
#define PITCH_MOTOR_ID 0x206

//对yaw轴电机编码器角度值的变化方向进行矫正
#define YAW_ANGLE_REVERSAL RT_TRUE   //俯视图下顺时针转动云台，yaw轴电机编码器值从小变大，为RT_FALSE，反之为RT_TRUE
#define PITCH_ANGLE_REVERSAL RT_TRUE //枪口往上抬，pitch轴电机编码器值从小变大，为RT_FALSE，反之为RT_TRUE

// 底盘电机电调 ID
#define LEFT_FRONT_MOTOR_ID 0x202
#define RIGHT_FRONT_MOTOR_ID 0x201
#define LEFT_BACK_MOTOR_ID 0x203
#define RIGHT_BACK_MOTOR_ID 0x204

#endif

#ifdef CORE_USING_C_BOARD_HERO // c板英雄

//前后轮间距，单位mm
#define VEHICLE_LONG 472 //单位毫米
//左右轮间距，单位mm
#define VEHICLE_WIDTH 446 //单位毫米

//麦轮半径
#define WHEEL_RADIUS 75 //单位毫米
//云台电机零位
#define YAW_ZERO_ANGLE 0x05f0
#define PITCH_ZERO_ANGLE 4321
// 云台电机反馈报文 ID
#define YAW_MOTOR_ID 0x205
#define PITCH_MOTOR_ID 0x206

//对yaw轴电机编码器角度值的变化方向进行矫正
#define YAW_ANGLE_REVERSAL RT_FALSE  //俯视图下顺时针转动云台，yaw轴电机编码器值从小变大，为RT_FALSE，反之为RT_TRUE
#define PITCH_ANGLE_REVERSAL RT_TRUE //枪口往上抬，pitch轴电机编码器值从小变大，为RT_FALSE，反之为RT_TRUE

// 底盘电机电调 ID
#define LEFT_FRONT_MOTOR_ID 0x202
#define RIGHT_FRONT_MOTOR_ID 0x201
#define LEFT_BACK_MOTOR_ID 0x203
#define RIGHT_BACK_MOTOR_ID 0x204

#endif

#ifdef CORE_USING_NATION_HERO //国赛英雄

//前后轮间距，单位mm
#define VEHICLE_LONG 438 //单位毫米
//左右轮间距，单位mm
#define VEHICLE_WIDTH 445 //单位毫米

//麦轮半径
#define WHEEL_RADIUS 75 //单位毫米
//云台电机零位
#define YAW_ZERO_ANGLE 4256
#define PITCH_ZERO_ANGLE 4321 //待定//////////////////////////////////
// 云台电机反馈报文 ID
#define YAW_MOTOR_ID 0x205
#define PITCH_MOTOR_ID 0x206

//对yaw轴电机编码器角度值的变化方向进行矫正，用于motor_speed_set函数
#define YAW_ANGLE_REVERSAL RT_FALSE  //俯视图下顺时针转动云台，yaw轴电机编码器值从小变大，为RT_FALSE，反之为RT_TRUE
#define PITCH_ANGLE_REVERSAL RT_TRUE //待定，应该和c板英雄时一样的//////////////////////////////////

// 底盘电机电调 ID
#define LEFT_FRONT_MOTOR_ID 0x202
#define RIGHT_FRONT_MOTOR_ID 0x201
#define LEFT_BACK_MOTOR_ID 0x203
#define RIGHT_BACK_MOTOR_ID 0x204

#endif

#ifdef CORE_USING_BUNCHES_HERO //双马尾英雄

//前后轮间距，单位mm
#define VEHICLE_LONG 455 //单位毫米
//左右轮间距，单位mm
#define VEHICLE_WIDTH 410 //单位毫米

//麦轮半径
#define WHEEL_RADIUS 75 //单位毫米
//云台电机零位
#define YAW_ZERO_ANGLE 0x1BF0
#define PITCH_ZERO_ANGLE 4321 //待定//////////////////////////////////
// 云台电机反馈报文 ID
#define YAW_MOTOR_ID 0x205
#define PITCH_MOTOR_ID 0x206

//对yaw轴电机编码器角度值的变化方向进行矫正，用于motor_speed_set函数
#define YAW_ANGLE_REVERSAL RT_FALSE  //俯视图下顺时针转动云台，yaw轴电机编码器值从小变大，为RT_FALSE，反之为RT_TRUE
#define PITCH_ANGLE_REVERSAL RT_TRUE //待定，应该和c板英雄时一样的//////////////////////////////////

// 底盘电机电调 ID
#define LEFT_FRONT_MOTOR_ID 0x202
#define RIGHT_FRONT_MOTOR_ID 0x201
#define LEFT_BACK_MOTOR_ID 0x203
#define RIGHT_BACK_MOTOR_ID 0x204

#endif

#endif
