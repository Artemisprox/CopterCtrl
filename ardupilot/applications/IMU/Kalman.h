#ifndef __KALMAN_FORWARD_v_H__
#define __KALMAN_FORWARD_v_H__

#include "drv_utils.h"
typedef struct
{
    float v_input;
    float a_input;
    float last_a_input;
    float P_covariance;
    float Kalman_gain;
    float v_prior;
    float Q;
    float R;
    float v_observe;
    float period;
    char Inited;
} Kalman_Forward_v_one_dimension;

typedef struct
{
    float input[2];     // 0号位是高度,1号位是速度
    float a_input;
    float last_a_input;
    float P_covariance[2][2];
    float Kalman_gain[2][2];
    matrix Kalman_gain_m;
    float State_prior[2];
    float Q[2][2];
    float R[2][2];
    float State_observe[2];
    float period;
    char Inited;

    // 中间变量
    float temp_Kt_dividend[2][2]; // 计算卡尔曼增益的被除矩阵
    matrix temp_Kt_dividend_m;
    float temp_Kt_divide[2][2];
    matrix temp_Kt_divide_m;
    float temp_inverse[2][2];
    matrix temp_inverse_m;
} Kalman_Height_t;

// 一维卡尔曼滤波器计算，状态观测量只有一个速度
extern float Kalman_calculate(Kalman_Forward_v_one_dimension *Kalman, float v_now, float a_now);

// 初始化二阶卡尔曼
extern void Kalman_height_init(Kalman_Height_t *Kalman, float preiod, float Q1, float Q2, float R1, float R2);

// 一维卡尔曼滤波器初始化
extern void Kalman_one_dimension_init(Kalman_Forward_v_one_dimension *Kalman, float preiod, float Q, float R);

// 二阶卡尔曼滤波器初始化
extern void Kalman_height_calculate(Kalman_Height_t *Kalman, float height, float height_v, float height_a);

#endif
