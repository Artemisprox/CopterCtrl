#include "Kalman.h"

// 一位卡尔曼滤波器计算，状态观测量只有一个速度
float Kalman_calculate(Kalman_Forward_v_one_dimension *Kalman,float v_now,float a_now)
{
    if (Kalman->Inited==0)
    {
        // 初始化的时候赋初值
        Kalman->Inited = 1;
        Kalman->last_a_input = a_now;
        Kalman->v_observe = v_now;
    }
    else 
    {
        //更新上一次的加速度
        Kalman->last_a_input = Kalman->a_input;
    }
    // 填写输入
    Kalman->a_input = a_now;
    Kalman->v_input = v_now;
    // 计算先验的速度估计值
    Kalman->v_prior = Kalman->v_observe + Kalman->period * Kalman->last_a_input;
    // 更新协方差
    Kalman->P_covariance = Kalman->P_covariance+Kalman->Q;
    // 计算卡尔曼增益
    Kalman->Kalman_gain = Kalman->P_covariance / (Kalman->P_covariance + Kalman->R);
    // 最终观测出来的速度
    Kalman->v_observe = Kalman->v_prior + Kalman->Kalman_gain * (Kalman->v_input - Kalman->v_prior);
    // 更新协方差
    Kalman->P_covariance = (1 - Kalman->Kalman_gain) * Kalman->P_covariance;
    return Kalman->v_observe;
}

// 方便协方差计算打代码做的替换
#define P1 (Kalman->P_covariance[0][0])
#define P2 (Kalman->P_covariance[0][1])
#define P3 (Kalman->P_covariance[1][0])
#define P4 (Kalman->P_covariance[1][1])

#define Kt1 (Kalman->Kalman_gain[0][0])
#define Kt2 (Kalman->Kalman_gain[0][1])
#define Kt3 (Kalman->Kalman_gain[1][0])
#define Kt4 (Kalman->Kalman_gain[1][1])

//二维卡尔曼滤波，状态观测量为速度和距离
void Kalman_height_calculate(Kalman_Height_t*Kalman,float height,float height_v,float height_a)
{
    if (Kalman->Inited == 0)
    {
        // 初始化的时候赋初值
        Kalman->Inited = 1;
        Kalman->last_a_input = height_a;
        // 赋初始状态
        Kalman->State_observe[0] = height;
        Kalman->State_observe[1] = height_v;
    }
    else
    {
        // 更新上一次的加速度
        Kalman->last_a_input = Kalman->a_input;
    }
    // 输入填入
    Kalman->a_input = height_a;
    Kalman->input[0] = height;
    Kalman->input[1] = height_v;
    // 先验估计
    Kalman->State_prior[0] = Kalman->State_observe[0] 
                            + Kalman->State_observe[1] * Kalman->period 
                            + 0.5f * Kalman->period * Kalman->period * Kalman->last_a_input;
    Kalman->State_prior[1] = Kalman->State_observe[1] + Kalman->last_a_input * Kalman->period;
    // 协方差计算
    P1 = P1 + (P2 + P3) * Kalman->period + P4 * Kalman->period * Kalman->period + Kalman->Q[0][0];
    P2 = P2 + P4 * Kalman->period + Kalman->Q[0][1];
    P3 = P3 + P4 * Kalman->period + Kalman->Q[1][0];
    P4 = P4 + Kalman->Q[1][1];
    // 计算卡尔曼增益
    Kalman->temp_Kt_dividend[0][0] = P1;
    Kalman->temp_Kt_dividend[0][1] = P2;
    Kalman->temp_Kt_dividend[1][0] = P3;
    Kalman->temp_Kt_dividend[1][1] = P4;

    Kalman->temp_Kt_divide[0][0] = P1 + Kalman->R[0][0];
    Kalman->temp_Kt_divide[0][1] = P2 + Kalman->R[0][1];
    Kalman->temp_Kt_divide[1][0] = P3 + Kalman->R[1][0];
    Kalman->temp_Kt_divide[1][1] = P4 + Kalman->R[1][1];
    // 矩阵除法求卡尔曼增益
    matrix_inverse(&Kalman->temp_Kt_divide_m, &Kalman->temp_inverse_m);
    matrix_multiply(&Kalman->temp_Kt_dividend_m, &Kalman->temp_inverse_m, &Kalman->Kalman_gain_m);
    // 求最优估计值
    float temp_height = Kalman->input[0] - Kalman->State_prior[0];
    float temp_height_v = Kalman->input[1] - Kalman->State_prior[1];
    Kalman->State_observe[0] = Kalman->State_prior[0] + Kt1 * temp_height + Kt2 * temp_height_v;
    Kalman->State_observe[1] = Kalman->State_prior[1] + Kt3 * temp_height + Kt4 * temp_height_v;
    // 更新协方差矩阵
    float temp_p[2][2];
    temp_p[0][0] = P1 * (1 - Kt1) + P3 * (0 - Kt2);
    temp_p[0][1] = P2 * (1 - Kt1) + P4 * (0 - Kt2);
    temp_p[1][0] = P1 * (0 - Kt3) + P3 * (1 - Kt4);
    temp_p[1][1] = P2 * (0 - Kt3) + P4 * (1 - Kt4);
    P1 = temp_p[0][0];
    P2 = temp_p[0][1];
    P3 = temp_p[1][0];
    P4 = temp_p[1][1];
    return;
}

// 一维卡尔曼滤波器初始化
void Kalman_one_dimension_init(Kalman_Forward_v_one_dimension*Kalman,float preiod,float Q,float R)
{
    Kalman->period = preiod;
    Kalman->Q = Q;
    Kalman->R = R;
}

// 二阶卡尔曼滤波器初始化
void Kalman_height_init(Kalman_Height_t *Kalman, float preiod, float Q1,float Q2, float R1,float R2)
{
    Kalman->period = preiod;
    matrix_init(&Kalman->temp_Kt_dividend_m, 2, 2, &Kalman->temp_Kt_dividend[0][0]);
    matrix_init(&Kalman->temp_Kt_divide_m, 2, 2, &Kalman->temp_Kt_divide[0][0]);
    matrix_init(&Kalman->Kalman_gain_m, 2, 2, &Kalman->Kalman_gain[0][0]);
    matrix_init(&Kalman->temp_inverse_m, 2, 2, &Kalman->temp_inverse[0][0]);
    Kalman->Q[0][0] = Q1;
    Kalman->Q[0][1] = 0;
    Kalman->Q[1][0] = 0;
    Kalman->Q[1][1] = Q2;
    Kalman->R[0][0] = R1;
    Kalman->R[0][1] = 0;
    Kalman->R[1][0] = 0;
    Kalman->R[1][1] = R2;
    Kalman->P_covariance[0][0] = 0.f;
    Kalman->P_covariance[0][1] = 0.f;
    Kalman->P_covariance[1][0] = 0.f;
    Kalman->P_covariance[1][1] = 0.f;
}
