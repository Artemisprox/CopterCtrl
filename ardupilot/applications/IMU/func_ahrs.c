/*
        姿态解算-开源的AHRS算法
*/
#include "func_ahrs.h"
#include <rtdef.h>
#include <arm_math.h>

#define BETA_DEFUALT (0.033f)

AHRS_t HERO_AHRS;
AHRS_t HERO_AHRS_Gyro;
AHRS_Eulr_t HERO_Eulr;
AHRS_Eulr_t HERO_Eulr_Gyro;

/***
 * @brief 四阶龙格库塔法求解四元数微分方程
 * @param
 * @retval none
 * @author dxy
 ***/
static void AHRS_Runge_Kutta(const AHRS_t *q, AHRS_Quaternion_t *q_dot,
                             const AHRS_Gyro_t *gyro)
{
    static float gx_last = 0, gy_last = 0, gz_last = 0;
    float k0[4], k1[4], k2[4], k3[4];
    float q0_t, q1_t, q2_t, q3_t;

    q0_t = q->quat.q0;
    q1_t = q->quat.q1;
    q2_t = q->quat.q2;
    q3_t = q->quat.q3;
    k0[0] = 0.5f * (-q->quat.q1 * gx_last - q->quat.q2 * gy_last - q->quat.q3 * gz_last);
    k0[1] = 0.5f * (q->quat.q0 * gx_last + q->quat.q2 * gz_last - q->quat.q3 * gy_last);
    k0[2] = 0.5f * (q->quat.q0 * gy_last - q->quat.q1 * gz_last + q->quat.q3 * gx_last);
    k0[3] = 0.5f * (q->quat.q0 * gz_last + q->quat.q1 * gy_last - q->quat.q2 * gx_last);
    q0_t = q->quat.q0 + k0[0] * q->inv_sample_freq / 2;
    q1_t = q->quat.q1 + k0[1] * q->inv_sample_freq / 2;
    q2_t = q->quat.q2 + k0[2] * q->inv_sample_freq / 2;
    q3_t = q->quat.q3 + k0[3] * q->inv_sample_freq / 2;
    gx_last += (gyro->x - gx_last) / 2;
    gy_last += (gyro->y - gy_last) / 2;
    gz_last += (gyro->z - gz_last) / 2;
    k1[0] = 0.5f * (-q1_t * gx_last - q2_t * gy_last - q3_t * gz_last);
    k1[1] = 0.5f * (q0_t * gx_last + q2_t * gz_last - q3_t * gy_last);
    k1[2] = 0.5f * (q0_t * gy_last - q1_t * gz_last + q3_t * gx_last);
    k1[3] = 0.5f * (q0_t * gz_last + q1_t * gy_last - q2_t * gx_last);
    q0_t = q->quat.q0 + k1[0] * q->inv_sample_freq / 2;
    q1_t = q->quat.q1 + k1[1] * q->inv_sample_freq / 2;
    q2_t = q->quat.q2 + k1[2] * q->inv_sample_freq / 2;
    q3_t = q->quat.q3 + k1[3] * q->inv_sample_freq / 2;
    k2[0] = 0.5f * (-q1_t * gx_last - q2_t * gy_last - q3_t * gz_last);
    k2[1] = 0.5f * (q0_t * gx_last + q2_t * gz_last - q3_t * gy_last);
    k2[2] = 0.5f * (q0_t * gy_last - q1_t * gz_last + q3_t * gx_last);
    k2[3] = 0.5f * (q0_t * gz_last + q1_t * gy_last - q2_t * gx_last);
    q0_t = q->quat.q0 + k2[0] * q->inv_sample_freq;
    q1_t = q->quat.q1 + k2[1] * q->inv_sample_freq;
    q2_t = q->quat.q2 + k2[2] * q->inv_sample_freq;
    q3_t = q->quat.q3 + k2[3] * q->inv_sample_freq;
    k3[0] = 0.5f * (-q1_t * gyro->x - q2_t * gyro->y - q3_t * gyro->z);
    k3[1] = 0.5f * (q0_t * gyro->x + q2_t * gyro->z - q3_t * gyro->y);
    k3[2] = 0.5f * (q0_t * gyro->y - q1_t * gyro->z + q3_t * gyro->x);
    k3[3] = 0.5f * (q0_t * gyro->z + q1_t * gyro->y - q2_t * gyro->x);
    q_dot->q0 = (k0[0] + 2 * k1[0] + 2 * k2[0] + k3[0]) / 6;
    q_dot->q1 = (k0[1] + 2 * k1[1] + 2 * k2[1] + k3[1]) / 6;
    q_dot->q2 = (k0[2] + 2 * k1[2] + 2 * k2[2] + k3[2]) / 6;
    q_dot->q3 = (k0[3] + 2 * k1[3] + 2 * k2[3] + k3[3]) / 6;
    gx_last = gyro->x;
    gy_last = gyro->y;
    gz_last = gyro->z;
}

static int8_t Set_FromQuaternion(const AHRS_t *ahrs, AHRS_t *ahrs_gyro)
{
    if (ahrs == NULL)
        return -1;
    if (ahrs_gyro == NULL)
        return -1;
    ahrs_gyro->quat.q0 = ahrs->quat.q0;
    ahrs_gyro->quat.q1 = ahrs->quat.q1;
    ahrs_gyro->quat.q2 = ahrs->quat.q2;
    ahrs_gyro->quat.q3 = ahrs->quat.q3;
    return 0;
}

/**
 * @brief 纯陀螺仪数据解算姿态角

 * @param ahrs_gyro 陀螺仪数据得到的四元数
 * @param ahrs 融合算法得到的四元数
 * @param gyro 陀螺仪数据
 * @param ifClear 是否清除陀螺仪四元数
 * @return int8_t 0对应没有错误
*/
int8_t AHRS_UpdateGyro(AHRS_t *ahrs_gyro, AHRS_t *ahrs,
                       const AHRS_Gyro_t *gyro, int ifClear)
{
    if (ahrs_gyro == NULL)
        return -1;
    if (ahrs == NULL)
        return -1;
    if (gyro == NULL)
        return -1;
    if (ifClear == RT_EOK)
        Set_FromQuaternion(ahrs, ahrs_gyro);
    else if (ifClear == RT_ERROR)
    {
        float recip_norm;
        AHRS_Quaternion_t q_dot;

        // /* Rate of change of quaternion from gyroscope */
        AHRS_Runge_Kutta(ahrs_gyro, &q_dot, gyro);

        /* Integrate rate of change of quaternion to yield quaternion */
        ahrs_gyro->quat.q0 += q_dot.q0 * ahrs_gyro->inv_sample_freq;
        ahrs_gyro->quat.q1 += q_dot.q1 * ahrs_gyro->inv_sample_freq;
        ahrs_gyro->quat.q2 += q_dot.q2 * ahrs_gyro->inv_sample_freq;
        ahrs_gyro->quat.q3 += q_dot.q3 * ahrs_gyro->inv_sample_freq;

        /* Normalise quaternion */
        recip_norm = 1 / sqrtf(ahrs_gyro->quat.q0 * ahrs_gyro->quat.q0 +
                               ahrs_gyro->quat.q1 * ahrs_gyro->quat.q1 +
                               ahrs_gyro->quat.q2 * ahrs_gyro->quat.q2 +
                               ahrs_gyro->quat.q3 * ahrs_gyro->quat.q3);
        ahrs_gyro->quat.q0 *= recip_norm;
        ahrs_gyro->quat.q1 *= recip_norm;
        ahrs_gyro->quat.q2 *= recip_norm;
        ahrs_gyro->quat.q3 *= recip_norm;
    }
    else
        return -1;
    return 0;
}

/* 2 * proportional gain (Kp) */
static float beta = 0.010f;

float AHRS_GetBeta()
{
    return beta;
}

void AHRS_SetBeta(float beta_set)
{
    beta = beta_set;
}

/**
 * @brief 不使用磁力计计算姿态
 *
 * @param ahrs 姿态解算主结构体
 * @param accl 加速度计数据
 * @param gyro 陀螺仪数据
 * @return int8_t 0对应没有错误
 */
static int8_t AHRS_UpdateIMU(AHRS_t *ahrs, const AHRS_Accl_t *accl,
                             const AHRS_Gyro_t *gyro)
{
    if (ahrs == NULL)
        return -1;
    if (accl == NULL)
        return -1;
    if (gyro == NULL)
        return -1;

    float ax = accl->x;
    float ay = accl->y;
    float az = accl->z;

    float recip_norm;
    float s0, s1, s2, s3;
    AHRS_Quaternion_t q_dot;
    float _2q0, _2q1, _2q2, _2q3, _4q0, _4q1, _4q2, _8q1, _8q2, q0q0, q1q1, q2q2,
        q3q3;

    // /* Rate of change of quaternion from gyroscope */
    AHRS_Runge_Kutta(ahrs, &q_dot, gyro);
    // q_dot.q0 = 0.5f * (-ahrs->quat.q1 * gx - ahrs->quat.q2 * gy -
    //                  ahrs->quat.q3 * gz);
    // q_dot.q1 = 0.5f * (ahrs->quat.q0 * gx + ahrs->quat.q2 * gz -
    //                  ahrs->quat.q3 * gy);
    // q_dot.q2 = 0.5f * (ahrs->quat.q0 * gy - ahrs->quat.q1 * gz +
    //                  ahrs->quat.q3 * gx);
    // q_dot.q3 = 0.5f * (ahrs->quat.q0 * gz + ahrs->quat.q1 * gy -
    //                  ahrs->quat.q2 * gx);

    /* Compute feedback only if accelerometer measurement valid (avoids NaN in
     * accelerometer normalisation) */
    if (!((ax == 0.0f) && (ay == 0.0f) && (az == 0.0f)))
    {
        /* Normalise accelerometer measurement */
        recip_norm = 1 / sqrtf(ax * ax + ay * ay + az * az);
        ax *= recip_norm;
        ay *= recip_norm;
        az *= recip_norm;

        /* Auxiliary variables to avoid repeated arithmetic */
        _2q0 = 2.0f * ahrs->quat.q0;
        _2q1 = 2.0f * ahrs->quat.q1;
        _2q2 = 2.0f * ahrs->quat.q2;
        _2q3 = 2.0f * ahrs->quat.q3;
        _4q0 = 4.0f * ahrs->quat.q0;
        _4q1 = 4.0f * ahrs->quat.q1;
        _4q2 = 4.0f * ahrs->quat.q2;
        _8q1 = 8.0f * ahrs->quat.q1;
        _8q2 = 8.0f * ahrs->quat.q2;
        q0q0 = ahrs->quat.q0 * ahrs->quat.q0;
        q1q1 = ahrs->quat.q1 * ahrs->quat.q1;
        q2q2 = ahrs->quat.q2 * ahrs->quat.q2;
        q3q3 = ahrs->quat.q3 * ahrs->quat.q3;

        /* Gradient decent algorithm corrective step */
        s0 = _4q0 * q2q2 + _2q2 * ax + _4q0 * q1q1 - _2q1 * ay;
        s1 = _4q1 * q3q3 - _2q3 * ax + 4.0f * q0q0 * ahrs->quat.q1 -
             _2q0 * ay - _4q1 + _8q1 * q1q1 + _8q1 * q2q2 + _4q1 * az;
        s2 = 4.0f * q0q0 * ahrs->quat.q2 + _2q0 * ax + _4q2 * q3q3 -
             _2q3 * ay - _4q2 + _8q2 * q1q1 + _8q2 * q2q2 + _4q2 * az;
        s3 = 4.0f * q1q1 * ahrs->quat.q3 - _2q1 * ax +
             4.0f * q2q2 * ahrs->quat.q3 - _2q2 * ay;

        /* normalise step magnitude */
        recip_norm = 1 / sqrtf(s0 * s0 + s1 * s1 + s2 * s2 + s3 * s3);

        s0 *= recip_norm;
        s1 *= recip_norm;
        s2 *= recip_norm;
        s3 *= recip_norm;

        /* Apply feedback step */
        q_dot.q0 -= beta * s0;
        q_dot.q1 -= beta * s1;
        q_dot.q2 -= beta * s2;
        q_dot.q3 -= beta * s3;
    }

    /* Integrate rate of change of quaternion to yield quaternion */
    ahrs->quat.q0 += q_dot.q0 * ahrs->inv_sample_freq;
    ahrs->quat.q1 += q_dot.q1 * ahrs->inv_sample_freq;
    ahrs->quat.q2 += q_dot.q2 * ahrs->inv_sample_freq;
    ahrs->quat.q3 += q_dot.q3 * ahrs->inv_sample_freq;

    /* Normalise quaternion */
    recip_norm = 1 / sqrtf(ahrs->quat.q0 * ahrs->quat.q0 +
                           ahrs->quat.q1 * ahrs->quat.q1 +
                           ahrs->quat.q2 * ahrs->quat.q2 +
                           ahrs->quat.q3 * ahrs->quat.q3);
    ahrs->quat.q0 *= recip_norm;
    ahrs->quat.q1 *= recip_norm;
    ahrs->quat.q2 *= recip_norm;
    ahrs->quat.q3 *= recip_norm;

    return 0;
}

/**
 * @brief 初始化姿态解算
 *
 * @param ahrs 姿态解算主结构体
 * @param magn 磁力计数据
 * @param sample_freq 采样频率
 * @return int8_t 0对应没有错误
 */
int8_t AHRS_Init(AHRS_t *ahrs, const AHRS_Magn_t *magn, float sample_freq)
{
    if (ahrs == NULL)
        return -1;

    ahrs->inv_sample_freq = 1.0f / sample_freq;

    ahrs->quat.q0 = 1.0f;
    ahrs->quat.q1 = 0.0f;
    ahrs->quat.q2 = 0.0f;
    ahrs->quat.q3 = 0.0f;

    if (magn)
    {
        float yaw = -atan2(magn->y, magn->x);

        if ((magn->x == 0.0f) && (magn->y == 0.0f) && (magn->z == 0.0f))
        {
            ahrs->quat.q0 = 0.800884545f;
            ahrs->quat.q1 = 0.00862364192f;
            ahrs->quat.q2 = -0.00283267116f;
            ahrs->quat.q3 = 0.598749936f;
        }
        else if ((yaw < (PI / 2.0f)) || (yaw > 0.0f))
        {
            ahrs->quat.q0 = 0.997458339f;
            ahrs->quat.q1 = 0.000336312107f;
            ahrs->quat.q2 = -0.0057230792f;
            ahrs->quat.q3 = 0.0740156546;
        }
        else if ((yaw < PI) || (yaw > (PI / 2.0f)))
        {
            ahrs->quat.q0 = 0.800884545f;
            ahrs->quat.q1 = 0.00862364192f;
            ahrs->quat.q2 = -0.00283267116f;
            ahrs->quat.q3 = 0.598749936f;
        }
        else if ((yaw < 90.0f) || (yaw > PI))
        {
            ahrs->quat.q0 = 0.800884545f;
            ahrs->quat.q1 = 0.00862364192f;
            ahrs->quat.q2 = -0.00283267116f;
            ahrs->quat.q3 = 0.598749936f;
        }
        else if ((yaw < 90.0f) || (yaw > 0.0f))
        {
            ahrs->quat.q0 = 0.800884545f;
            ahrs->quat.q1 = 0.00862364192f;
            ahrs->quat.q2 = -0.00283267116f;
            ahrs->quat.q3 = 0.598749936f;
        }
    }
    return 0;
}

/**
 * @brief 姿态运算更新一次
 * @note 输入数据必须是NED(North East Down) 参考坐标系
 *
 * @param ahrs 姿态解算主结构体
 * @param accl 加速度计数据
 * @param gyro 陀螺仪数据
 * @param magn 磁力计数据
 * @return int8_t 0对应没有错误
 */
int8_t AHRS_Update(AHRS_t *ahrs, const AHRS_Accl_t *accl,
                   const AHRS_Gyro_t *gyro, const AHRS_Magn_t *magn)
{
    if (ahrs == NULL)
        return -1;
    if (accl == NULL)
        return -1;
    if (gyro == NULL)
        return -1;

    float recip_norm;
    float s0, s1, s2, s3;
    AHRS_Quaternion_t q_dot;
    float hx, hy;
    float _2q0mx, _2q0my, _2q0mz, _2q1mx, _2bx, _2bz, _4bx, _4bz, _2q0, _2q1,
        _2q2, _2q3, _2q0q2, _2q2q3, q0q0, q0q1, q0q2, q0q3, q1q1, q1q2, q1q3,
        q2q2, q2q3, q3q3;

    if (magn == NULL)
        return AHRS_UpdateIMU(ahrs, accl, gyro);

    float mx = magn->x;
    float my = magn->y;
    float mz = magn->z;

    /* Use IMU algorithm if magnetometer measurement invalid (avoids NaN in */
    /* magnetometer normalisation) */
    if ((mx == 0.0f) && (my == 0.0f) && (mz == 0.0f))
    {
        return AHRS_UpdateIMU(ahrs, accl, gyro);
    }

    float ax = accl->x;
    float ay = accl->y;
    float az = accl->z;

    /* Rate of change of quaternion from gyroscope */
    AHRS_Runge_Kutta(ahrs, &q_dot, gyro);
    // q_dot1 = 0.5f * (-ahrs->quat.q1 * gx - ahrs->quat.q2 * gy -
    //                  ahrs->quat.q3 * gz);
    // q_dot2 = 0.5f * (ahrs->quat.q0 * gx + ahrs->quat.q2 * gz -
    //                  ahrs->quat.q3 * gy);
    // q_dot3 = 0.5f * (ahrs->quat.q0 * gy - ahrs->quat.q1 * gz +
    //                  ahrs->quat.q3 * gx);
    // q_dot4 = 0.5f * (ahrs->quat.q0 * gz + ahrs->quat.q1 * gy -
    //                  ahrs->quat.q2 * gx);

    /* Compute feedback only if accelerometer measurement valid (avoids NaN in
     * accelerometer normalisation) */
    if (!((ax == 0.0f) && (ay == 0.0f) && (az == 0.0f)))
    {
        /* Normalise accelerometer measurement */
        recip_norm = 1 / sqrtf(ax * ax + ay * ay + az * az);
        ax *= recip_norm;
        ay *= recip_norm;
        az *= recip_norm;

        /* Normalise magnetometer measurement */
        recip_norm = 1 / sqrtf(mx * mx + my * my + mz * mz);
        mx *= recip_norm;
        my *= recip_norm;
        mz *= recip_norm;

        /* Auxiliary variables to avoid repeated arithmetic */
        _2q0mx = 2.0f * ahrs->quat.q0 * mx;
        _2q0my = 2.0f * ahrs->quat.q0 * my;
        _2q0mz = 2.0f * ahrs->quat.q0 * mz;
        _2q1mx = 2.0f * ahrs->quat.q1 * mx;
        _2q0 = 2.0f * ahrs->quat.q0;
        _2q1 = 2.0f * ahrs->quat.q1;
        _2q2 = 2.0f * ahrs->quat.q2;
        _2q3 = 2.0f * ahrs->quat.q3;
        _2q0q2 = 2.0f * ahrs->quat.q0 * ahrs->quat.q2;
        _2q2q3 = 2.0f * ahrs->quat.q2 * ahrs->quat.q3;
        q0q0 = ahrs->quat.q0 * ahrs->quat.q0;
        q0q1 = ahrs->quat.q0 * ahrs->quat.q1;
        q0q2 = ahrs->quat.q0 * ahrs->quat.q2;
        q0q3 = ahrs->quat.q0 * ahrs->quat.q3;
        q1q1 = ahrs->quat.q1 * ahrs->quat.q1;
        q1q2 = ahrs->quat.q1 * ahrs->quat.q2;
        q1q3 = ahrs->quat.q1 * ahrs->quat.q3;
        q2q2 = ahrs->quat.q2 * ahrs->quat.q2;
        q2q3 = ahrs->quat.q2 * ahrs->quat.q3;
        q3q3 = ahrs->quat.q3 * ahrs->quat.q3;

        /* Reference direction of Earth's magnetic field */
        hx = mx * q0q0 - _2q0my * ahrs->quat.q3 +
             _2q0mz * ahrs->quat.q2 + mx * q1q1 +
             _2q1 * my * ahrs->quat.q2 + _2q1 * mz * ahrs->quat.q3 -
             mx * q2q2 - mx * q3q3;
        hy = _2q0mx * ahrs->quat.q3 + my * q0q0 -
             _2q0mz * ahrs->quat.q1 + _2q1mx * ahrs->quat.q2 -
             my * q1q1 + my * q2q2 + _2q2 * mz * ahrs->quat.q3 - my * q3q3;
        _2bx = sqrtf(hx * hx + hy * hy);
        _2bz = -_2q0mx * ahrs->quat.q2 + _2q0my * ahrs->quat.q1 +
               mz * q0q0 + _2q1mx * ahrs->quat.q3 - mz * q1q1 +
               _2q2 * my * ahrs->quat.q3 - mz * q2q2 + mz * q3q3;
        _4bx = 2.0f * _2bx;
        _4bz = 2.0f * _2bz;

        /* Gradient decent algorithm corrective step */
        s0 = -_2q2 * (2.0f * q1q3 - _2q0q2 - ax) +
             _2q1 * (2.0f * q0q1 + _2q2q3 - ay) -
             _2bz * ahrs->quat.q2 *
                 (_2bx * (0.5f - q2q2 - q3q3) + _2bz * (q1q3 - q0q2) - mx) +
             (-_2bx * ahrs->quat.q3 + _2bz * ahrs->quat.q1) *
                 (_2bx * (q1q2 - q0q3) + _2bz * (q0q1 + q2q3) - my) +
             _2bx * ahrs->quat.q2 *
                 (_2bx * (q0q2 + q1q3) + _2bz * (0.5f - q1q1 - q2q2) - mz);
        s1 = _2q3 * (2.0f * q1q3 - _2q0q2 - ax) +
             _2q0 * (2.0f * q0q1 + _2q2q3 - ay) -
             4.0f * ahrs->quat.q1 * (1 - 2.0f * q1q1 - 2.0f * q2q2 - az) +
             _2bz * ahrs->quat.q3 *
                 (_2bx * (0.5f - q2q2 - q3q3) + _2bz * (q1q3 - q0q2) - mx) +
             (_2bx * ahrs->quat.q2 + _2bz * ahrs->quat.q0) *
                 (_2bx * (q1q2 - q0q3) + _2bz * (q0q1 + q2q3) - my) +
             (_2bx * ahrs->quat.q3 - _4bz * ahrs->quat.q1) *
                 (_2bx * (q0q2 + q1q3) + _2bz * (0.5f - q1q1 - q2q2) - mz);
        s2 = -_2q0 * (2.0f * q1q3 - _2q0q2 - ax) +
             _2q3 * (2.0f * q0q1 + _2q2q3 - ay) -
             4.0f * ahrs->quat.q2 * (1 - 2.0f * q1q1 - 2.0f * q2q2 - az) +
             (-_4bx * ahrs->quat.q2 - _2bz * ahrs->quat.q0) *
                 (_2bx * (0.5f - q2q2 - q3q3) + _2bz * (q1q3 - q0q2) - mx) +
             (_2bx * ahrs->quat.q1 + _2bz * ahrs->quat.q3) *
                 (_2bx * (q1q2 - q0q3) + _2bz * (q0q1 + q2q3) - my) +
             (_2bx * ahrs->quat.q0 - _4bz * ahrs->quat.q2) *
                 (_2bx * (q0q2 + q1q3) + _2bz * (0.5f - q1q1 - q2q2) - mz);
        s3 = _2q1 * (2.0f * q1q3 - _2q0q2 - ax) +
             _2q2 * (2.0f * q0q1 + _2q2q3 - ay) +
             (-_4bx * ahrs->quat.q3 + _2bz * ahrs->quat.q1) *
                 (_2bx * (0.5f - q2q2 - q3q3) + _2bz * (q1q3 - q0q2) - mx) +
             (-_2bx * ahrs->quat.q0 + _2bz * ahrs->quat.q2) *
                 (_2bx * (q1q2 - q0q3) + _2bz * (q0q1 + q2q3) - my) +
             _2bx * ahrs->quat.q1 *
                 (_2bx * (q0q2 + q1q3) + _2bz * (0.5f - q1q1 - q2q2) - mz);
        /* normalise step magnitude */
        recip_norm = 1 / sqrtf(s0 * s0 + s1 * s1 + s2 * s2 + s3 * s3);
        s0 *= recip_norm;
        s1 *= recip_norm;
        s2 *= recip_norm;
        s3 *= recip_norm;

        /* Apply feedback step */
        q_dot.q0 -= beta * s0;
        q_dot.q1 -= beta * s1;
        q_dot.q2 -= beta * s2;
        q_dot.q3 -= beta * s3;
    }

    /* Integrate rate of change of quaternion to yield quaternion */
    ahrs->quat.q0 += q_dot.q0 * ahrs->inv_sample_freq;
    ahrs->quat.q1 += q_dot.q1 * ahrs->inv_sample_freq;
    ahrs->quat.q2 += q_dot.q2 * ahrs->inv_sample_freq;
    ahrs->quat.q3 += q_dot.q3 * ahrs->inv_sample_freq;

    /* Normalise quaternion */
    recip_norm = 1 / sqrtf(ahrs->quat.q0 * ahrs->quat.q0 +
                           ahrs->quat.q1 * ahrs->quat.q1 +
                           ahrs->quat.q2 * ahrs->quat.q2 +
                           ahrs->quat.q3 * ahrs->quat.q3);
    ahrs->quat.q0 *= recip_norm;
    ahrs->quat.q1 *= recip_norm;
    ahrs->quat.q2 *= recip_norm;
    ahrs->quat.q3 *= recip_norm;

    return 0;
}

/**
 * @brief 通过姿态解算主结构体中的四元数计算欧拉角
 *
 * @param eulr 欧拉角
 * @param ahrs 姿态解算主结构体
 * @return int8_t 0对应没有错误
 */
float euler_test[3];
int8_t AHRS_GetEulr(AHRS_Eulr_t *eulr, const AHRS_t *ahrs)
{
    if (eulr == NULL)
        return -1;
    if (ahrs == NULL)
        return -1;

    const float sinr_cosp = 2.0f * (ahrs->quat.q0 * ahrs->quat.q1 +
                                    ahrs->quat.q2 * ahrs->quat.q3);
    const float cosr_cosp =
        1.0f - 2.0f * (ahrs->quat.q1 * ahrs->quat.q1 +
                       ahrs->quat.q2 * ahrs->quat.q2);
    eulr->rol = atan2f(sinr_cosp, cosr_cosp);

    const float sinp = 2.0f * (ahrs->quat.q0 * ahrs->quat.q2 -
                               ahrs->quat.q3 * ahrs->quat.q1);

    if (fabsf(sinp) >= 1.0f)
        eulr->pit = copysignf(PI / 2.0f, sinp);
    else
        eulr->pit = asinf(sinp);

    const float siny_cosp = 2.0f * (ahrs->quat.q0 * ahrs->quat.q3 +
                                    ahrs->quat.q1 * ahrs->quat.q2);
    const float cosy_cosp =
        1.0f - 2.0f * (ahrs->quat.q2 * ahrs->quat.q2 +
                       ahrs->quat.q3 * ahrs->quat.q3);
    eulr->yaw = atan2f(siny_cosp, cosy_cosp);

    euler_test[0] = eulr->yaw * 180.0f / PI;
    euler_test[1] = eulr->pit * 180.0f / PI;
    euler_test[2] = eulr->rol * 180.0f / PI;
#if 0
  eulr->yaw *= M_RAD2DEG_MULT;
  eulr->rol *= M_RAD2DEG_MULT;
  eulr->pit *= M_RAD2DEG_MULT;
#endif

    return 0;
}

/***
 * @brief
 * @param eulr: 欧拉角结构体
 * @param ahrs: 四元数结构体
 * @retval none
 * @author dxy
 ***/
void AHRS_Euler2Quarternion(const AHRS_Eulr_t *eulr, AHRS_t *ahrs)
{
    float sin_half_p, sin_half_r, sin_half_y = 0.0f;
    float cos_half_p, cos_half_r, cos_half_y = 0.0f;
    sin_half_p = sinf(0.5f * eulr->pit);
    sin_half_r = sinf(0.5f * eulr->rol);
    sin_half_y = sinf(0.5f * eulr->yaw);
    cos_half_p = cosf(0.5f * eulr->pit);
    cos_half_r = cosf(0.5f * eulr->rol);
    cos_half_y = cosf(0.5f * eulr->yaw);

    ahrs->quat.q0 = (cos_half_r * cos_half_p * cos_half_y) + (sin_half_r * sin_half_p * sin_half_y);
    ahrs->quat.q1 = (sin_half_r * cos_half_p * cos_half_y) - (cos_half_r * sin_half_p * sin_half_y);
    ahrs->quat.q2 = (cos_half_r * sin_half_p * cos_half_y) + (sin_half_r * cos_half_p * sin_half_y);
    ahrs->quat.q3 = (cos_half_r * cos_half_p * sin_half_y) - (sin_half_r * sin_half_p * cos_half_y);
}

/***
 * @brief 将yaw轴数据清零
 * @param eulr:欧拉角
 * @param ahrs:四元数
 * @retval none
 * @author dxy
 ***/
void AHRS_ResetYaw(AHRS_Eulr_t *eulr, AHRS_t *ahrs)
{
    eulr->yaw = 0;
    AHRS_Euler2Quarternion(eulr, ahrs);
}
