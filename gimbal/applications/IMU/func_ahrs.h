#ifndef __FUNC_AHRS_H__
#define __FUNC_AHRS_H__

/*
  开源的AHRS算法。
  MadgwickAHRS
*/
#include <stdint.h>

/* 欧拉角（Euler angle） */
typedef struct {
  float yaw; /* 偏航角（Yaw angle） */
  float pit; /* 俯仰角（Pitch angle） */
  float rol; /* 翻滚角（Roll angle） */
} AHRS_Eulr_t;

/* 加速度计 Accelerometer */
typedef struct {
  float x;
  float y;
  float z;
} AHRS_Accl_t;

/* 陀螺仪 Gyroscope */
typedef struct {
  float x;
  float y;
  float z;
} AHRS_Gyro_t;

/* 磁力计 Magnetometer */
typedef struct {
  float x;
  float y;
  float z;
} AHRS_Magn_t;

/* 四元数 */
typedef struct {
  float q0;
  float q1;
  float q2;
  float q3;
} AHRS_Quaternion_t;

/* 姿态解算算法主结构体 */
typedef struct {
  /* 四元数 */
  AHRS_Quaternion_t quat;

  float inv_sample_freq; /* 采样频率的的倒数 */
} AHRS_t;

extern AHRS_t HERO_AHRS;
extern AHRS_t HERO_AHRS_Gyro;
extern AHRS_Eulr_t HERO_Eulr;
extern AHRS_Eulr_t HERO_Eulr_Gyro;

/**
 * @brief 初始化姿态解算
 *
 * @param ahrs 姿态解算主结构体
 * @param magn 磁力计数据
 * @param sample_freq 采样频率
 * @return int8_t 0对应没有错误
 */
int8_t AHRS_Init(AHRS_t *ahrs, const AHRS_Magn_t *magn, float sample_freq);

/**
 * @brief 姿态运算更新一次
 *
 * @param ahrs 姿态解算主结构体
 * @param accl 加速度计数据
 * @param gyro 陀螺仪数据
 * @param magn 磁力计数据
 * @return int8_t 0对应没有错误
 */
int8_t AHRS_Update(AHRS_t *ahrs, const AHRS_Accl_t *accl,
                   const AHRS_Gyro_t *gyro, const AHRS_Magn_t *magn);

/**
 * @brief 通过姿态解算主结构体中的四元数计算欧拉角
 *
 * @param eulr 欧拉角
 * @param ahrs 姿态解算主结构体
 * @return int8_t 0对应没有错误
 */
int8_t AHRS_GetEulr(AHRS_Eulr_t *eulr, const AHRS_t *ahrs);



/***
* @brief 欧拉角转四元数,yaw->pitch->roll顺规
* @param eulr: 欧拉角结构体
* @param ahrs: 四元数结构体
* @retval none
* @author dxy
***/
void AHRS_Euler2Quarternion(const AHRS_Eulr_t *eulr, AHRS_t *ahrs);


/**
 * \brief 将对应数据置零
 *
 * \param eulr 欧拉角结构体
 * \param ahrs 四元数结构体
 */
void AHRS_ResetYaw(AHRS_Eulr_t *eulr, AHRS_t *ahrs);

/**
 * @brief 纯陀螺仪数据解算姿态角
 * @param ahrs_gyro 陀螺仪数据得到的四元数
 * @param ahrs 融合算法得到的四元数
 * @param gyro 陀螺仪数据
 * @param ifClear 是否清除陀螺仪四元数
 * @return int8_t 0对应没有错误
 */
int8_t AHRS_UpdateGyro(AHRS_t *ahrs_gyro, AHRS_t *ahrs,
                       const AHRS_Gyro_t *gyro, int ifClear);

extern void AHRS_SetBeta(float beta_set);

extern float AHRS_GetBeta(void);
#endif

