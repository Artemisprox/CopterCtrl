#ifndef __FUNC_BMI088_H__
#define __FUNC_BMI088_H__

#include "func_ahrs.h"
#include <rtdef.h>

typedef struct
{
  AHRS_Accl_t Accl_Raw;
  AHRS_Gyro_t Gyro_Raw;
  float Temperature; // 温度

  int RawDataReady; // 数据是否有效
  float DataRate; // 中断接收数据的频率
} BMI088_t;

// IMU结构体，内含当前最新的六轴原始数据和温度数据
extern BMI088_t HERO_BMI088_DEV;

// BMI088设备初始化
// 初始化后可通过 HERO_BMI088_DEV 获取已有最新数据
// 调用 BMI088_WaitForRawData 可以挂起等待新数据产生
extern rt_err_t BMI088_Init(void);

// 调用函数后会挂起在信号量上，等待新的一组数据产生
extern void BMI088_WaitForRawData(void);

#endif

