#ifndef __FUNC_SENSORRAW_H__
#define __FUNC_SENSORRAW_H__

#include "rtthread.h"
#include "rtdevice.h"
#include "board.h"
#include "func_ahrs.h"
#include "drv_flash.h"

typedef struct
{
  AHRS_Accl_t Accl_Raw;
  AHRS_Gyro_t Gyro_Raw;
  float Temperature; // 温度

  int DataFreshtime;
  int RawDataReady; // 数据是否有效
  float DataRate; // 中断接收数据的频率
} Sensor_RAW_t;

// Sensor结构体，内含当前最新的六轴原始数据和温度数据
extern Sensor_RAW_t Sensor_RAW_IMU1;
extern Sensor_RAW_t Sensor_RAW_IMU2;

// 传感器初始化
// 初始化后可通过 Sensor_RAW 获取已有最新数据
// 调用 Sensor_WaitForRawData 可以挂起等待新数据产生
extern int SensorRawProcess_Init(void);

// 调用函数后会挂起在信号量上，等待新的一组数据产生
extern void Sensor_WaitFor_IMU1_RawData(void);
extern void Sensor_WaitFor_IMU2_RawData(void);
#endif
