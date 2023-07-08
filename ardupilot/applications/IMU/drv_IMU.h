#ifndef __DRV_GYRO_H__
#define __DRV_GYRO_H__

#include <rtdef.h>

#define IMU_STABLE_SET_MS (150)

typedef struct
{
	//IMU,大地坐标系
	float pitch;										/*pitch*///与安装方向有关
	float yaw;											/*yaw*/
	float roll;											/*roll*///与安装方向有关

	float pitch_speed;											/*PITCH轴角速度*/
	float roll_speed;											/*ROLL轴角速度*/
	float yaw_speed;											/*YAW轴角速度*/

	char speed_ready;										//角速度数据正常标志
	char atti_ready;										//姿态数据正常标志

}IMU_t;//IMU传感器结构体

typedef struct
{
	//云台电机系,
	float pitch;										/*pitch*/
	float yaw;											/*yaw*/
	float roll;											/*roll*/

	float pitch_speed;											/*PITCH轴角速度*/
	float roll_speed;											/*ROLL轴角速度*/
	float yaw_speed;											/*YAW轴角速度*/

	
}ATTI_t;//姿态结构体

extern void IMU_transfer2_gm(void);

extern rt_err_t IMU_WaitForInit(void); // 上电后等待陀螺仪启动完成

extern IMU_t HERO_IMU; //单位m/s^2,rad/s
extern ATTI_t gimbal_atti;//

extern rt_err_t IMU_GetAttiState(void);
extern rt_tick_t IMU_LastValid_tick;// 标记最近一次陀螺仪数据有效对应的时刻

/***
  * @Name     gyro_read_extern
  * @brief    陀螺仪姿态解算函数调用，类似于CAN接收，用于板载陀螺仪的兼容
  * @param	  float 单位：dps、degree
  * @author   ych
***/
extern void IMU_SetData_Extern(float PitchSpe,
					  float YawSpe,
					  float RollSpe,
					  float PitchAng,
					  float YawAng,
					  float RollAng,
					  int   AttiReady);
#endif
