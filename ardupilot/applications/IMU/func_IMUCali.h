#include "func_ahrs.h"
#include "func_IMU_redundancy.h"
// 是否等待温度达到设定值再开始零飘测定
#define GYROCALI_WAIT_FOR_TEMPERATURE 1

#define IMU1_BIAS_DATA_ADDR ((uint32_t)(0x08070000))
#define IMU1_CALI_FLAG_ADDR ((uint32_t)(IMU1_BIAS_DATA_ADDR + 4*5))
#define IMU2_BIAS_DATA_ADDR ((uint32_t)(IMU1_CALI_FLAG_ADDR + 1))
#define IMU2_CALI_FLAG_ADDR ((uint32_t)(IMU2_BIAS_DATA_ADDR + 4*5))

// 经过坐标换算和零飘校准后的数据
typedef struct
{
    AHRS_Accl_t AcclFix;
    AHRS_Gyro_t GyroFix;
} IMU_CaliData_t;

typedef enum
{
    Cali_Error,
    Cali_Start,
    Cali_Wait, // 延时一段时间，这样可以避免温度不稳定
    Cali_Recording,
    Cali_Processing,
    Cali_OK,
} IMU_GyroCali_State_e;


// 输入IMU原始数据，按照设置获取经安装位置校正和零飘校正后的六轴数据
extern void GetCaliIMUData(AHRS_Accl_t *AcclRaw, AHRS_Gyro_t *GyroRaw, AHRS_Accl_t *Accl, AHRS_Gyro_t *Gyro);

extern AHRS_Gyro_t IMU1_OffSet; // 默认为0
extern AHRS_Gyro_t IMU2_OffSet; // 默认为0

// 尝试从Flash中读取 若无数据或需要重测，则会自动重测，完成后函数返回
extern int LoadGyroOffSet(IMU_set_e IMU_set);
extern void AcclPosCorrect(AHRS_Accl_t *Accl , IMU_set_e IMU);
