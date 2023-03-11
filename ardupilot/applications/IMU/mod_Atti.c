#include "mod_Atti.h"

#include "drv_thread.h"
#include "drv_HWTimer.h"
#include "func_bmi088.h"
#include "func_ahrs.h"
#include "func_IMUCali.h"
#include "func_TempCtr.h"
#include "mod_Monitor.h"
#include "drv_utils.h"
#include "func_EKF.h"
#include "drv_IMU.h"

static rt_thread_t atti_calcu = RT_NULL;

/***
 * @brief 欧拉角初始化,yaw->pitch->roll顺规
 * @param none
 * @retval none
 * @author dxy
 ***/
void Atti_FirstUpdate(AHRS_Accl_t *Accl)
{
    AHRS_Eulr_t first_euler = {0};
    float ax, ay, az;

    ax = Accl->x;
    ay = Accl->y;
    az = Accl->z;

    //初始姿态(欧拉角)
    first_euler.pit = -atan2f(ax, sqrtf(az * az + ay * ay));
    first_euler.rol = atan2f(ay, az);
    first_euler.yaw = 0.0f;

    //欧拉角转四元数
    AHRS_Euler2Quarternion(&first_euler, &HERO_AHRS);
}

#define ACCL_STATIC 9.8f // 定义正常模长
#define BETA_MAX 0.006f
#define BETA_MIN 0.001f
#define BETA_FIX_K 0.002f // ACCL_ERROR为 1 m/(s^2) 时，BETA减小的量

static float Accl_Len = ACCL_STATIC, Accl_Filter;
static float Accl_Error;

static volatile float Accl_BetaFix_EN = 1;

static void Fresh_Beta(void)
{
    float beta_cal;
    float Accl_Len_2;
    float Beta_Filter;

    // 计算加速度矢量模长
    Accl_Len_2 = SQUARE(HERO_BMI088_DEV.Accl_Raw.x) + SQUARE(HERO_BMI088_DEV.Accl_Raw.y) + SQUARE(HERO_BMI088_DEV.Accl_Raw.z);
    if (Accl_Len_2 < 0)
        Accl_Len_2 = 0;
    Accl_Filter = 10 * sqrtf(Accl_Len_2);
    if (isnan(Accl_Len))
        Accl_Len = Accl_Filter;
    Accl_Len = Accl_Filter * 0.01f + Accl_Len * 0.99f; // 截止频率1.6Hz

    Accl_Error = fabsf(Accl_Len - ACCL_STATIC);
    if (Accl_Error < 0.3f)
        // 误差小于一定值时，认为当前加速度计数据完全没有问题
        Accl_Error = 0;

    Beta_Filter = BETA_MAX - Accl_Error * BETA_FIX_K;
    if (Beta_Filter < BETA_MIN)
        Beta_Filter = BETA_MIN;
    beta_cal = AHRS_GetBeta();
    beta_cal = beta_cal * 0.95f + Beta_Filter * 0.05f;
    if (Accl_BetaFix_EN)
        AHRS_SetBeta(beta_cal);
}

/**
 * @brief：姿态解算线程
 * @param [in]	parameter:该参数不会被使用
 * @return：		无
 * @author：zzj
 */
static void AttiCalcu_thread(void *parameter)
{
    int count, AttiReady_Flag;
    count = 0;
    rt_uint8_t first_flag = 1;
    int FirstCount = 100; // 前100次计算时，对加速度计数据进行积分来确定初始角度
    float inv_sample_freq;
	
    AHRS_Accl_t AcclFix;       // 经过坐标变换后的加速度计数据
    AHRS_Accl_t AcclSum = {0}; // 启动时
    AHRS_Gyro_t GyroFix;       // 经过坐标变换和零飘校正后的角速度数据
//    SWDG_START(SWDG_IMU_ID);		
		
    while (1)
    {
        BMI088_WaitForRawData();

        // 坐标换算，零飘校正
        GetCaliIMUData(&HERO_BMI088_DEV.Accl_Raw, &HERO_BMI088_DEV.Gyro_Raw, &AcclFix, &GyroFix);

        if (first_flag)
        { // 初始位置还没确定
            FirstCount--;
            if (FirstCount > 0)
            {
                AcclSum.x += AcclFix.x;
                AcclSum.y += AcclFix.y;
                AcclSum.z += AcclFix.z;
//                SWDG_FEED(SWDG_IMU_ID);
                continue;
            }
            else
            {
                Atti_FirstUpdate(&AcclSum);
                first_flag = 0;
            }
            AttiReady_Flag = 0;
        }
        else
        { // 初始位置确定完成
            inv_sample_freq = 1 / HERO_BMI088_DEV.DataRate;
            Fresh_Beta();

            HERO_AHRS.inv_sample_freq = inv_sample_freq;

            AHRS_Update(&HERO_AHRS, &AcclFix, &GyroFix, NULL);

            AHRS_GetEulr(&HERO_Eulr, &HERO_AHRS);
					
            if (count <= 50)
            {
                count++;
                AttiReady_Flag = 0;
            }
            else
                AttiReady_Flag = 1;
        }
        // 刷新姿态角数据
        IMU_SetData_Extern(GyroFix.y, GyroFix.z, GyroFix.x, HERO_Eulr.pit, HERO_Eulr.yaw, HERO_Eulr.rol, AttiReady_Flag);
//        SWDG_FEED(SWDG_IMU_ID);
    }
}

/**
* @brief：初始化姿态解算线程
* @param [in]	无
* @return：		1:初始化成功
                0:初始化失败
* @author：zzj
*/
int Atti_init(void)
{
    AHRS_Init(&HERO_AHRS, NULL, 1000);

    TempCTR_init();
    // 尝试从Flash中读取零飘数据 若无数据或需要重测，则会自动重测，完成后函数返回
    LoadGyroOffSet();

    //初始化姿态解算线程
    atti_calcu = rt_thread_create(
        "AT_CTRL",             //线程名
        AttiCalcu_thread,      //线程入口
        RT_NULL,               //入口参数无
        4096,                  //线程栈
        THREAD_PRIO_ATTICALCU, //线程优先级
        2);                    //线程时间片大小

    //线程创建失败返回false
    if (atti_calcu == RT_NULL)
        return RT_ERROR;

    //线程启动失败返回false
    if (rt_thread_startup(atti_calcu) != RT_EOK)
        return RT_ERROR;

    return RT_EOK;
}
