#include "mod_Atti.h"
#include "drv_dataserve.h"
#include "drv_thread.h"
#include "drv_HWTimer.h"
#include "func_SensorRAW.h"
#include "func_IMUCali.h"
#include "func_TempCtr.h"
#include "mod_Monitor.h"
#include "drv_utils.h"
#include "drv_IMU.h"
#include "velocity_estimator.h"
#include "INS_FLOW.h"

static rt_thread_t atti_calcu = RT_NULL;
static rt_int8_t Package_ID;
static AHRS_Eulr_t first_euler = {0};
extern Sensor_RAW_t copter_IMU_RAW;

#define LOW_K 0.6f 

/***
 * @brief 欧拉角初始化,yaw->pitch->roll顺规
 * @param none
 * @retval none
 * @author dxy
 ***/
void Atti_FirstUpdate(AHRS_Accl_t *Accl)
{
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

void AcclInstallCorrect(AHRS_Accl_t *Accl)
{
    float ax, ay, az;

    ax = Accl->x;
    ay = Accl->y;
    az = Accl->z;

    Accl->x = ax + arm_sin_f32(first_euler.pit)/arm_cos_f32(first_euler.pit)*arm_sin_f32(first_euler.rol)*ay + arm_sin_f32(first_euler.pit)/arm_cos_f32(first_euler.pit)*arm_cos_f32(first_euler.rol)*az ;
    Accl->y = arm_cos_f32(first_euler.rol)*ay - arm_sin_f32(first_euler.rol)*az;
    Accl->z = arm_sin_f32(first_euler.rol)/arm_cos_f32(first_euler.pit)*ay + arm_cos_f32(first_euler.rol)/arm_cos_f32(first_euler.pit)*ay ; 

}

#define ACCL_STATIC 9.8f // 定义正常模长
#define BETA_MAX 0.006f
#define BETA_MIN 0.001f
#define BETA_FIX_K 0.002f // ACCL_ERROR为 1 m/(s^2) 时，BETA减小的量

static float Accl_Len = ACCL_STATIC, Accl_Filter;
static float Accl_Error;

static volatile float Accl_BetaFix_EN = 1;
float test_g=9.8f;

static void Fresh_Beta(void)
{
    float beta_cal;
    float Accl_Len_2;
    float Beta_Filter;

    // 计算加速度矢量模长
    Accl_Len_2 = SQUARE(copter_IMU_RAW.Accl_Raw.x) + SQUARE(copter_IMU_RAW.Accl_Raw.y) + SQUARE(copter_IMU_RAW.Accl_Raw.z);
    if (Accl_Len_2 < 0)
        Accl_Len_2 = 0;
    Accl_Filter = sqrtf(Accl_Len_2);//此处改动
    if (isnan(Accl_Len))
        Accl_Len = Accl_Filter;
    Accl_Len = Accl_Filter * 0.01f + Accl_Len * 0.99f; // 截止频率1.6Hz

    Accl_Error = fabsf(Accl_Len - ACCL_STATIC);
    if (Accl_Error < 0.3f)
        // 误差小于一定值时，认为当前加速度计数据完全没有问题
        Accl_Error = 0;
		test_g = Accl_Len;
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
        IMU_WaitForRawData();

        // 坐标换算，零飘校正
        GetCaliIMUData(&copter_IMU_RAW.Accl_Raw, &copter_IMU_RAW.Gyro_Raw, &AcclFix, &GyroFix);
				float datarate = copter_IMU_RAW.DataRate;
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
        {
					
//							ins_flow_data.flo[0] = AcclFix.x;
//							ins_flow_data.flo[1] = AcclFix.y;
//							ins_flow_data.flo[2] = AcclFix.z;
//							ins_flow_data.flo[3] = GyroFix.x;
//							ins_flow_data.flo[4] = GyroFix.y;
//							ins_flow_data.flo[5] = GyroFix.z;
					
             // 初始位置确定完成
            inv_sample_freq = 1 / copter_IMU_RAW.DataRate;
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
        IMU_SetData_Extern(GyroFix.y, GyroFix.z, GyroFix.x, HERO_Eulr.pit, HERO_Eulr.yaw, HERO_Eulr.rol, AttiReady_Flag && copter_IMU_RAW.RawDataReady);

        //刷新高度估计数据
        accl_vec_estimator(HERO_IMU , AcclFix , datarate);
				
        /*数据服务器写入*/
        IMU_t *p =  Package_Pionter_Single(Package_ID,IMU_t);
        *p = HERO_IMU;
        Package_Write_Pionter_End(Package_ID,IMU_t);
        //SWDG_FEED(SWDG_IMU_ID);
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

    //陀螺仪加热初始化
    IMU1_TempCTR_init();
		IMU2_TempCTR_init();
    // 尝试从Flash中读取零飘数据 若无数据或需要重测，则会自动重测，完成后函数返回
    LoadGyroOffSet(IMU1_set);
		LoadGyroOffSet(IMU2_set);
    
    //陀螺仪冗余调度初始化
    IMU_redundancy_init();

    //数据服务器初始化
    Package_Pionter_Add("IMU", HERO_IMU);
	Package_ID = Package_Find_Num("IMU");

    //初始化姿态解算线程
    atti_calcu = rt_thread_create(
        "AT_CTRL",             //线程名
        AttiCalcu_thread,      //线程入口
        RT_NULL,               //入口参数无
        4096,                  //线程栈
        THREAD_PRIO_ATTICALCU, //线程优先级
        2);                    //线程时间片大小

    //线程启动失败返回false
    if (rt_thread_startup(atti_calcu) != RT_EOK)
        return RT_ERROR;

    return RT_EOK;
}
