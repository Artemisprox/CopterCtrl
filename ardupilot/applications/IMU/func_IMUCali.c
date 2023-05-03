#include "func_IMUCali.h"
#include "drv_thread.h"
#include "drv_HWTimer.h"
#include "func_SensorRAW.h"
#include "func_TempCtr.h"


#define FLASH_ERASE_PIN GET_PIN(B, 6)
#define GYRO_SUM_MAX 60000 // 校准陀螺仪积分时需要收集数据的个数

#define GYRO_RAWDATA(Axis) (copter_IMU_RAW.Gyro_Raw.Axis)
#define ACCL_RAWDATA(Axis) (copter_IMU_RAW.Accl_Raw.Axis)

// 以云台为例：枪口方向为前方，垂直地面向上为上方，类似于第一视角
// X为 后方(枪管反方向轴)
// GYRO_X_SOURCE -GYRO_RAWDATA(y)表示 云台后方数据=(-1*原始y轴数据)
// 角速度方向遵循右手螺旋
#define GYRO_X_SOURCE -GYRO_RAWDATA(y)
#define ACCL_X_SOURCE -ACCL_RAWDATA(y)
// Y为 右方
#define GYRO_Y_SOURCE -GYRO_RAWDATA(x)
#define ACCL_Y_SOURCE -ACCL_RAWDATA(x)
// Z为 上方
#define GYRO_Z_SOURCE -GYRO_RAWDATA(z)
#define ACCL_Z_SOURCE -ACCL_RAWDATA(z)

// 偏移量为Raw数据的偏移量，与安装位置无关
// 使用偏移量时，先用Raw数据减去偏移量，再进行安装位置校正
AHRS_Gyro_t IMU1_OffSet = {0}; // 默认为0
AHRS_Gyro_t IMU2_OffSet = {0}; // 默认为0

IMU_set_e IMU_Cali_set;

// 输入IMU原始数据，按照设置获取经安装位置校正和零飘校正后的六轴数据
void GetCaliIMUData(AHRS_Accl_t *AcclRaw, AHRS_Gyro_t *GyroRaw, AHRS_Accl_t *Accl, AHRS_Gyro_t *Gyro)
{
    Accl->x = ACCL_X_SOURCE;
    Accl->y = ACCL_Y_SOURCE;
    Accl->z = ACCL_Z_SOURCE;
    Gyro->x = GYRO_X_SOURCE;
    Gyro->y = GYRO_Y_SOURCE;
    Gyro->z = GYRO_Z_SOURCE;
}

// Flash存储函数
static void FlashRecord(AHRS_Gyro_t *CaliData , rt_uint8_t IMU_set)
{
    rt_uint8_t temp_data[13];
    temp_data[0] = ((rt_uint8_t *)(&CaliData->x))[0];
    temp_data[1] = ((rt_uint8_t *)(&CaliData->x))[1];
    temp_data[2] = ((rt_uint8_t *)(&CaliData->x))[2];
    temp_data[3] = ((rt_uint8_t *)(&CaliData->x))[3];
    temp_data[4] = ((rt_uint8_t *)(&CaliData->y))[0];
    temp_data[5] = ((rt_uint8_t *)(&CaliData->y))[1];
    temp_data[6] = ((rt_uint8_t *)(&CaliData->y))[2];
    temp_data[7] = ((rt_uint8_t *)(&CaliData->y))[3];
    temp_data[8] = ((rt_uint8_t *)(&CaliData->z))[0];
    temp_data[9] = ((rt_uint8_t *)(&CaliData->z))[1];
    temp_data[10] = ((rt_uint8_t *)(&CaliData->z))[2];
    temp_data[11] = ((rt_uint8_t *)(&CaliData->z))[3];
    temp_data[12] = 0x0F;
    /* 测量完毕, 开始读写 Flash */
//    Hwdt_Feed_Slowly(RT_TRUE); // 开始操作 Flash 数据, 需要开始缓慢喂狗
    rt_enter_critical();       // 进入临界区防止操作系统调度
    
    if(IMU_set == 1)
        stm32_flash_erase(IMU1_BIAS_DATA_ADDR, sizeof(float) * 3 + 1);
    else if(IMU_set == 2)
        stm32_flash_erase(IMU2_BIAS_DATA_ADDR, sizeof(float) * 3 + 1);

    rt_exit_critical();    // 退出临界区继续启动调度器
    rt_thread_mdelay(500); // 擦除与写入需要间隔一段时间
    rt_enter_critical();   // 进入临界区防止操作系统调度

    if(IMU_set == 1)
        stm32_flash_write(IMU1_BIAS_DATA_ADDR, temp_data, sizeof(float) * 3 + 1);
    else if(IMU_set == 2)
        stm32_flash_write(IMU2_BIAS_DATA_ADDR, temp_data, sizeof(float) * 3 + 1);
        
    rt_exit_critical();         // 退出临界区继续启动调度器
//    Hwdt_Feed_Slowly(RT_FALSE); // 操作 Flash 数据结束, 重新开始正常喂狗
}

IMU_GyroCali_State_e Gyro_Cali_State = Cali_Error; // 记录当前校准状态

static struct rt_semaphore IMU1_CaliFinish_Sem; // 通信结束后通知数据处理线程处理数据
static struct rt_semaphore IMU2_CaliFinish_Sem; // 通信结束后通知数据处理线程处理数据
static rt_thread_t IMU_GyroCali_Tid = RT_NULL; // IMU校准线程

static double Gyro_Sum[3] = {0};         // x、y、z三轴的校准求和数值
static int GyroSum_Count;                // 记录参与求和的数据个数
static AHRS_Gyro_t GyroRawNow, Cali_Out; // 当前陀螺仪Raw数据、最终输出的测定结果

#define X2CAL(x) ((x) * (x))

// 校准线程
void IMU_GyroCali_Thread(void *Para)
{
    float GyroSumNow;  // 三轴角速度平方和，用于判断当前IMU是否被意外转动
    int TimeCount = 0; // 用来记录时间

    Gyro_Cali_State = (IMU_GyroCali_State_e)1; // 开始进行第一步

    // 记录数据的过程在这个while中完成
    while (1)
    {   
        int TempOK = 1;
        // 等待新数据
        switch (IMU_Cali_set)
        {
        case IMU1_set:
            Sensor_WaitFor_IMU1_RawData();
            // 读数据
            GyroRawNow.x = Sensor_RAW_IMU1.Gyro_Raw.x;
            GyroRawNow.y = Sensor_RAW_IMU1.Gyro_Raw.y;
						GyroRawNow.z = Sensor_RAW_IMU1.Gyro_Raw.z;
            TempOK = IMU1_IfTempOK;
            break;
        
        case IMU2_set:
            Sensor_WaitFor_IMU2_RawData();
            // 读数据
            GyroRawNow.x = Sensor_RAW_IMU2.Gyro_Raw.x;
            GyroRawNow.y = Sensor_RAW_IMU2.Gyro_Raw.y;
            GyroRawNow.z = Sensor_RAW_IMU2.Gyro_Raw.z;
            TempOK = IMU2_IfTempOK;
            break;
        }
					
        
        GyroSumNow = X2CAL(GyroRawNow.x) + X2CAL(GyroRawNow.y) + X2CAL(GyroRawNow.z);
        if (GyroSumNow > 1.0f)
        {
            // 云台发生了移动，重新开始
            Gyro_Cali_State = (IMU_GyroCali_State_e)1; // 开始进行第一步
        }
#if GYROCALI_WAIT_FOR_TEMPERATURE
        if (TempOK == RT_ERROR)
        {
            // 温度达不到设定值，重新开始
            Gyro_Cali_State = (IMU_GyroCali_State_e)1; // 开始进行第一步
            rt_thread_delay(50);
        }
#endif
        switch (Gyro_Cali_State)
        {
        case Cali_Start:
            // 清空求和变量
            Gyro_Sum[0] = 0;
            Gyro_Sum[1] = 0;
            Gyro_Sum[2] = 0;
            GyroSum_Count = 0;
            TimeCount = 0;
            Gyro_Cali_State++;
            break;

        case Cali_Wait:
            rt_thread_delay(10);
            TimeCount++;
            if (TimeCount > 500)
            {
                // 等待5S后进行下一步
                TimeCount = 0;
                Gyro_Cali_State++;
                rt_thread_delay(80);
            }
            break;
        case Cali_Recording:
            // 求和过程
            Gyro_Sum[0] += GyroRawNow.x;
            Gyro_Sum[1] += GyroRawNow.y;
            Gyro_Sum[2] += GyroRawNow.z;
            GyroSum_Count++;
            Cali_Out.x = Gyro_Sum[0] / GyroSum_Count;
            Cali_Out.y = Gyro_Sum[1] / GyroSum_Count;
            Cali_Out.z = Gyro_Sum[2] / GyroSum_Count;
            break;

        default:
            Gyro_Cali_State = (IMU_GyroCali_State_e)1; // 开始进行第一步
            break;
        }

        if (GyroSum_Count >= GYRO_SUM_MAX)
        {
            // 这时擦除Flash并进行数据保存
            Cali_Out.x = Gyro_Sum[0] / GyroSum_Count;
            Cali_Out.y = Gyro_Sum[1] / GyroSum_Count;
            Cali_Out.z = Gyro_Sum[2] / GyroSum_Count;
            GyroSumNow = X2CAL(Cali_Out.x) + X2CAL(Cali_Out.y) + X2CAL(Cali_Out.z);
            if (GyroSumNow < 2.0f)
            {
                switch (IMU_Cali_set)
                {
                case IMU1_set:
										Sensor_WaitFor_IMU1_RawData();
                    // 数据记录完成，准备保存数据
                    FlashRecord(&Cali_Out,IMU_Cali_set);
                    IMU1_OffSet.x = Cali_Out.x;
                    IMU1_OffSet.y = Cali_Out.y;
                    IMU1_OffSet.z = Cali_Out.z;
                    break;
                
                case IMU2_set:
                    Sensor_WaitFor_IMU2_RawData();
                    // 读数据
                    FlashRecord(&Cali_Out,IMU_Cali_set);
                    IMU2_OffSet.x = Cali_Out.x;
                    IMU2_OffSet.y = Cali_Out.y;
                    IMU2_OffSet.z = Cali_Out.z;
                    break;
                }
               break;
            }
            else
            { // 如果测量失败则本次放弃
                break;
            }
        }
    }
		switch(IMU_Cali_set)
		{
			case IMU1_set:
				rt_sem_release(&IMU1_CaliFinish_Sem);
				break;
			case IMU2_set:
				rt_sem_release(&IMU2_CaliFinish_Sem);
				break;
		}
    
		
		rt_pin_write(BEEP_PIN_NUM, PIN_HIGH);
    rt_thread_mdelay(20);
		rt_pin_write(BEEP_PIN_NUM, PIN_LOW);
		
    while (1)
    {
        rt_thread_delay(100);
    }
}

// 角速度计校准函数，此函数初始化校准线程，并挂起等待校准线程结束，线程运行结束后此函数退出
static void Gyro_Cali(IMU_set_e IMU_set)
{
    switch(IMU_set)
    {
        case IMU1_set:
            IMU_Cali_set = IMU1_set;
						// 用来挂起的信号量，校准线程运行结束后会释放
						rt_sem_init(&IMU1_CaliFinish_Sem, "IMU1TriS", 0, RT_IPC_FLAG_PRIO);
						break;
        case IMU2_set:
            IMU_Cali_set = IMU2_set;
						// 用来挂起的信号量，校准线程运行结束后会释放
						rt_sem_init(&IMU2_CaliFinish_Sem, "IMU2TriS", 0, RT_IPC_FLAG_PRIO);
						break;
    }  

    // 初始化零飘校准线程
    IMU_GyroCali_Tid = rt_thread_create(
        "INTSPI",                 // 线程名
        IMU_GyroCali_Thread,      // 线程入口
        RT_NULL,                  // 入口参数无
        512,                      // 线程栈
        THREAD_PRIO_IMU_GYROCALI, // 线程优先级
        1);                       // 线程时间片大小

    // 线程创建失败返回false
    if (IMU_GyroCali_Tid == RT_NULL)
    {
        return;
    }

    // 线程启动失败返回false
    if (rt_thread_startup(IMU_GyroCali_Tid) != RT_EOK)
    {
        return;
    }

    // 挂起等待校准结束
		switch(IMU_Cali_set)
		{
			case IMU1_set:
				rt_sem_take(&IMU1_CaliFinish_Sem,RT_WAITING_FOREVER);
				break;
			case IMU2_set:
				rt_sem_take(&IMU2_CaliFinish_Sem,RT_WAITING_FOREVER);
				break;
		}
    rt_thread_delete(IMU_GyroCali_Tid);
}

// 全片擦除后应为0xFF，校准完成后为0x0F
static uint8_t CaliFlag_Read;
static uint8_t OffSetRST_KeyFlag;

// 从上电前开始按住按键，上电约1s后蜂鸣器响起1s，响起期间松手可以触发零飘重测，其它情况无法触发零飘重测
// 不满足按键触发规则时返回0，满足触发规则返回1，返回1意味着需要清除Flash和重测零飘
static int CaliKeyCheck()
{
    int Time;
    int ConfirmCount;

    // 初始化零飘校准触发按键
    rt_pin_mode(FLASH_ERASE_PIN, PIN_MODE_INPUT);

    ConfirmCount = 15;
    Time = 0;

    // 上电后的第一秒内在下面这个While中运行
    while (1)
    {
        if (!rt_pin_read(FLASH_ERASE_PIN))
        {                              // 按下按键
            if (ConfirmCount < 30)
            {
                ConfirmCount++;
            }
        }
        else
        {
            ConfirmCount -= 2;
            if (ConfirmCount < 0)
            {
                // 按键松开了，此时松开说明不需要重测零飘
                return 0;
            }
        }
        rt_thread_delay(1);
        Time++;
        if (Time > 900)
        {
            break;
        }
    }
    // 上电后的第一秒内按键一直保持按下，此时开蜂鸣器并等待按键松开
    Time = 0; // 重新计时
    while (1)
    {
        if (!rt_pin_read(FLASH_ERASE_PIN))
        { // 按下按键
            if (ConfirmCount < 30)
            {
                ConfirmCount++;
            }
        }
        else
        {
            ConfirmCount -= 2;
            if (ConfirmCount < 0)
            {
                // 按键松开了，此时松开说明需要重测零飘
                return 1;
            }
        }
        rt_thread_delay(1);
        Time++;
        if (Time > 1000)
        {
            break;
        }
    }
    // 到这里说明可能是误触，不进行零飘重测，直接跳过
    return 0;
}

// 尝试从Flash中读取 若无数据或需要重测，则会自动重测，完成后函数返回
int LoadGyroOffSet(IMU_set_e IMU_set)
{
    uint8_t ReadTemp[12];

    // 读取Flash中的标记
    rt_enter_critical();
//    Hwdt_Feed_Slowly(RT_TRUE);
    switch (IMU_set)
    {
    case IMU1_set:
        stm32_flash_read(IMU1_CALI_FLAG_ADDR, &CaliFlag_Read, sizeof(rt_uint8_t));
        stm32_flash_read(IMU1_BIAS_DATA_ADDR, &ReadTemp[0], sizeof(ReadTemp));
        break;
    
    case IMU2_set:
        stm32_flash_read(IMU2_CALI_FLAG_ADDR, &CaliFlag_Read, sizeof(rt_uint8_t));
        stm32_flash_read(IMU2_BIAS_DATA_ADDR, &ReadTemp[0], sizeof(ReadTemp));
        break;
    }
//    Hwdt_Feed_Slowly(RT_FALSE);
    rt_exit_critical();

    // 检查按键，判断是否需要重测零飘
    OffSetRST_KeyFlag = CaliKeyCheck();
    if ((CaliFlag_Read == 0x0F) && (OffSetRST_KeyFlag == 0))
    { // Flash中的标记表明有正常的零飘数据且不需要重测零飘
        if (!(((ReadTemp[0] == 0xFF) && (ReadTemp[1] == 0xFF) && (ReadTemp[2] == 0xFF) && (ReadTemp[3] == 0xFF)) ||
              ((ReadTemp[4] == 0xFF) && (ReadTemp[5] == 0xFF) && (ReadTemp[6] == 0xFF) && (ReadTemp[7] == 0xFF)) ||
              ((ReadTemp[8] == 0xFF) && (ReadTemp[9] == 0xFF) && (ReadTemp[10] == 0xFF) && (ReadTemp[11] == 0xFF))))
        { // 数据不为空
            // 正常加载已有零飘数据
            switch (IMU_set)
                    {
                    case IMU1_set:
                        IMU1_OffSet.x = *((float *)&ReadTemp[0]);
                        IMU1_OffSet.y = *((float *)&ReadTemp[4]);
                        IMU1_OffSet.z = *((float *)&ReadTemp[8]);
                        break;
                    
                    case IMU2_set:
                        // 读数据
                        FlashRecord(&Cali_Out,IMU_set);
                        IMU2_OffSet.x = *((float *)&ReadTemp[0]);
                        IMU2_OffSet.y = *((float *)&ReadTemp[4]);
                        IMU2_OffSet.z = *((float *)&ReadTemp[8]);
                        break;
                    }
            return 0;
        }
    }
    else
    {
        // 如果标志位不正确, 先检查前方数据是不是全部为 0xFF 此时只有被全片擦除才会开始自动重新标定
        if (!((ReadTemp[0] == 0xFF) && (ReadTemp[1] == 0xFF) && (ReadTemp[2] == 0xFF) && (ReadTemp[3] == 0xFF) &&
              (ReadTemp[4] == 0xFF) && (ReadTemp[5] == 0xFF) && (ReadTemp[6] == 0xFF) && (ReadTemp[7] == 0xFF) &&
              (ReadTemp[8] == 0xFF) && (ReadTemp[9] == 0xFF) && (ReadTemp[10] == 0xFF) && (ReadTemp[11] == 0xFF)))
        { // 标志位异常异常但是内存前方存在部分数据, 将零漂载入为 0 同时报警

            switch (IMU_set)
                {
                case IMU1_set:
                    IMU1_OffSet.x = 0.f;
                    IMU1_OffSet.y = 0.f;
                    IMU1_OffSet.z = 0.f;                        
                    break;
                
                case IMU2_set:
                    // 读数据
                    FlashRecord(&Cali_Out,IMU_set);
                    IMU2_OffSet.x = 0.f;
                    IMU2_OffSet.y = 0.f;                        
                    IMU2_OffSet.z = 0.f;
                    break;
                }

            // 发出报警提示
            rt_thread_mdelay(1000);
            return 0;
        }
    }
		rt_pin_write(BEEP_PIN_NUM, PIN_HIGH);
    rt_thread_mdelay(20);
		rt_pin_write(BEEP_PIN_NUM, PIN_LOW);
		
		rt_pin_write(BEEP_PIN_NUM, PIN_HIGH);
    rt_thread_mdelay(20);
		rt_pin_write(BEEP_PIN_NUM, PIN_LOW);
		
    // FLASH 未写入数据或需要重测零飘
    Gyro_Cali(IMU_set);
		
		
    return 1;
}
