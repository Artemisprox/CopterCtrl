#include "func_SensorRAW.h"
#include "drv_thread.h"
#include "board.h"
#include "drv_HWTimer.h"
#include "drv_icm20602.h"
#include "func_TempCtr.h"
#include "mod_Monitor.h"
#include "drv_utils.h"

// 传感器原始数据
Sensor_RAW_t Sensor_RAW;

/* Private define ----------------------------------------------------------- */
#define IMU1_INT_PIN GET_PIN(C, 3)

// 通信读取温度数据
static int8_t Sensor_ParseTemp()
{ 
    static float last_temp = 0;
    float now_temp;

    icm20602_get_temper(&now_temp);

    Sensor_RAW.Temperature = now_temp;

    if (Sensor_RAW.Temperature != last_temp)
    { // 检测到温度数据更新
        while (rt_sem_trytake(&temp_pid_sem) == RT_EOK)
            ;                          // 清空多余的信号量
        rt_sem_release(&temp_pid_sem); // 重新释放信号量
    }

    last_temp = Sensor_RAW.Temperature;

    return RT_EOK;
}

// 通信读取角速度
static int8_t Sensor_ParseACCL()
{
    float Accl[3];
    icm20602_get_accel(Accl);
    Sensor_RAW.Accl_Raw.x = Accl[0];
    Sensor_RAW.Accl_Raw.y = Accl[1];
    Sensor_RAW.Accl_Raw.z = Accl[2];
    return RT_EOK;
}
// 通信读取加速度
static int8_t Sensor_ParseGYRO()
{
    float Gyro[3];
    icm20602_get_gyro(Gyro);
    Sensor_RAW.Gyro_Raw.x = Gyro[0];
    Sensor_RAW.Gyro_Raw.y = Gyro[1];
    Sensor_RAW.Gyro_Raw.z = Gyro[2];
    return RT_EOK;
}

static int TempReadScale = 100;
// 刷新角速度、加速度、温度
static void Sensor_FreshData()
{
    Sensor_ParseACCL();
    Sensor_ParseGYRO();
    if (TempReadScale>0)
    {
        TempReadScale--;
    }
    else
    {
        Sensor_ParseTemp();
        TempReadScale = 100;
    }
    Sensor_RAW.RawDataReady = 1;
}

/********** 中断读取部分 **********/

static rt_thread_t IMU_SpiTrans = RT_NULL;

static struct rt_event IMU_Event;           // 使用事件集对IMU中断进行响应
static struct rt_semaphore IMU_CALTrig_Sem; // 通信结束后通知数据处理线程处理数据

// 用于计算中断触发频率的相关变量
static int LastCount, NowCount, FirstRecFlag, FirstFilterFlag;
static float IMUFrqNow, IMUFrqFilter;

// 中断与事件的对应关系
//（暂时只使用EVT_GYRO，因为当前仅使用陀螺仪进行触发读取）
#define EVT_ACCL 1 << 1
#define EVT_GYRO 1 << 2

// IO中断中进行频率计算和事件集的发送，其它线程中进行SPI读取
static void Gyro_irq(void *Para)
{
    int DeltaCount;

    NowCount = TIM11_GetCNT();
    if (FirstRecFlag)
    {
        LastCount = NowCount;
        FirstRecFlag = 0;
        return;
    }

    DeltaCount = NowCount - LastCount;
    if (DeltaCount <= 0)
    { // 计数值跨圈处理
        DeltaCount += 65536;
    }
    IMUFrqFilter = 20000000.0f / DeltaCount;
    LastCount = NowCount;

    if (FirstFilterFlag || UTILS_IS_NAN(IMUFrqNow))
    {
        FirstFilterFlag = 0;
        IMUFrqNow = IMUFrqFilter;
        Sensor_RAW.RawDataReady = 0;
    }
    else
    {
        IMUFrqNow = IMUFrqNow * 0.96f + IMUFrqFilter * 0.04f; // 滞后滤波
        Sensor_RAW.DataRate = IMUFrqNow;
        rt_event_send(&IMU_Event, EVT_GYRO);
    }
}

// IMU触发式数据接收线程
static void IMU_SensorRAWProcess_thread(void *Para)
{
    rt_uint32_t EVT_recv;
   // SWDG_START(SWDG_RAW_DATA_PROCESS_ID);

    while (1)
    {
        // 等待数据产生
        rt_event_recv(&IMU_Event, EVT_GYRO,
                      RT_EVENT_FLAG_OR | RT_EVENT_FLAG_CLEAR, 2, &EVT_recv);

      //  SWDG_FEED(SWDG_RAW_DATA_PROCESS_ID);
        if (EVT_recv == 0)
        {
            rt_thread_delay(1); // 出问题了
        }

        Sensor_FreshData();

        // 取完可能存在的堆积的信号量
        while (rt_sem_trytake(&IMU_CALTrig_Sem) == RT_EOK)
            ;
        // 发送信号量，触发姿态融合算法
        rt_sem_release(&IMU_CALTrig_Sem);
    }
}

// 挂起在信号量上，等待新数据接收完毕
void Sensor_WaitForRawData()
{
    /* 等待硬触发 */
    rt_sem_take(&IMU_CALTrig_Sem, RT_WAITING_FOREVER);
}

/********** 初始化与启动部分 **********/

// IMU硬触发数据接收的初始化
static void HWTrig_init(void)
{
    // 初始化用于计算中断频率的定时器
    MX_TIM11_Init();
    FirstFilterFlag = 1;
    FirstRecFlag = 1;
    LastCount = 0;

    // 初始化中断读取用的事件集
    rt_event_init(&IMU_Event, "IMU_EVT", RT_IPC_FLAG_PRIO);
    // 初始化用于触发IMU数据读取的信号量
    rt_sem_init(&IMU_CALTrig_Sem, "IMUTriS", 0, RT_IPC_FLAG_PRIO);

    //初始化中断数据接收线程
    IMU_SpiTrans = rt_thread_create(
        "INTSPI",                     //线程名
        IMU_SensorRAWProcess_thread,       //线程入口
        RT_NULL,                      //入口参数无
        2048,                         //线程栈
        THREAD_PRIO_RAW_DATA_PROCESS, //线程优先级
        1);                           //线程时间片大小

    //线程创建失败返回false
    if (IMU_SpiTrans == RT_NULL)
    {
        return;
    }

    //线程启动失败返回false
    if (rt_thread_startup(IMU_SpiTrans) != RT_EOK)
    {
        return;
    }

    // 初始化中断IO及其回调
    rt_pin_mode(IMU1_INT_PIN, PIN_MODE_INPUT);
    rt_pin_attach_irq(IMU1_INT_PIN, PIN_IRQ_MODE_RISING, Gyro_irq, RT_NULL);
    rt_pin_irq_enable(IMU1_INT_PIN, PIN_IRQ_ENABLE);
}

// ICM20602设备初始化 初始化后可通过接口读取已有最新数据，通过接口可以挂起等待新数据产生
void SensorRawProcess_Init()
{
    Sensor_RAW.RawDataReady = 0;

    // 初始化用于控温的信号量，信号量在每次检测到温度数据变化后释放一个
	rt_sem_init(&temp_pid_sem, "TP_Sem", 0, RT_IPC_FLAG_FIFO);

    // 初始化传感芯片
    ICM_init();

    // 启动硬触发数据接收
    HWTrig_init();
}
