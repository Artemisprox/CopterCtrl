#include "func_SensorRAW.h"
#include "drv_thread.h"
#include "board.h"
#include "drv_HWTimer.h"
#include "drv_icm20602.h"
#include "func_TempCtr.h"
#include "mod_Monitor.h"
#include "drv_utils.h"
#include "func_IMU_redundancy.h"

// 传感器原始数据
Sensor_RAW_t Sensor_RAW_IMU1;
Sensor_RAW_t Sensor_RAW_IMU2;

/* Private define ----------------------------------------------------------- */
#define IMU1_INT_PIN GET_PIN(B, 14)
#define IMU2_INT_PIN GET_PIN(B, 15)
// 通信读取温度数据
static int8_t Sensor_ParseTemp_IMU1()
{ 
    static float last_temp = 0;
    float now_temp;

    icm20602_get_temper_IMU1(&now_temp);

    Sensor_RAW_IMU1.Temperature = now_temp;

    if (Sensor_RAW_IMU1.Temperature != last_temp)
    { // 检测到温度数据更新
        while (rt_sem_trytake(&imu1_temp_pid_sem) == RT_EOK)
            ;                          // 清空多余的信号量
        rt_sem_release(&imu1_temp_pid_sem); // 重新释放信号量
    }

    last_temp = Sensor_RAW_IMU1.Temperature;

    return RT_EOK;
}

static int8_t Sensor_ParseTemp_IMU2()
{ 
    static float last_temp = 0;
    float now_temp;

    icm20602_get_temper_IMU2(&now_temp);

    Sensor_RAW_IMU2.Temperature = now_temp;

    if (Sensor_RAW_IMU2.Temperature != last_temp)
    { // 检测到温度数据更新
        while (rt_sem_trytake(&imu1_temp_pid_sem) == RT_EOK)
            ;                          // 清空多余的信号量
        rt_sem_release(&imu1_temp_pid_sem); // 重新释放信号量
    }

    last_temp = Sensor_RAW_IMU2.Temperature;

    return RT_EOK;
}

// 通信读取角速度
static int8_t Sensor_ParseACCL_IMU1()
{
    float Accl[3];
    icm20602_get_accel_IMU1(Accl);
    Sensor_RAW_IMU1.Accl_Raw.x = Accl[0];
    Sensor_RAW_IMU1.Accl_Raw.y = Accl[1];
    Sensor_RAW_IMU1.Accl_Raw.z = Accl[2];
    return RT_EOK;
}

// 通信读取角速度
static int8_t Sensor_ParseACCL_IMU2()
{
    float Accl[3];
    icm20602_get_accel_IMU2(Accl);
    Sensor_RAW_IMU2.Accl_Raw.x = Accl[0];
    Sensor_RAW_IMU2.Accl_Raw.y = Accl[1];
    Sensor_RAW_IMU2.Accl_Raw.z = Accl[2];
    return RT_EOK;
}

// 通信读取加速度
static int8_t Sensor_ParseGYRO_IMU1()
{
    float Gyro[3];
    icm20602_get_gyro_IMU1(Gyro);
    Sensor_RAW_IMU1.Gyro_Raw.x = Gyro[0];
    Sensor_RAW_IMU1.Gyro_Raw.y = Gyro[1];
    Sensor_RAW_IMU1.Gyro_Raw.z = Gyro[2];
	
    return RT_EOK;
}

// 通信读取加速度
static int8_t Sensor_ParseGYRO_IMU2()
{
    float Gyro[3];
    icm20602_get_gyro_IMU2(Gyro);
    Sensor_RAW_IMU2.Gyro_Raw.x = Gyro[0];
    Sensor_RAW_IMU2.Gyro_Raw.y = Gyro[1];
    Sensor_RAW_IMU2.Gyro_Raw.z = Gyro[2];
    return RT_EOK;
}

// 刷新角速度、加速度、温度
static void Sensor_FreshData_IMU1()
{
    static int TempReadScale = 100;
    Sensor_ParseACCL_IMU1();
    Sensor_ParseGYRO_IMU1();
    if (TempReadScale>0)
    {
        TempReadScale--;
    }
    else
    {
        Sensor_ParseTemp_IMU1();
        TempReadScale = 100;
    }
		
    Sensor_RAW_IMU1.RawDataReady = 1;
}

static void Sensor_FreshData_IMU2()
{
    static int TempReadScale = 100;
    Sensor_ParseACCL_IMU2();
    Sensor_ParseGYRO_IMU2();
    if (TempReadScale>0)
    {
        TempReadScale--;
    }
    else
    {
        Sensor_ParseTemp_IMU2();
        TempReadScale = 100;
    }
    Sensor_RAW_IMU2.RawDataReady = 1;
}

/********** 中断读取部分 **********/

static rt_thread_t IMU_SpiTrans = RT_NULL;

static struct rt_event IMU_Event;           // 使用事件集对IMU中断进行响应
static struct rt_semaphore IMU1_CALTrig_Sem; // 通信结束后通知数据处理线程处理数据
static struct rt_semaphore IMU2_CALTrig_Sem; // 通信结束后通知数据处理线程处理数据
static struct rt_semaphore IMU_Read_Sem;//防止IMU1和IMU2读取互相打断
// 用于计算中断触发频率的相关变量
static int LastCount1, NowCount1, FirstRecFlag1, FirstFilterFlag1;
static int LastCount2, NowCount2, FirstRecFlag2, FirstFilterFlag2;
static float IMU1FrqNow, IMU1FrqFilter;
static float IMU2FrqNow, IMU2FrqFilter;
// 中断与事件的对应关系
//（暂时只使用EVT_GYRO，因为当前仅使用陀螺仪进行触发读取）
#define EVT_IMU1 1 << 1
#define EVT_IMU2 1 << 2

// IO中断中进行频率计算和事件集的发送，其它线程中进行SPI读取
static void IMU1_irq(void *Para)
{
    int DeltaCount;

    NowCount1 = TIM11_GetCNT();
    if (FirstRecFlag1)
    {
        LastCount1 = NowCount1;
        FirstRecFlag1 = 0;
        return;
    }

    DeltaCount = NowCount1 - LastCount1;
    if (DeltaCount <= 0)
    { // 计数值跨圈处理
        DeltaCount += 65536;
    }
    IMU1FrqFilter = 20000000.0f / DeltaCount;
    LastCount1 = NowCount1;

    if (FirstFilterFlag1 || UTILS_IS_NAN(IMU1FrqNow))
    {
        FirstFilterFlag1 = 0;
        IMU1FrqNow = IMU1FrqFilter;
        Sensor_RAW_IMU1.RawDataReady = 0;
    }
    else
    {
        IMU1FrqNow = IMU1FrqNow * 0.96f + IMU1FrqFilter * 0.04f; // 滞后滤波
        Sensor_RAW_IMU1.DataRate = IMU1FrqNow;
        Sensor_RAW_IMU1.DataFreshtime = NowCount1;
        rt_event_send(&IMU_Event, EVT_IMU1 );
    }
}

// IO中断中进行频率计算和事件集的发送，其它线程中进行SPI读取
static void IMU2_irq(void *Para)
{
    int DeltaCount;

    NowCount2 = TIM11_GetCNT();
    if (FirstRecFlag2)
    {
        LastCount2 = NowCount2;
        FirstRecFlag2 = 0;
        return;
    }

    DeltaCount = NowCount2 - LastCount2;
    if (DeltaCount <= 0)
    { // 计数值跨圈处理
        DeltaCount += 65536;
    }
    IMU2FrqFilter = 20000000.0f / DeltaCount;
    LastCount2 = NowCount2;

    if (FirstFilterFlag2 || UTILS_IS_NAN(IMU2FrqNow))
    {
        FirstFilterFlag2 = 0;
        IMU2FrqNow = IMU2FrqFilter;
        Sensor_RAW_IMU2.RawDataReady = 0;
    }
    else
    {
        IMU2FrqNow = IMU2FrqNow * 0.96f + IMU2FrqFilter * 0.04f; // 滞后滤波
        Sensor_RAW_IMU2.DataRate = IMU2FrqNow;
        Sensor_RAW_IMU2.DataFreshtime = NowCount2;
        rt_event_send(&IMU_Event, EVT_IMU2 );
    }
}

// IMU触发式数据接收线程
static void IMU1_SensorRAWProcess_thread(void *Para)
{
    rt_uint32_t EVT_recv;
   // SWDG_START(SWDG_RAW_DATA_PROCESS_ID);

    while (1)
    {
        // 等待数据产生
        rt_event_recv(&IMU_Event, EVT_IMU1,
                      RT_EVENT_FLAG_OR | RT_EVENT_FLAG_CLEAR, 2, &EVT_recv);

      //  SWDG_FEED(SWDG_RAW_DATA_PROCESS_ID);
        if (EVT_recv == 0)
        {
            rt_thread_delay(1); // 出问题了
				}
				else{
				
				rt_sem_take(&IMU_Read_Sem,RT_WAITING_FOREVER);//IMU读取信号量，防止spi读取时被打断
				
				Sensor_FreshData_IMU1();//读取IMU1数据
				
				// 取完可能存在的堆积的信号量
        while (rt_sem_trytake(&IMU_Read_Sem) == RT_EOK)
            ;
        // 发送信号量，触发姿态融合算法
        rt_sem_release(&IMU_Read_Sem);
					
        // 取完可能存在的堆积的信号量
        while (rt_sem_trytake(&IMU1_CALTrig_Sem) == RT_EOK)
            ;
        // 发送信号量，触发姿态融合算法
        rt_sem_release(&IMU1_CALTrig_Sem);
        }

        
    }
}

// IMU触发式数据接收线程
static void IMU2_SensorRAWProcess_thread(void *Para)
{
    rt_uint32_t EVT_recv;
   // SWDG_START(SWDG_RAW_DATA_PROCESS_ID);

    while (1)
    {
        // 等待数据产生
        rt_event_recv(&IMU_Event, EVT_IMU2,
                      RT_EVENT_FLAG_OR | RT_EVENT_FLAG_CLEAR, 2, &EVT_recv);

      //  SWDG_FEED(SWDG_RAW_DATA_PROCESS_ID);
        if (EVT_recv == 0)
        {
            rt_thread_delay(1); // 出问题了
				}else
				{
					
				rt_sem_take(&IMU_Read_Sem,RT_WAITING_FOREVER);//IMU读取信号量，防止spi读取时被打断
					
				Sensor_FreshData_IMU2();

						// 取完可能存在的堆积的信号量
        while (rt_sem_trytake(&IMU_Read_Sem) == RT_EOK)
            ;
        // 发送信号量，触发姿态融合算法
        rt_sem_release(&IMU_Read_Sem);
					
        // 取完可能存在的堆积的信号量
        while (rt_sem_trytake(&IMU2_CALTrig_Sem) == RT_EOK)
            ;
        // 发送信号量，触发姿态融合算法
        rt_sem_release(&IMU2_CALTrig_Sem);
        }

        
    }
}

// 挂起在信号量上，等待新数据接收完毕
void Sensor_WaitFor_IMU1_RawData()
{
    /* 等待硬触发 */
    rt_sem_take(&IMU1_CALTrig_Sem, RT_WAITING_FOREVER);
}

void Sensor_WaitFor_IMU2_RawData()
{
    /* 等待硬触发 */
    rt_sem_take(&IMU2_CALTrig_Sem, RT_WAITING_FOREVER);
}

/********** 初始化与启动部分 **********/

// IMU硬触发数据接收的初始化
static void HWTrig_init(void)
{
    // 初始化用于计算中断频率的定时器
    MX_TIM11_Init();
    FirstFilterFlag1 = 1;
    FirstRecFlag1 = 1;
    LastCount1 = 0;
	
		FirstFilterFlag2 = 1;
    FirstRecFlag2 = 1;
    LastCount2 = 0;

    // 初始化中断读取用的事件集
    rt_event_init(&IMU_Event, "IMU_EVT", RT_IPC_FLAG_PRIO);
    // 初始化用于触发IMU数据读取的信号量
    rt_sem_init(&IMU1_CALTrig_Sem, "IMU1TriS", 0, RT_IPC_FLAG_PRIO);
    rt_sem_init(&IMU2_CALTrig_Sem, "IMU2TriS", 0, RT_IPC_FLAG_PRIO);
    rt_sem_init(&IMU_Read_Sem, "IMU_Read_Sem", 1, RT_IPC_FLAG_PRIO);
	  
    //初始化中断数据接收线程
    IMU_SpiTrans = rt_thread_create(
        "IMU1_PRO",                     //线程名
        IMU1_SensorRAWProcess_thread,       //线程入口
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
    rt_pin_mode(IMU1_INT_PIN, PIN_MODE_INPUT_PULLDOWN);
    rt_pin_attach_irq(IMU1_INT_PIN, PIN_IRQ_MODE_RISING, IMU1_irq, RT_NULL);
    rt_pin_irq_enable(IMU1_INT_PIN, PIN_IRQ_ENABLE);

    //初始化中断数据接收线程
    IMU_SpiTrans = rt_thread_create(
        "IMU2_PRO",                     //线程名
        IMU2_SensorRAWProcess_thread,       //线程入口
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
    rt_pin_mode(IMU2_INT_PIN, PIN_MODE_INPUT_PULLDOWN);
    rt_pin_attach_irq(IMU2_INT_PIN, PIN_IRQ_MODE_RISING, IMU2_irq, RT_NULL);
    rt_pin_irq_enable(IMU2_INT_PIN, PIN_IRQ_ENABLE);
}

// ICM20602设备初始化 初始化后可通过接口读取已有最新数据，通过接口可以挂起等待新数据产生
int SensorRawProcess_Init()
{
    Sensor_RAW_IMU1.RawDataReady = 0;
    Sensor_RAW_IMU2.RawDataReady = 0;

    // 初始化用于控温的信号量，信号量在每次检测到温度数据变化后释放一个
	  rt_sem_init(&imu1_temp_pid_sem, "TP1_Sem", 0, RT_IPC_FLAG_FIFO);
    rt_sem_init(&imu2_temp_pid_sem, "TP2_Sem", 0, RT_IPC_FLAG_FIFO);
		
    // 初始化传感芯片
    ICM_init();

    // 启动硬触发数据接收
    HWTrig_init();
	
	return RT_EOK;
}
