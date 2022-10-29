/*
        BMI088 陀螺仪+加速度计传感器。
*/
#include "func_bmi088.h"
#include "func_TempCtr.h"

#include "drv_spithread.h"
#include "drv_thread.h"
#include "board.h"
#include "string.h"
#include "drv_HWTimer.h"
#include "drv_flash.h"
#include "drv_utils.h"

/* Private define ----------------------------------------------------------- */
#define GYRO_INT_PIN GET_PIN(C, 5)
#define ACCL_INT_PIN GET_PIN(C, 4)

#define BMI088_REG_ACCL_CHIP_ID (0x00)
#define BMI088_REG_ACCL_ERR (0x02)
#define BMI088_REG_ACCL_STATUS (0x03)
#define BMI088_REG_ACCL_X_LSB (0x12)
#define BMI088_REG_ACCL_X_MSB (0x13)
#define BMI088_REG_ACCL_Y_LSB (0x14)
#define BMI088_REG_ACCL_Y_MSB (0x15)
#define BMI088_REG_ACCL_Z_LSB (0x16)
#define BMI088_REG_ACCL_Z_MSB (0x17)
#define BMI088_REG_ACCL_SENSORTIME_0 (0x18)
#define BMI088_REG_ACCL_SENSORTIME_1 (0x19)
#define BMI088_REG_ACCL_SENSORTIME_2 (0x1A)
#define BMI088_REG_ACCL_INT_STAT_1 (0x1D)
#define BMI088_REG_ACCL_TEMP_MSB (0x22)
#define BMI088_REG_ACCL_TEMP_LSB (0x23)
#define BMI088_REG_ACCL_CONF (0x40)
#define BMI088_REG_ACCL_RANGE (0x41)
#define BMI088_REG_ACCL_INT1_IO_CONF (0x53)
#define BMI088_REG_ACCL_INT2_IO_CONF (0x54)
#define BMI088_REG_ACCL_INT1_INT2_MAP_DATA (0x58)
#define BMI088_REG_ACCL_SELF_TEST (0x6D)
#define BMI088_REG_ACCL_PWR_CONF (0x7C)
#define BMI088_REG_ACCL_PWR_CTRL (0x7D)
#define BMI088_REG_ACCL_SOFTRESET (0x7E)

#define BMI088_REG_GYRO_CHIP_ID (0x00)
#define BMI088_REG_GYRO_X_LSB (0x02)
#define BMI088_REG_GYRO_X_MSB (0x03)
#define BMI088_REG_GYRO_Y_LSB (0x04)
#define BMI088_REG_GYRO_Y_MSB (0x05)
#define BMI088_REG_GYRO_Z_LSB (0x06)
#define BMI088_REG_GYRO_Z_MSB (0x07)
#define BMI088_REG_GYRO_INT_STAT_1 (0x0A)
#define BMI088_REG_GYRO_RANGE (0x0F)
#define BMI088_REG_GYRO_BANDWIDTH (0x10)
#define BMI088_REG_GYRO_LPM1 (0x11)
#define BMI088_REG_GYRO_SOFTRESET (0x14)
#define BMI088_REG_GYRO_INT_CTRL (0x15)
#define BMI088_REG_GYRO_INT3_INT4_IO_CONF (0x16)
#define BMI088_REG_GYRO_INT3_INT4_IO_MAP (0x18)
#define BMI088_REG_GYRO_SELF_TEST (0x3C)

#define BMI088_CHIP_ID_ACCL (0x1E)
#define BMI088_CHIP_ID_GYRO (0x0F)

#define BMI088_LEN_RX_BUFF (19)
/* Private macro ------------------------------------------------------------ */
#define BMI088_ACCL_CS_SET() rt_pin_write(ACCL_CS_Pin,PIN_HIGH)
#define BMI088_ACCL_CS_RESET() rt_pin_write(ACCL_CS_Pin,PIN_LOW)

#define BMI088_GYRO_CS_SET() rt_pin_write(GYRO_CS_Pin,PIN_HIGH)
#define BMI088_GYRO_CS_RESET() rt_pin_write(GYRO_CS_Pin,PIN_LOW)

/* Private variables -------------------------------------------------------- */
static uint8_t Gyro_Wbuffer[2];
static uint8_t Gyro_Rbuffer[2];
static uint8_t Accl_Wbuffer[2];
static uint8_t Accl_Rbuffer[2];
static uint8_t bmi088_rxbuf[BMI088_LEN_RX_BUFF];

BMI088_t HERO_BMI088_DEV = {0};

/********** 基础通信部分 **********/

/**
* @brief BMI088单个数据读取函数
* @param [uint8_t] reg：读写命令
* @param [uint8_t] data：读/写的数据
* @return 无
* @author zzj
*/
static void GYRO_WriteSingle(uint8_t reg, uint8_t data)
{
    Gyro_Wbuffer[0] = (reg & 0x7f);
    Gyro_Wbuffer[1] = data;

    msgGYROWrite1.send_buf = Gyro_Wbuffer;
    msgGYROWrite2.send_buf = Gyro_Wbuffer + 1;
    
    rt_spi_take_bus(spi_dev_gyro);
    rt_spi_transfer_message(spi_dev_gyro, &msgGYROWrite1);
    rt_spi_release_bus(spi_dev_gyro);
}
static void ACCL_WriteSingle(uint8_t reg, uint8_t data)
{
    Accl_Wbuffer[0] = (reg & 0x7f);
    Accl_Wbuffer[1] = data;

    msgACCLWrite1.send_buf = Accl_Wbuffer;
    msgACCLWrite2.send_buf = Accl_Wbuffer + 1;
    
    rt_spi_take_bus(spi_dev_accl);
    rt_spi_transfer_message(spi_dev_accl, &msgACCLWrite1);
    rt_spi_release_bus(spi_dev_accl);
}

static uint8_t GYRO_ReadSingle(uint8_t reg)
{
    Gyro_Rbuffer[0] = (uint8_t)(reg | 0x80);
    Gyro_Rbuffer[1] = 0;

    msgGYRORead1.send_buf = Gyro_Rbuffer;
    msgGYRORead2.recv_buf = Gyro_Rbuffer + 1;

    rt_spi_take_bus(spi_dev_gyro);
    rt_spi_transfer_message(spi_dev_gyro, &msgGYRORead1);
    rt_spi_release_bus(spi_dev_gyro);
    return Gyro_Rbuffer[1];
}
static uint8_t ACCL_ReadSingle(uint8_t reg)
{
    Accl_Rbuffer[0] = (uint8_t)(reg | 0x80);
    Accl_Rbuffer[1] = 0;

    msgACCLRead1.send_buf = Accl_Rbuffer;
    msgACCLRead3.recv_buf = Accl_Rbuffer + 1;

    rt_spi_take_bus(spi_dev_accl);
    rt_spi_transfer_message(spi_dev_accl, &msgACCLRead1);
    rt_spi_release_bus(spi_dev_accl);
    return Accl_Rbuffer[1];
}

/**
* @brief BMI088的角速度加速度与温度读取
* @param 无
* @return 无
* @author zzj
*/
static void BMI_Read_Gyro()
{
    bmi088_rxbuf[6] = GYRO_ReadSingle(BMI088_REG_GYRO_X_LSB);
    bmi088_rxbuf[7] = GYRO_ReadSingle(BMI088_REG_GYRO_X_MSB);
    
    bmi088_rxbuf[8] = GYRO_ReadSingle(BMI088_REG_GYRO_Y_LSB);
    bmi088_rxbuf[9] = GYRO_ReadSingle(BMI088_REG_GYRO_Y_MSB);
    
    bmi088_rxbuf[10] = GYRO_ReadSingle(BMI088_REG_GYRO_Z_LSB);
    bmi088_rxbuf[11] = GYRO_ReadSingle(BMI088_REG_GYRO_Z_MSB);

}
static void BMI_Read_Accl()
{
    bmi088_rxbuf[0] = ACCL_ReadSingle(BMI088_REG_ACCL_X_LSB);
    bmi088_rxbuf[1] = ACCL_ReadSingle(BMI088_REG_ACCL_X_MSB);

    bmi088_rxbuf[2] = ACCL_ReadSingle(BMI088_REG_ACCL_Y_LSB);
    bmi088_rxbuf[3] = ACCL_ReadSingle(BMI088_REG_ACCL_Y_MSB);
    
    bmi088_rxbuf[4] = ACCL_ReadSingle(BMI088_REG_ACCL_Z_LSB);
    bmi088_rxbuf[5] = ACCL_ReadSingle(BMI088_REG_ACCL_Z_MSB);
}
static void BMI_Read_Temp()
{
    bmi088_rxbuf[12] = ACCL_ReadSingle(BMI088_REG_ACCL_TEMP_LSB);
    bmi088_rxbuf[13] = ACCL_ReadSingle(BMI088_REG_ACCL_TEMP_MSB);
}

// 通信读取加速度数据
static int8_t BMI088_ParseAccl()
{
    BMI_Read_Accl();

    int16_t raw_x, raw_y, raw_z;
    raw_x = *((int16_t *)(bmi088_rxbuf));
    raw_y = *((int16_t *)(bmi088_rxbuf + 2));
    raw_z = *((int16_t *)(bmi088_rxbuf + 4));

    /* 3G: 10920. 6G: 5460. 12G: 2730. 24G: 1365. */
    HERO_BMI088_DEV.Accl_Raw.x = (float)raw_x / 5460.0f;
    HERO_BMI088_DEV.Accl_Raw.y = (float)raw_y / 5460.0f;
    HERO_BMI088_DEV.Accl_Raw.z = (float)raw_z / 5460.0f;

    return RT_EOK;
}

static const double Gyro_UnitFix = 1 / 32.768 / 360 * 2 * 3.14159265;
// 通信读取角速度数据
static int8_t BMI088_ParseGyro()
{
    BMI_Read_Gyro();

    /* Gyroscope imu_raw -> degrees/sec -> radians/sec */
    int16_t raw_x, raw_y, raw_z;
    raw_x = *((int16_t *)(bmi088_rxbuf + 6));
    raw_y = *((int16_t *)(bmi088_rxbuf + 8));
    raw_z = *((int16_t *)(bmi088_rxbuf + 10));

    /* FS125: 262.144. FS250: 131.072. FS500: 65.536. FS1000: 32.768.
    * FS2000: 16.384.*/
    // 单位：弧度/s
    HERO_BMI088_DEV.Gyro_Raw.x = (float)raw_x * Gyro_UnitFix;
    HERO_BMI088_DEV.Gyro_Raw.y = (float)raw_y * Gyro_UnitFix;
    HERO_BMI088_DEV.Gyro_Raw.z = (float)raw_z * Gyro_UnitFix;

    return RT_EOK;
}

// 通信读取温度数据
static int8_t BMI088_ParseTemp()
{
    static float last_temp = 0;
    int16_t raw_temp;

    BMI_Read_Temp();
    raw_temp = (uint16_t)((bmi088_rxbuf[13] << 3) | (bmi088_rxbuf[12] >> 5));

    if (raw_temp > 1023)
        raw_temp -= 2048;

    HERO_BMI088_DEV.Temperature = (float)raw_temp * 0.125f + 23.0f;

    if (HERO_BMI088_DEV.Temperature != last_temp)
    { // 检测到温度数据更新
        while (rt_sem_trytake(&temp_pid_sem) == RT_EOK)
            ;                          // 清空多余的信号量
        rt_sem_release(&temp_pid_sem); // 重新释放信号量
    }

    last_temp = HERO_BMI088_DEV.Temperature;
    
    return RT_EOK;
}

static int TempReadScale = 100;
// 刷新角速度、加速度、温度
static void BMI088_FreshData()
{
    BMI088_ParseGyro();
    BMI088_ParseAccl();
    if (TempReadScale>0)
        TempReadScale--;
    else
    {
        BMI088_ParseTemp();
        TempReadScale = 100;
    }
    HERO_BMI088_DEV.RawDataReady = 1;
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
    if (DeltaCount < 0)
    { // 计数值跨圈处理
        DeltaCount += 65536;
    }
    IMUFrqFilter = 21000000.f / DeltaCount;
    LastCount = NowCount;

    if (FirstFilterFlag || isnan(IMUFrqNow))
    {
        FirstFilterFlag = 0;
        IMUFrqNow = IMUFrqFilter;
        HERO_BMI088_DEV.RawDataReady = 0;
    }
    else
    {
        IMUFrqNow = IMUFrqNow * 0.99f + IMUFrqFilter * 0.01f; // 滞后滤波
        HERO_BMI088_DEV.DataRate = IMUFrqNow;
        rt_event_send(&IMU_Event, EVT_GYRO);
    }
}

// IMU触发式数据接收线程
static void IMU_DataCollect_thread(void *Para)
{
    rt_uint32_t EVT_recv;

    while (1)
    {
        // 等待数据产生
        rt_event_recv(&IMU_Event, EVT_GYRO,
                      RT_EVENT_FLAG_OR | RT_EVENT_FLAG_CLEAR, 2, &EVT_recv);

        if (EVT_recv == 0)
        {
            rt_thread_delay(1); // 出问题了
            continue;
        }

        BMI088_FreshData();

        // 取完可能存在的堆积的信号量
        while (rt_sem_trytake(&IMU_CALTrig_Sem) == RT_EOK)
            ;
        // 发送信号量，触发姿态融合算法
        rt_sem_release(&IMU_CALTrig_Sem);
    }
}

// 挂起在信号量上，等待新数据接收完毕
void BMI088_WaitForRawData()
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
        IMU_DataCollect_thread,       //线程入口
        RT_NULL,                      //入口参数无
        1024,                         //线程栈
        THREAD_PRIO_IMU_DATA_COLLECT, //线程优先级
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
    rt_pin_mode(GYRO_INT_PIN, PIN_MODE_INPUT_PULLDOWN);
    rt_pin_attach_irq(GYRO_INT_PIN, PIN_IRQ_MODE_FALLING, Gyro_irq, RT_NULL);
    rt_pin_irq_enable(GYRO_INT_PIN, PIN_IRQ_ENABLE);
}

volatile int BMI088_Accl_ID_Read = 0;
// BMI088中的寄存器数据初始化，即芯片内部设置初始化
static void bmi088_RegInit()
{
    rt_thread_delay(30);
    ACCL_WriteSingle(BMI088_REG_ACCL_SOFTRESET, 0xB6);
    GYRO_WriteSingle(BMI088_REG_GYRO_SOFTRESET, 0xB6);
    rt_thread_delay(30);

    /* Switch accl to SPI mode. */
    BMI088_Accl_ID_Read = ACCL_ReadSingle(BMI088_CHIP_ID_ACCL);

    if (ACCL_ReadSingle(BMI088_REG_ACCL_CHIP_ID) != BMI088_CHIP_ID_ACCL)
        return;

    if (GYRO_ReadSingle(BMI088_REG_GYRO_CHIP_ID) != BMI088_CHIP_ID_GYRO)
        return;

    /* Accl init. */
    /* Filter setting: Normal. */
    /* ODR: 0xAB: 800Hz. 0xAA: 400Hz. 0xA9: 200Hz. 0xA8: 100Hz. 0xA6: 25Hz. */
    /* ODR: 0x9C: 3dB--234Hz , Output Data Rate--1600Hz */
    ACCL_WriteSingle(BMI088_REG_ACCL_CONF, 0x8B);

    /* 0x00: +-3G. 0x01: +-6G. 0x02: +-12G. 0x03: +-24G. */
    ACCL_WriteSingle(BMI088_REG_ACCL_RANGE, 0x01);

    /* INT1 as output. Push-pull. Active low. Output. */
    ACCL_WriteSingle(BMI088_REG_ACCL_INT1_IO_CONF, 0x08);

    /* Map data ready interrupt to INT1. */
    ACCL_WriteSingle(BMI088_REG_ACCL_INT1_INT2_MAP_DATA, 0x04);

    /* Turn on accl. Now we can read data. */
    ACCL_WriteSingle(BMI088_REG_ACCL_PWR_CTRL, 0x04);
    rt_thread_delay(5);

    /* Gyro init. */
    /* 0x00: +-2000. 0x01: +-1000. 0x02: +-500. 0x03: +-250. 0x04: +-125. */
    GYRO_WriteSingle(BMI088_REG_GYRO_RANGE, 0x01);

    /* Filter bw: 47Hz. */
    /* ODR: 0x02: 1000Hz. 0x03: 400Hz. 0x06: 200Hz. 0x07: 100Hz. */
    GYRO_WriteSingle(BMI088_REG_GYRO_BANDWIDTH, 0x02);

    /* INT3 and INT4 as output. Push-pull. Active low. */
    GYRO_WriteSingle(BMI088_REG_GYRO_INT3_INT4_IO_CONF, 0x00);

    /* Map data ready interrupt to INT3. */
    GYRO_WriteSingle(BMI088_REG_GYRO_INT3_INT4_IO_MAP, 0x01);

    /* Enable new data interrupt. */
    GYRO_WriteSingle(BMI088_REG_GYRO_INT_CTRL, 0x80);

    rt_thread_delay(5);
}

// BMI088设备初始化 初始化后可通过接口读取已有最新数据，通过接口可以挂起等待新数据产生
rt_err_t BMI088_Init(void)
{
    HERO_BMI088_DEV.RawDataReady = 0;

    // 初始化用于控温的信号量，信号量在每次检测到温度数据变化后释放一个
	rt_sem_init(&temp_pid_sem, "TP_Sem", 0, RT_IPC_FLAG_FIFO);

    // 初始化SPI通信
    spi_BMI088_init();

    // 初始化BMI088的内部设置
    bmi088_RegInit();

    // 启动硬触发数据接收
    HWTrig_init();

    return RT_EOK;
}
