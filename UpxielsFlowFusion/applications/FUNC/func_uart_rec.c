#include "func_uart_rec.h"
#include "drv_thread.h"
#include "drv_Resolve.h"
#include "drv_CustCtrler_Data.h"
#include <board.h>

// 磁编码器数据接收缓存区
uint8_t Rec_DataBuff[3][Data_len];
// 接收错误计数
uint16_t Rec_err_count[3];

/* 串口设备句柄 */
static rt_device_t Encoder_1_UartDev;
static rt_device_t Encoder_2_UartDev;
static rt_device_t Encoder_3_UartDev;

static rt_sem_t encoder_1_sem;
static rt_sem_t encoder_2_sem;
static rt_sem_t encoder_3_sem;

EncoderData_s Encoder1;
EncoderData_s Encoder2;
EncoderData_s Encoder3;

// 串口接收中断回调函数
static rt_err_t encoder_1_RecCallback(rt_device_t dev, rt_size_t size)
{
    rt_sem_release(encoder_1_sem);
    return RT_EOK;
}
static rt_err_t encoder_2_RecCallback(rt_device_t dev, rt_size_t size)
{
    rt_sem_release(encoder_2_sem);
    return RT_EOK;
}
static rt_err_t encoder_3_RecCallback(rt_device_t dev, rt_size_t size)
{
    rt_sem_release(encoder_3_sem);
    return RT_EOK;
}

/**
 * @brief 错误计数
 * @param encoder_num
 */
static void IncErrCount(Encoder_Num encoder_num)
{
    switch (encoder_num)
    {
    case encoder_1:
        Rec_err_count[0]++;
        break;
    case encoder_2:
        Rec_err_count[1]++;
        break;
    case encoder_3:
        Rec_err_count[2]++;
        break;
    }
}

/**
 * @brief 错误计数清零
 * @param encoder_num
 */
static void IncErrCountClear(Encoder_Num encoder_num)
{
    switch (encoder_num)
    {
    case encoder_1:
        Rec_err_count[0] = 0;
        break;
    case encoder_2:
        Rec_err_count[1] = 0;
        break;
    case encoder_3:
        Rec_err_count[2] = 0;
        break;
    }
}

/**
 * @brief
 * @param encoder_num
 */
void *EncoderData_Get(Encoder_Num encoder_num)
{
    switch (encoder_num)
    {
    case encoder_1:
        return (void *)&Encoder1;
    case encoder_2:
        return (void *)&Encoder2;
    case encoder_3:
        return (void *)&Encoder3;
    default:
        return NULL;
    }
}

/**
 * @brief 磁编码器接收数据处理函数
 * @param RecData_buff
 * @param encoder_num
 * @return rt_err_t
 */
rt_err_t RecData_Process(SendData_t *RecData_buff, Encoder_Num encoder_num)
{
    float data_temp;

    // 帧头帧尾校验
    if (RecData_buff->head[0] != 0xFD || RecData_buff->head[1] != 0xFE || RecData_buff->end != 0xFC)
    {
        // 相应错误计数+1
        IncErrCount(encoder_num);
        //方式DMA搬运错位后一直错位
	    if(Rec_err_count[0]>30||Rec_err_count[1]>30||Rec_err_count[2]>30)
		{
            __set_FAULTMASK(1);
            NVIC_SystemReset();
		}
        return RT_ERROR;
    }
    else
			{		//这里不能删
        IncErrCountClear(encoder_num);
    }
    /*换算编码器长度*/
    EncoderData_raw2L(RecData_buff->raw_angle,
                      EncoderData_Get(encoder_num));
    /*滤波*/
    data_temp = EncoderData_LowPass(EncoderData_Get(encoder_num));
    /*存入数据服务器*/
    CustCtrler_Data_L_Write_Single(data_temp, encoder_num);
    return RT_EOK;
}

// 磁编码器数据处理线程
static void encoder_1_thread(void *parameter)
{
    while (1)
    {
        rt_sem_take(encoder_1_sem, RT_WAITING_FOREVER);
        rt_device_read(Encoder_1_UartDev, 0, Rec_DataBuff[0], Data_len);
        // 接收数据的处理
        RecData_Process((SendData_t *)Rec_DataBuff[0], encoder_1);
    }
}
static void encoder_2_thread(void *parameter)
{
    while (1)
    {
        rt_sem_take(encoder_2_sem, RT_WAITING_FOREVER);
        rt_device_read(Encoder_2_UartDev, 0, Rec_DataBuff[1], Data_len);
        // 接收数据的处理
        RecData_Process((SendData_t *)Rec_DataBuff[1], encoder_2);
    }
}
static void encoder_3_thread(void *parameter)
{
    while (1)
    {
        rt_sem_take(encoder_3_sem, RT_WAITING_FOREVER);
        rt_device_read(Encoder_3_UartDev, 0, Rec_DataBuff[2], Data_len);
        // 接收数据的处理
        RecData_Process((SendData_t *)Rec_DataBuff[2], encoder_3);
    }
}

// 串口接收数据初始化
rt_err_t UART_REC_Init(void)
{
    rt_err_t res;
    rt_thread_t thread;

    EncoderData_Init(&Encoder1,
                     forward,
                     ENCODER1_FILTERRATE);
    EncoderData_Init(&Encoder2,
                     forward,
                     ENCODER2_FILTERRATE);
    EncoderData_Init(&Encoder3,
                     forward,
                     ENCODER3_FILTERRATE);

    // 电机接收数据信号量
    encoder_1_sem = rt_sem_create("encoder_1_sem", 0, RT_IPC_FLAG_PRIO);
    encoder_2_sem = rt_sem_create("encoder_2_sem", 0, RT_IPC_FLAG_PRIO);
    encoder_3_sem = rt_sem_create("encoder_3_sem", 0, RT_IPC_FLAG_PRIO);

    // 查找串口设备
    Encoder_1_UartDev = rt_device_find(Encoder_1_DevName);
    if (Encoder_1_UartDev == RT_NULL)
    {
        while (1)
            continue;
    }
    Encoder_2_UartDev = rt_device_find(Encoder_2_DevName);
    if (Encoder_2_UartDev == RT_NULL)
    {
        while (1)
            continue;
    }
    Encoder_3_UartDev = rt_device_find(Encoder_3_DevName);
    if (Encoder_3_UartDev == RT_NULL)
    {
        while (1)
            continue;
    }

    // 打开串口设备
    res = rt_device_open(Encoder_1_UartDev, RT_DEVICE_FLAG_DMA_RX);
    if (res != RT_EOK)
    {
        while (1)
            continue;
    }
    res = rt_device_open(Encoder_2_UartDev, RT_DEVICE_FLAG_DMA_RX);
    if (res != RT_EOK)
    {
        while (1)
            continue;
    }
    res = rt_device_open(Encoder_3_UartDev, RT_DEVICE_FLAG_DMA_RX);
    if (res != RT_EOK)
    {
        while (1)
            continue;
    }

    thread = rt_thread_create("encoder_1", encoder_1_thread,
                              RT_NULL, 1024,
                              THREAD_PRIO_ENCODER1_RX, 1);
    /* 创建成功则启动线程 */
    if (thread != RT_NULL)
    {
        rt_thread_startup(thread);
    }
    else
    {
        return RT_ERROR;
    }
    thread = rt_thread_create("encoder_2", encoder_2_thread,
                              RT_NULL, 1024,
                              THREAD_PRIO_ENCODER2_RX, 1);
    /* 创建成功则启动线程 */
    if (thread != RT_NULL)
    {
        rt_thread_startup(thread);
    }
    else
    {
        return RT_ERROR;
    }
    thread = rt_thread_create("encoder_3", encoder_3_thread,
                              RT_NULL, 1024,
                              THREAD_PRIO_ENCODER3_RX, 1);
    /* 创建成功则启动线程 */
    if (thread != RT_NULL)
    {
        rt_thread_startup(thread);
    }
    else
    {
        return RT_ERROR;
    }

    /* 设置接收回调函数 */
    rt_device_set_rx_indicate(Encoder_1_UartDev, encoder_1_RecCallback);
    rt_device_set_rx_indicate(Encoder_2_UartDev, encoder_2_RecCallback);
    rt_device_set_rx_indicate(Encoder_3_UartDev, encoder_3_RecCallback);

    return RT_EOK;
}
