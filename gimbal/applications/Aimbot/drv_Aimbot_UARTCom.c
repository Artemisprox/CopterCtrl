/**
 * @file drv_Aimbot_UARTCom.c
 * @brief 本文件用于处理与视觉通信的串口发送和接收数据
 * (这里在原二代自瞄的基础上重新规定了一套串口专用的通信协议)
 * 串口的通信波特率 115200 每一帧数据分为三个部分：
 * 第一字节固定为 0xFC 作为数据帧头
 * 第二字节的高第一位作为区分数据类型的标志位(1 代表姿态数据, 0 代表标志位/对时数据)
 * 第二字节其余部分作为数据真实长度(即不包含本两个字节)
 * 其余内容保持为 CAN 的通信报文的数据段, 但是为了与帧头相互区分, 遇到数据 0xFC 必须连续发送两遍
 * @author fwlh
 * @version 1.1
 * @date 2022-06-21
 *
 * @copyright Copyright (c) 2022  哈尔滨工业大学(威海)HERO战队
 */

#include "drv_Aimbot_UARTCom.h"
#include <rtdevice.h>
#include <board.h>
#include "drv_thread.h"
#include "drv_utils.h"

#ifndef AIMBOT_CIMMUNICATION_USING_CAN

#define AIMBOT_RX_READ_SIZE RT_SERIAL_RB_BUFSZ // 接收缓冲区大小
#define AIMBOT_TX_BUFFER_SIZE 15               // 发送缓冲区大小(双段缓冲)
#define AIMBOT_RX_REAL_DATA_SIZEMAX 20         // 收到一帧数据的最大大小

struct UART_Msg
{
    rt_device_t *device;          // 本文件使用的串口设备
    rt_thread_t rx_handle_thread; // 用于处理接收到数据的线程句柄
    rt_sem_t rx_sem;              // 用于记录数据已经被接收的信号量

    rt_size_t current_receive_size;                          // 当前接收到数据的大小
    rt_int8_t current_left_size;                             // 当前剩下的仍待处理的数据
    rt_size_t current_used_size;                             // 本次使用完毕的数据量
    rt_uint8_t rx_msg[AIMBOT_RX_READ_SIZE];                  // 直接接收的数据位置
    rt_uint8_t rx_msg_unpacked[AIMBOT_RX_REAL_DATA_SIZEMAX]; // 解包(指串口协议包)后的通信数据
    rt_uint8_t tx_msg[2][AIMBOT_TX_BUFFER_SIZE];             // 待发送的实际数据, 开辟两端缓冲防止写入时数据覆盖

    rt_uint8_t tx_data_to_write; // 当前该写入到哪个串口发送缓冲区(tx_msg[0/1])
    rt_uint8_t tx_complete_flag; // 串口发送完成标志

    rt_err_t (*flag_get)(rt_uint8_t rxmsg[]); // 标志位报文的处理函数
    rt_err_t (*atti_get)(rt_uint8_t rxmsg[]); // 云台姿态报文的处理函数
} Aimbot_UART_Msg = {0};

/**
 * @brief 用于将待发送的数据以符合通信协议的方式发送出去
 * @author fwlh
 * @param  dev              串口设备
 * @param  ID               信息 ID, 1 代表姿态, 0 代表标志位
 * @param  msg              真实数据(数组名)
 * @param  size             数组中数据的数量
 * @return rt_size_t        返回写入数据的数量
 */
rt_size_t Aimbot_Write_UART_Data(rt_device_t *dev, rt_uint8_t ID, rt_uint8_t msg[], rt_size_t size)
{
    // 用于记录当前写到缓存区的哪个位置了, 这里默认从 2 开始是因为前两字节是串口通信协议固定的内容
    rt_uint8_t write_pos = 2;
    // 用于指定当前该写入哪一段缓冲数据, 如果当前数据还没发送完就使用另一段缓冲区
    if (!Aimbot_UART_Msg.tx_complete_flag)
        Aimbot_UART_Msg.tx_data_to_write = 1 - Aimbot_UART_Msg.tx_data_to_write;
    rt_uint8_t *write_msg = Aimbot_UART_Msg.tx_msg[Aimbot_UART_Msg.tx_data_to_write];

    // 写入帧头
    write_msg[0] = 0xFC;
    // 遍历发送缓存区写入待发送数据
    for (int i = 0; i < size; ++i)
    {
        // 如果遇到 0xFC 就需要发两遍
        if (0xFC == msg[i])
        {
            write_msg[write_pos++] = 0xFC;
            write_msg[write_pos++] = 0xFC;
        }
        else
            write_msg[write_pos++] = msg[i];
    }
    // 写入数据实际长度(不包括前两字节)与数据类型
    if (1 == ID)
        write_msg[1] = ((write_pos - 2) & 0x7F) | (ID << 7);
    else
        write_msg[1] = ((write_pos - 3) & 0x7F) | (ID << 7);

    // 发送数据
    Aimbot_UART_Msg.tx_complete_flag = 0;
    if (!dev)
        return 0;
    else
        return rt_device_write(*dev, 0, write_msg, (write_msg[1] & 0x7F) + 2);
}

/**
 * @brief 在数组中寻找第一个独立(它的前后均不和他相等)的被指定数据
 * @author fwlh
 * @param  data             待查找数据的数组
 * @param  start            待查找目标在数组中的起始位置(包含)
 * @param  end              待查找目标在数组中的结束位置(不包含)
 * @param  target           寻找的目标
 * @return int              返回值为非负数代表找到的目标数据下标, 返回 -1 代表未找到
 */
static int Find_First_Indenpendent(const uint8_t data[], const uint8_t start, const uint8_t end, const uint8_t target)
{
    int index = start;
    // 开始遍历查找
    for (; index < end; ++index)
    {
        // 如果它是开头就只需要关注右侧
        if ((!index) || (start == index))
        {
            if ((target == data[index]) && (target != data[index + 1]))
                break;
        }
        else if (index + 1 != end)
        { // 限制条件防止数组越界
            if ((target != data[index - 1]) && (target == data[index]) && (target != data[index + 1]))
                break;
        }
    }

    // 返回结果
    if (index < end)
        return index;
    else
        return -1;
}

/**
 * @brief 将数组中连续重复出现的数字删掉放到另一个数组中
 * @author fwlh
 * @param  dst              目标位置
 * @param  src              源数据循环队列数组(不会被修改)
 * @param  start            检查数组的起始位置
 * @param  end              检查数组的结束位置
 * @param  target           检查的数字
 */
static void Remove_Continuous_Identical_Data(uint8_t dst[], const uint8_t src[], const uint8_t start, const uint8_t end, const uint8_t target)
{
    // 定义当前操作到哪个位置了
    int dst_i = 0, src_i = start;
    // 遍历写入数据
    for (; src_i < end; ++src_i)
    {
        if ((target == src[src_i]) && (target == src[src_i + 1]))
        {
            src_i += 1;
            dst[dst_i++] = target;
        }
        else
            dst[dst_i++] = src[src_i];
    }
}

/**
 * @brief 将收到的数据解包成原始通信协议数据
 * @author fwlh
 * @param  rx_msg           收到的原始数据
 * @param  unpacked         串口包解开以后的数据存放的位置
 * @param  size             检查数组的大小
 * @return int              本次被处理为正常数据的字节数
 */
static int Aimbot_Read_UART_Data(const rt_uint8_t rx_msg[], rt_uint8_t unpacked[], const uint8_t size)
{
    int fori = 0;            // 用于记录收到的数据中实际数据的起始位置(独立 0xFC 所在的位置)
    int last_frame_end = 0;  // 用于记录上一帧数据结束的下标位置
    rt_err_t err = RT_ERROR; // 用于记录本次解包是否成功

    // 不断寻找独立 0xFC 直至无法找到
    while (-1 != (fori = Find_First_Indenpendent(rx_msg, fori, size, 0xFC)))
    {
        err = RT_ERROR;
        // 将数据写到缓存区
        Remove_Continuous_Identical_Data(
            unpacked, rx_msg + fori + 2, 0, utils_min_2_int((rx_msg[fori + 1] & 0x7F), AIMBOT_RX_REAL_DATA_SIZEMAX), 0xFC);
        // 判断数据类型, 第二字节第一位为 1 代表姿态数据
        if ((rx_msg[fori + 1] & 0x80) >> 7)
            err = Aimbot_UART_Msg.atti_get(unpacked);
        else
            err = Aimbot_UART_Msg.flag_get(unpacked);
        // 判断本次解包是否成功
        if (RT_EOK == err)
        {
            // 继续检查剩余的数据内是否存在新的报文
            fori += ((rx_msg[fori + 1] & 0x7F) + 1);
            // 更新一帧数据结束的位置
            last_frame_end = fori;
        }
        else
            // 没有成功解包就直接从下一字节开始继续寻找
            ++fori;
        if (fori >= size)
            break;
    }
    return last_frame_end;
}

/**
 * @brief 串口的接收中断回调函数
 * @author fwlh
 * @param  dev              设备名
 * @param  size             数据大小
 * @return rt_err_t         不会被用户使用
 */
static rt_err_t Aimbot_RX_Callback(rt_device_t dev, rt_size_t size)
{
    rt_sem_release(Aimbot_UART_Msg.rx_sem);
    return RT_EOK;
}

/**
 * @brief 串口发送完成中断回调函数
 * @author fwlh
 * @param  dev              发送完成的设备
 * @param  buffer           缓冲区
 * @return rt_err_t         暂时不会被使用
 */
static rt_err_t Aimbot_TXComplete_Callback(rt_device_t dev, void *buffer)
{
    Aimbot_UART_Msg.tx_complete_flag = 1;
    return RT_EOK;
}

/**
 * @brief 用于处理来自视觉的串口数据的线程
 * @author fwlh
 * @param  param            线程的入口参数, 暂时未被使用
 */
static void Aimbot_RX_Thread(void *param)
{
    // 初始化相关的标志位等
    Aimbot_UART_Msg.current_left_size = 0;
    Aimbot_UART_Msg.tx_complete_flag = 0;
    Aimbot_UART_Msg.current_used_size = 0;
    Aimbot_UART_Msg.current_left_size = 0;
    Aimbot_UART_Msg.current_receive_size = 0;
    int temp_left_size = 0;
    // 等待信号量被初始化
    while (Aimbot_UART_Msg.rx_sem == RT_NULL)
    {
        rt_thread_mdelay(1);
        continue;
    }
    // 线程正常运行
    while (1)
    {
        // 等待接收到数据的信号量
        rt_sem_take(Aimbot_UART_Msg.rx_sem, RT_WAITING_FOREVER);
        // 尝试获取串口数据, 同时记录本次获到的数据量
        Aimbot_UART_Msg.current_receive_size = rt_device_read(
            *Aimbot_UART_Msg.device, 0, Aimbot_UART_Msg.rx_msg + Aimbot_UART_Msg.current_left_size,
            AIMBOT_RX_READ_SIZE - Aimbot_UART_Msg.current_left_size);
        // 处理通信数据, 并记录本次使用完毕的数据量
        Aimbot_UART_Msg.current_used_size = Aimbot_Read_UART_Data(
            Aimbot_UART_Msg.rx_msg, Aimbot_UART_Msg.rx_msg_unpacked,
            Aimbot_UART_Msg.current_receive_size + Aimbot_UART_Msg.current_left_size);
        // 将剩余数据搬运到数组起始位置
        temp_left_size = Aimbot_UART_Msg.current_receive_size - Aimbot_UART_Msg.current_used_size + Aimbot_UART_Msg.current_left_size;
        temp_left_size = (temp_left_size > 0) ? temp_left_size : 0;
        for (int pos = 0; pos < temp_left_size; ++pos)
            Aimbot_UART_Msg.rx_msg[pos] = Aimbot_UART_Msg.rx_msg[Aimbot_UART_Msg.current_used_size + pos];
        Aimbot_UART_Msg.current_left_size = temp_left_size;
        // 剩余的待处理的数据过多就直接扔掉
        if (Aimbot_UART_Msg.current_left_size > AIMBOT_RX_REAL_DATA_SIZEMAX * 1.5)
            Aimbot_UART_Msg.current_left_size -= AIMBOT_RX_REAL_DATA_SIZEMAX;
    }
}

/**
 * @brief 初始化与视觉通信的串口
 * @author fwlh
 * @param  aimbot_device    设备指针
 * @param  flag_get         处理标志位报文的函数
 * @param  atti_get         处理云台姿态报文的函数
 * @return rt_err_t         初始化结果
 */
rt_err_t Aimbot_UART_Init(rt_device_t *aimbot_device, rt_err_t (*flag_get)(rt_uint8_t rxmsg[]), rt_err_t (*atti_get)(rt_uint8_t rxmsg[]))
{
#ifndef BSP_USING_UART6
#error "aimbot com: uart6 not opened!"
#endif
    // 使用 NUC 时需要初始化串口6
    *aimbot_device = rt_device_find("uart6"); //使用默认的串口配置，配置为波特率 115200,8位数据位,1位停止位,无校验位
    if (!(*aimbot_device))
    {
        rt_kprintf("find aimbot uart device failed !\n");
        return RT_ERROR;
    }
    // 修改串口配置参数
    struct serial_configure config = {0};
    config.baud_rate = BAUD_RATE_115200; //修改波特率为 115200
    config.data_bits = DATA_BITS_8;      //数据位 8
    config.stop_bits = STOP_BITS_1;      //停止位 1
    config.bufsz = AIMBOT_RX_READ_SIZE;  //修改缓冲区 buff size 为 120
    config.parity = PARITY_NONE;         //无奇偶校验位
    // 控制串口设备 通过控制接口传入命令控制字，与控制参数
    if (rt_device_control(*aimbot_device, RT_DEVICE_CTRL_CONFIG, &config) != RT_EOK)
        return RT_ERROR;
#if ((!defined(AIMBOT_CIMMUNICATION_USING_CAN)) && (!((defined(BSP_UART6_RX_USING_DMA)) && (defined(BSP_UART6_TX_USING_DMA)))))
#error "aimbot com: uart6 DMA not opened!"
#endif
    // 使用 DMA 发送与 DMA 接收模式
    if (rt_device_open(*aimbot_device, RT_DEVICE_FLAG_DMA_TX | RT_DEVICE_FLAG_DMA_RX) != RT_EOK)
        return RT_ERROR;
    // 定义串口接收中断回调函数
    if (rt_device_set_rx_indicate(*aimbot_device, Aimbot_RX_Callback) != RT_EOK)
        return RT_ERROR;
    // 定义串口发送完成回调函数
    if (rt_device_set_tx_complete(*aimbot_device, Aimbot_TXComplete_Callback) != RT_EOK)
        return RT_ERROR;
    // 记录串口设备句柄
    Aimbot_UART_Msg.device = aimbot_device;

    // 初始化接收数据处理信号量
    Aimbot_UART_Msg.rx_sem = rt_sem_create("uart rx sem", 0, RT_IPC_FLAG_FIFO);
    if (Aimbot_UART_Msg.rx_sem == RT_NULL)
        return RT_ERROR;
    // 初始化接收处理线程
    Aimbot_UART_Msg.rx_handle_thread = rt_thread_create("uart rx thread", Aimbot_RX_Thread, RT_NULL, 1024, THREAD_PRIO_AIMBOT_UART_RX, 1);
    if (Aimbot_UART_Msg.rx_handle_thread != RT_NULL)
        rt_thread_startup(Aimbot_UART_Msg.rx_handle_thread);
    else
        return RT_ERROR;

    // 写入相关函数指针
    Aimbot_UART_Msg.flag_get = flag_get;
    Aimbot_UART_Msg.atti_get = atti_get;

    return RT_EOK;
}

#endif /* AIMBOT_CIMMUNICATION_USING_CAN */
