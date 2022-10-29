#include "mod_RefSystem.h"
#include "HThread_data.h"
#include "drv_CRC.h"
#include "app_GetRef.h"
#include "mod_Monitor.h"
#include "app_GetGim.h"

/* 对裁判系统解析后的数据存放结构体 */
DJI_Data_t DJI_ReadData;

/* rtt串口驱动 */
struct DJI_Mxg
{
    rt_device_t dev;
    rt_size_t size;
};
static struct rt_messagequeue DJI_mq;
static rt_uint8_t msg_pool[512];
static rt_uint8_t DJI_buffer[RT_SERIAL_RB_BUFSZ + 1];

RefReceiveTime_s RefReceiveTime;

uint8_t Debug_UART_Enable = 1;

/***
 * @brief	对串口6的数据进行解析
 * @param	pData: [输入/出]
 *			DJI_ReadData: 解析后存放信息的结构体
 * @return	成功or失败
 ***/
static rt_err_t DJI_DataProcess(uint8_t *pData, DJI_Data_t *DJI_ReadData)
{
    uint8_t length;

    // pData为空返回
    if (pData == NULL)
    {
        return RT_ERROR;
    }
    //判断帧头是否为0xA5
    if (pData[0] == 0xA5)
    {
        Frame_t *tempData; //暂存数据
        tempData = (Frame_t *)(&pData[0]);

        length = tempData->FrameHeader.DataLength;
        // 判断是否会出现数组越界
        if ((uint32_t)(pData + sizeof(FrameHeader_t) + LEN_CMDID + length + LEN_TAIL) >= (uint32_t)&DJI_buffer[RT_SERIAL_RB_BUFSZ])
            return RT_ERROR;

        if (Verify_CRC8_Check_Sum(&pData[0], LEN_HEADER) && Verify_CRC16_Check_Sum(&pData[0], length + LEN_HEADER + LEN_CMDID + LEN_TAIL))
        {
            DJI_ReadData->FrameHeader = tempData->FrameHeader;
            DJI_ReadData->CmdID = tempData->CmdID;
            switch (tempData->CmdID)
            {
            case ID_game_state: // 0x0001
                DJI_ReadData->ext_game_state = tempData->Data.ext_game_state;
                break;
            case ID_game_result: // 0x0002
                DJI_ReadData->ext_game_result = tempData->Data.ext_game_result;
                break;
            case ID_game_robot_survivors: // 0x0003
                DJI_ReadData->ext_game_robot_survivors = tempData->Data.ext_game_robot_survivors;
                Ref_Bulid_If_Hurt(&DJI_ReadData->ext_game_robot_survivors);
                break;
            case ID_dart_status: // 0x0004
                DJI_ReadData->ext_dart_status = tempData->Data.ext_dart_status;
                break;
            case ID_ICRA_buff_status: // 0x0005
                DJI_ReadData->ext_ICRA_buff_status = tempData->Data.ext_ICRA_buff_status;
                break;
            case ID_event_data: // 0x0101
                DJI_ReadData->ext_event_data = tempData->Data.ext_event_data;
                break;
            case ID_supply_projectile_action: // 0x0102
                DJI_ReadData->ext_supply_projectile_action = tempData->Data.ext_supply_projectile_action;
                break;
            /*case ID_supply_projectile_booking://0x0103
                DJI_ReadData->ext_supply_projectile_booking = tempData->Data.ext_supply_projectile_booking;
                break;
                */
            case ID_referee_warning: // 0x0104
                DJI_ReadData->ext_referee_warning = tempData->Data.ext_referee_warning;
                break;
            case ID_dart_remaining_time: // 0x0105
                DJI_ReadData->ext_dart_remaining_time = tempData->Data.ext_dart_remaining_time;
                break;
            case ID_game_robot_state: // 0x0201
                RefReceiveTime.game_robot_state = rt_tick_get();
                DJI_ReadData->ext_game_robot_state = tempData->Data.ext_game_robot_state;
                break;
            case ID_power_heat_data: // 0x0202
                RefReceiveTime.power_heat_data = rt_tick_get();
                DJI_ReadData->ext_power_heat_data = tempData->Data.ext_power_heat_data;
                break;
            case ID_game_robot_pos: // 0x0203
                DJI_ReadData->ext_game_robot_pos = tempData->Data.ext_game_robot_pos;
                break;
            case ID_buff_musk: // 0x0204
                RefReceiveTime.buff_musk = rt_tick_get();
                DJI_ReadData->ext_buff_musk = tempData->Data.ext_buff_musk;
                break;
            case ID_aerial_robot_energy: // 0x0205
                DJI_ReadData->aerial_robot_energy = tempData->Data.aerial_robot_energy;
                break;
            case ID_robot_hurt: // 0x0206
                DJI_ReadData->ext_robot_hurt = tempData->Data.ext_robot_hurt;
                Ref_Robot_Set_Hurt();
                break;
            case ID_shoot_data: // 0x0207
                RefReceiveTime.shoot_data = rt_tick_get();
                DJI_ReadData->ext_shoot_data = tempData->Data.ext_shoot_data;
                break;
            case ID_bullet_remaining: // 0x0208
                DJI_ReadData->ext_bullet_remaining = tempData->Data.ext_bullet_remaining;
                break;
            case ID_rfid_status: // 0x0209
                DJI_ReadData->ext_rfid_status = tempData->Data.ext_rfid_status;
                break;
            case ID_dart_client_cmd: // 0x020A
                DJI_ReadData->ext_dart_client_cmd = tempData->Data.ext_dart_client_cmd;
                break;
            case ID_student_interactive_data: // 0x0301
                DJI_ReadData->ext_send_user_data = tempData->Data.ext_send_user_data;
                break;
            /*case ID_robot_interactive_data: 	//0x302
                DJI_ReadData->ext_robot_interactive_data = tempData->Data.ext_robot_interactive_data;
                break;*/
            case ID_robot_command_map: // 0x303
                DJI_ReadData->ext_robot_command_map = tempData->Data.ext_robot_command_map;
                break;
            case ID_robot_command_remote: // 0x304
                DJI_ReadData->ext_robot_command_remote = tempData->Data.ext_robot_command_remote;
                break;
            default:
                break;
            } // switch
        }
        if (*(pData + sizeof(FrameHeader_t) + LEN_CMDID + length + LEN_TAIL) == 0xA5)
        {
            //如果有多帧数据递归读取
            DJI_DataProcess(pData + sizeof(FrameHeader_t) + LEN_CMDID + length + LEN_TAIL, DJI_ReadData);
        }
    }
    return RT_EOK;
}

/***
 * @brief	串口回调
 * @param	dev 设备，size 字节数
 * @return	成功or失败
 ***/
static rt_err_t DJI_Callback(rt_device_t dev, rt_size_t size)
{
    struct DJI_Mxg msg;
    rt_err_t result;
    msg.dev = dev;
    msg.size = size;

    result = rt_mq_send(&DJI_mq, &msg, sizeof(msg));
    if (result == -RT_EFULL)
        rt_kprintf("DJI: message queue full\n");
    return result;
}

/***
 * @brief:   接收裁判系统的帧数据并进行解析的线程
 *			接收周期不定,有1Hz的，10Hz的，50Hz的等，具体看裁判系统手册
 * @param:   parameter: None
 * @return:  None
 ***/
static void DJI_Process_thread(void *parameter)
{
    struct DJI_Mxg msg;
    rt_err_t result;
    SWDG_START(SWDG_REFSYSTEM_ID);

    while (1)
    {
        rt_memset(&msg, 0, sizeof(msg));
        result = rt_mq_recv(&DJI_mq, &msg, sizeof(msg), 500); //等待回调函数的消息队列

        if (result == RT_EOK)
        {
            rt_device_read(msg.dev, 0, DJI_buffer, msg.size);

            // 云台控制命令有效且信任裁判系统数据
            if ((Debug_UART_Enable) &&
                ((!Get_Ref_Offline_Cmd()) && (rt_tick_get() - Get_Gim_FreshTick() < 200) && (Get_Gim_FreshTick())))
                DJI_DataProcess(DJI_buffer, &DJI_ReadData); //对裁判系统数据进行处理
            rt_memset(&DJI_buffer, 0, sizeof(DJI_buffer));
        }

        SWDG_FEED(SWDG_REFSYSTEM_ID);
    }
}

/**
 * @brief: 初始化裁判系统
 * @param {None}
 * @return {*}
 */
rt_err_t DJI_Init(void)
{
    rt_err_t res = RT_EOK;
    rt_device_t DJI_Serial;

    DJI_Serial = rt_device_find(DJI_UART); //使用默认的串口配置，配置为波特率 115200,8位数据位,1位停止位,无校验位
    if (!DJI_Serial)
    {
        rt_kprintf("rt_device_find DJI_UART failed !\n"); // while(1);
        return RT_ERROR;
    }

    /* 初始化消息队列 */
    res = rt_mq_init(&DJI_mq, "DJI_mq",
                     msg_pool,
                     sizeof(struct DJI_Mxg),
                     sizeof(msg_pool),
                     RT_IPC_FLAG_FIFO);
    if (res != RT_EOK)
        return res;

    /* DMA 接收模式 */
    res = rt_device_open(DJI_Serial, RT_DEVICE_FLAG_DMA_RX);
    if (res != RT_EOK)
        return res;

    res = rt_device_set_rx_indicate(DJI_Serial, DJI_Callback);
    if (res != RT_EOK)
        return res;

    rt_thread_t DJI_thread = rt_thread_create("DJI_Read",
                                              DJI_Process_thread,
                                              RT_NULL,
                                              THREAD_STACK_DJI,
                                              THREAD_PRIO_DJI,
                                              THREAD_TICK_DJI);
    if (DJI_thread != RT_NULL)
    {
        rt_thread_startup(DJI_thread);
    }
    else
    {
        return RT_ERROR;
    }

    return RT_EOK;
}
