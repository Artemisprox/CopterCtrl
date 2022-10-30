#include "drv_Aimbot_Public.h"
#include "drv_VisualTiming.h"
#include "func_Aimbot_Com.h"
#include "drv_canthread.h"
#include "drv_thread.h"
#include "robodata.h"
#include "drv_IMU.h"
#include "drv_GimbalPublic.h"
#ifndef AIMBOT_CIMMUNICATION_USING_CAN
#include "drv_Aimbot_UARTCom.h"
#endif /* AIMBOT_CIMMUNICATION_USING_CAN */
#include "mod_Monitor.h"
#include "drv_utils.h"

static rt_thread_t Visual_Send_ThreadTid = RT_NULL; // 自瞄视觉通信线程句柄

static AttitudeData_Type Gimbal_Atti_Send;

#undef AIMBOT_COM_COLLECT_PACKET_LOSS_RATE
#ifdef AIMBOT_COM_COLLECT_PACKET_LOSS_RATE
rt_uint8_t flag_packet_loss_rate_start = 0;  // 丢包统计开始标志

rt_uint16_t send_Package_Num = 0;            // 己方发送包的个数
rt_uint32_t Package_SumCheck_Passed_Num = 0; // 和校验通过的个数
#endif                                       /* AIMBOT_COM_COLLECT_PACKET_LOSS_RATE */

#define AIMBOT_WATCH_CLICK_DATA
#ifdef AIMBOT_WATCH_CLICK_DATA
uint8_t click = 0;
#endif /* AIMBOT_WATCH_CLICK_DATA */

// 自瞄通信设备, 可能为串口或者 CAN , 可在 menuconfig 内修改
rt_device_t aimbot_device;

#if (LED_CTRL_EN)
//收到有效数据后LED闪灯
static char rec = 0;
static int count = 100;
static void can2_LEDCTRL()
{
    count++;
    if (count > 25)
    {
        count = 0;
        if (rec == 0)
        {
            rec = 1;
            rt_pin_write(GET_PIN(E, 2), PIN_HIGH);
        }
        else
        {
            rec = 0;
            rt_pin_write(GET_PIN(E, 2), PIN_LOW);
        }
    }
}
#endif

struct rt_can_msg VisualTiming_Sendmsg;
struct rt_can_msg GimbalAtti_Sendmsg;

static rt_tick_t Timing_Tick_Get;      // 用于获取当前tick
static rt_tick_t Timing_Tick_NextSend; // 计算下一次发送时的Tick

static rt_uint32_t Count_100ms; // 记录当前为第多少个100ms
static rt_uint32_t Count_5ms;   // 记录当前为第多少个5ms

// 外部调用，用于修改当前发给视觉的鼠标右键标志位 按下为1
void Aimbot_FreshMouseClick(char ClickData)
{
    utils_write_bit(&VisualSend_Flags, AimFlag_MouseRightData, ClickData);
}

// 发送对时信息
static void VisualCom_TimingSend(void)
{
    rt_int16_t MuzzleV_Temp; // 用于适应发送协议的中转变量
    int fori;
    char sum;

    MuzzleV_Temp = (rt_int16_t)(Muzzle_V_REM * 100); // 先存入变量，避免读写访问冲突导致出错 单位0.01
    if (MuzzleV_Temp > 4095 || MuzzleV_Temp <= 0)
        // 通信不能溢出
        MuzzleV_Temp = 4095;

    VisualTiming_Sendmsg.data[0] = (Count_100ms >> 0) & 0xFF;
    VisualTiming_Sendmsg.data[1] = (Count_100ms >> 8) & 0xFF;
    VisualTiming_Sendmsg.data[2] = (Count_100ms >> 16) & 0xFF;
    VisualTiming_Sendmsg.data[3] = (Visual_Mode_Set & 0x0F) | ((MuzzleV_Temp >> 4) & 0xF0);
    VisualTiming_Sendmsg.data[4] = MuzzleV_Temp & 0xFF;

    utils_write_bit(&VisualSend_Flags, AimFlag_MyColor, Color_Myself); // 更新当前自瞄颜色
    VisualTiming_Sendmsg.data[5] = (VisualSend_Flags & 0xC3);

    sum = 0;
    for (fori = 0; fori < 6; fori++)
        sum += VisualTiming_Sendmsg.data[fori];
    VisualTiming_Sendmsg.data[6] = sum;

#ifdef AIMBOT_COM_COLLECT_PACKET_LOSS_RATE
    ++send_Package_Num;
#endif /* AIMBOT_COM_COLLECT_PACKET_LOSS_RATE */

    /* 发送 */
#ifndef AIMBOT_CIMMUNICATION_USING_CAN
    Aimbot_Write_UART_Data(&aimbot_device, 0, VisualTiming_Sendmsg.data, 8);
#else
    rt_device_write(aimbot_device, 0, &VisualTiming_Sendmsg, sizeof(GimbalAtti_Sendmsg));
#endif
}

static unsigned char AttiSend_5ms_Count;

static int SendCount = 0;

// 发送云台姿态信息
static void VisualCom_AttiSend(void)
{
    int fori;
    char sum;
    rt_int16_t IntToChar_Temp;

    // 获取当前陀螺仪姿态
    Gimbal_Atti_Send.Pitch = gimbal_atti.pitch;
    Gimbal_Atti_Send.Yaw = gimbal_atti.yaw;
    Gimbal_Atti_Send.Roll = gimbal_atti.roll;

    // 矫正摄像头安装角度误差
    Gimbal_Atti_Send.Pitch += CAMERA_PITCH_FIX;
    Gimbal_Atti_Send.Yaw += CAMERA_YAW_FIX;
    utils_norm_circle_number(&Gimbal_Atti_Send.Yaw, -180.f, 360.f);

    IntToChar_Temp = (rt_int16_t)(Gimbal_Atti_Send.Pitch * 180);
    GimbalAtti_Sendmsg.data[0] = (rt_uint8_t)(IntToChar_Temp >> 8);
    GimbalAtti_Sendmsg.data[1] = (rt_uint8_t)IntToChar_Temp;

    IntToChar_Temp = (rt_int16_t)(Gimbal_Atti_Send.Yaw * 180);
    GimbalAtti_Sendmsg.data[2] = (rt_uint8_t)(IntToChar_Temp >> 8);
    GimbalAtti_Sendmsg.data[3] = (rt_uint8_t)IntToChar_Temp;

    IntToChar_Temp = (rt_int16_t)(Gimbal_Atti_Send.Roll * 180);
    GimbalAtti_Sendmsg.data[4] = (rt_uint8_t)(IntToChar_Temp >> 8);
    GimbalAtti_Sendmsg.data[5] = (rt_uint8_t)IntToChar_Temp;

    AttiSend_5ms_Count = Count_5ms % 200;

    if (AttiSend_5ms_Count == 0)
        SendCount = 0;
    else
        SendCount++;

    GimbalAtti_Sendmsg.data[6] = AttiSend_5ms_Count;

    sum = 0;
    for (fori = 0; fori < 7; fori++)
        sum += GimbalAtti_Sendmsg.data[fori];
    GimbalAtti_Sendmsg.data[7] = sum;

#ifdef AIMBOT_COM_COLLECT_PACKET_LOSS_RATE
    ++send_Package_Num;
#endif /* AIMBOT_COM_COLLECT_PACKET_LOSS_RATE */

    /* 发送 */
#ifndef AIMBOT_CIMMUNICATION_USING_CAN
    Aimbot_Write_UART_Data(&aimbot_device, 1, GimbalAtti_Sendmsg.data, 8);
#else
    rt_device_write(aimbot_device, 0, &GimbalAtti_Sendmsg, sizeof(GimbalAtti_Sendmsg));
#endif
}

// 自瞄与视觉通信线程
void Visual_Send_Thread(void *Para)
{
    VisualTiming_Sendmsg.id = ID_VISUAL_TIMING_SEND; //设置ID
    VisualTiming_Sendmsg.ide = RT_CAN_STDID;         //标准帧
    VisualTiming_Sendmsg.rtr = RT_CAN_DTR;           //数据帧
    VisualTiming_Sendmsg.priv = 0;                   //报文优先级最高
    VisualTiming_Sendmsg.len = 7;                    //长度7

    GimbalAtti_Sendmsg.id = ID_VISUAL_ATTI_SEND; //设置ID
    GimbalAtti_Sendmsg.ide = RT_CAN_STDID;       //标准帧
    GimbalAtti_Sendmsg.rtr = RT_CAN_DTR;         //数据帧
    GimbalAtti_Sendmsg.priv = 0;                 //报文优先级最高
    GimbalAtti_Sendmsg.len = 8;                  //长度8

    /* 延时，使第一次发送的时刻为100ms的整数倍 */
    // 读当前Tick
    Timing_Tick_Get = rt_tick_get();
    // 取当前Tick的百位及以上 再加2 得到需要延时到的时刻
    Timing_Tick_NextSend = (Timing_Tick_Get / 100 + 2) * 100;
    // 等到Timing_Tick_NextSend
    rt_thread_delay_to_tick(Timing_Tick_NextSend, &Timing_Tick_Get);
    SWDG_START(SWDG_AIMBOT_SEND_ID);

    while (1)
    {
        Count_5ms = Timing_Tick_Get / 5;          // 取当前Tick的整5ms的个数
        Timing_Tick_NextSend = (Count_5ms)*5 + 5; // 计算出下一次发送的时间
        if (Timing_Tick_Get % 100 == 0)
        {
            // 当前为整100ms，需要先发送对时
            Count_100ms = Timing_Tick_Get / 100;
            VisualCom_TimingSend(); // 发送对时信息
#ifdef AIMBOT_WATCH_CLICK_DATA
            click = VisualSend_Flags & 0x01;
#endif /* AIMBOT_WATCH_CLICK_DATA */
        }
        VisualCom_AttiSend(); // 发送云台姿态信息

        rt_thread_delay_to_tick(Timing_Tick_NextSend, &Timing_Tick_Get); // 准备下一次发送
        SWDG_FEED(SWDG_AIMBOT_SEND_ID);
    }
}

// 视觉通信初始化
int Visual_Com_Init(void)
{
    // 清空当前记录的视觉工作模式
    VisualMode_FB = VISUAL_MODE_ERR;
    VisualSend_Flags = 0;        // 清空所有发送标志位
    Color_Myself = My_Color_Red; // 默认识别红色

    Gimbal_Set_Cal_READ_Valid = 0; // 默认可读设定值：组0

    //所有自瞄姿态设定值数据标记无效
    GimbalSet_Receive[0].State = RT_ERROR;
    GimbalSet_Receive[1].State = RT_ERROR;

    Visual_Mode_Set = VISUAL_MODE_AIMBOT_V2; //默认开启二代自瞄

    // 如果使用 CAN 通信, 就将自瞄数据发送的设备写为 CAN2, 否则就初始化串口 6
#ifndef AIMBOT_CIMMUNICATION_USING_CAN
    if (RT_EOK != Aimbot_UART_Init(&aimbot_device, VisualCom_Receive_Flag, VisualCom_Receive_Atti))
        return RT_ERROR;
#else  /* AIMBOT_CIMMUNICATION_USING_CAN */
    aimbot_device = can2_dev;
#endif /* AIMBOT_CIMMUNICATION_USING_CAN */

    // 初始化自瞄通信线程
    /* 创建线程，名称是 Aimbot_2，入口是 Visual_Send_Thread */
    Visual_Send_ThreadTid = rt_thread_create("AimbotSend", Visual_Send_Thread, RT_NULL,
                                             1024, THREAD_PRIO_AIMBOT_V2_SEND, 1);

    /* 如果获得线程控制块，启动这个线程 */
    if (Visual_Send_ThreadTid != RT_NULL)
        rt_thread_startup(Visual_Send_ThreadTid);

    return RT_EOK;
}

// 接收：标志位报文
rt_err_t VisualCom_Receive_Flag(rt_uint8_t rxmsg[])
{
    int fori;
    char sum;
    char FlagsTemp;
    // 校验和检查
    sum = 0;
    for (fori = 0; fori < 6; fori++)
        sum += rxmsg[fori];
    if (sum != rxmsg[fori])
        // 和校验未通过
        return RT_ERROR;

    Visual_LastFresh_Tick = rt_tick_get();

    // 数据保存
    VisualMode_FB = rxmsg[1];
    GimbalTolerance_Pitch = ((rt_int16_t)(rxmsg[2]) << 8 | rxmsg[3]) / 180.0f;
    GimbalTolerance_Yaw = ((rt_int16_t)(rxmsg[4]) << 8 | rxmsg[5]) / 180.0f;

    FlagsTemp = rxmsg[0];
    VisualFlag_TargetFound = utils_read_bit(FlagsTemp, 0);
    VisualFlag_Fire = utils_read_bit(FlagsTemp, 1);
    VisualFlag_BurstShoot = utils_read_bit(FlagsTemp, 2);
    VisualFlag_ExitRune = utils_read_bit(FlagsTemp, 3);
    VisualFlag_RuneFire = utils_read_bit(FlagsTemp, 4);
    VisualFlag_RuneBurstShoot = utils_read_bit(FlagsTemp, 5);
    VisualFlag_WorkingCorrect = utils_read_bit(FlagsTemp, 6);

    if (VisualFlag_TargetFound == 0)
    {
        // 视觉丢失目标了，此时需要直接将已有设定值数据设定为 失效
        GimbalSet_Receive[0].State = RT_ERROR;
        GimbalSet_Receive[1].State = RT_ERROR;
    }

#ifdef AIMBOT_COM_COLLECT_PACKET_LOSS_RATE
    ++Package_SumCheck_Passed_Num;
#endif /* AIMBOT_COM_COLLECT_PACKET_LOSS_RATE */
    return RT_EOK;
}

static float VisualAttiSetOri_Pitch, VisualAttiSetOri_Yaw;
static float VisualAttiSetOri_PitchSpe, VisualAttiSetOri_YawSpe;
static rt_uint8_t VisualAttiSet_TickOri; // 收到的设定值报文给出的对应Tick的原始值

static rt_tick_t VisualAttiSetReceive_TickOut; // 收到的设定值报文给出的对应Tick的完整识别值

// 推算补全视觉发来的Tick数据
static rt_tick_t VisualCom_Tick_Fill(rt_uint8_t TickOri)
{
    rt_tick_t Tick_Temp;
    rt_int32_t DeltaTick;
    rt_tick_t TickNow = rt_tick_get(); // 读取并暂存当前的Tick

    Tick_Temp = (TickNow / 1000) * 1000 + TickOri * 10; // 当前整数秒+视觉发送的零头
    // 检查是否差了整秒
    DeltaTick = Tick_Temp - TickNow;
    if (DeltaTick > 500)
        VisualAttiSetReceive_TickOut = Tick_Temp - 1000;
    else if (DeltaTick < -500)
        VisualAttiSetReceive_TickOut = Tick_Temp + 1000;
    else
        VisualAttiSetReceive_TickOut = Tick_Temp;
    return VisualAttiSetReceive_TickOut;
}

// 接收：云台设定值报文
rt_err_t VisualCom_Receive_Atti(rt_uint8_t rxmsg[])
{
    int fori;
    char sum;
    // 校验和检查
    sum = 0;
    for (fori = 0; fori < 7; fori++)
        sum += rxmsg[fori];
    if (sum != rxmsg[fori])
        // 和校验未通过
        return RT_ERROR;

    Visual_LastFresh_Tick = rt_tick_get();

    // 数据保存
    VisualAttiSet_TickOri = rxmsg[0];
    VisualAttiSetOri_Pitch = ((rt_int16_t)(rxmsg[1] << 8 | rxmsg[2])) / 180.0f;
    VisualAttiSetOri_Yaw = ((rt_int16_t)(rxmsg[3] << 8 | rxmsg[4])) / 180.0f;
    VisualAttiSetOri_PitchSpe = (rt_int8_t)rxmsg[5];
    VisualAttiSetOri_YawSpe = (rt_int8_t)rxmsg[6];

    /*数据处理*/
    // 先计算Tick的数值
    GimbalSet_Receive[!Gimbal_Set_Cal_READ_Valid].PredictedTime = VisualCom_Tick_Fill(VisualAttiSet_TickOri);
    GimbalSet_Receive[!Gimbal_Set_Cal_READ_Valid].GimbalSet_Angle.Pitch = VisualAttiSetOri_Pitch + PitchFix;
    GimbalSet_Receive[!Gimbal_Set_Cal_READ_Valid].GimbalSet_Angle.Yaw = VisualAttiSetOri_Yaw + YawFix;
    GimbalSet_Receive[!Gimbal_Set_Cal_READ_Valid].GimbalSet_Speed.Pitch = VisualAttiSetOri_PitchSpe;
    GimbalSet_Receive[!Gimbal_Set_Cal_READ_Valid].GimbalSet_Speed.Yaw = VisualAttiSetOri_YawSpe;

    if (Visual_Mode_Set == VisualMode_FB)
    {
        // 当前视觉工作模式正确
        GimbalSet_Receive[!Gimbal_Set_Cal_READ_Valid].State = RT_EOK;
        Gimbal_Set_Cal_READ_Valid = !Gimbal_Set_Cal_READ_Valid; // 切换可读取
    }
    else
    {
        // 视觉工作模式错误 则所有数据均标记为失效
        GimbalSet_Receive[!Gimbal_Set_Cal_READ_Valid].State = RT_ERROR;
        GimbalSet_Receive[Gimbal_Set_Cal_READ_Valid].State = RT_ERROR;
    }
#ifdef AIMBOT_COM_COLLECT_PACKET_LOSS_RATE
    ++Package_SumCheck_Passed_Num;
#endif /* AIMBOT_COM_COLLECT_PACKET_LOSS_RATE */
    return RT_EOK;
}
