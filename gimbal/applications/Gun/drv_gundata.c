#include "drv_gundata.h"
#include "mod_Monitor.h"
#include "drv_thread.h"

#include "drv_Queue.h" // 使用循环队列记录发弹情况

char BoosterPOS_WR_sem_InitFlag = 0;
static rt_thread_t GunData_Thread = RT_NULL;  /* 用于进行热量计算的线程 */
static struct rt_semaphore BoosterPOS_WR_sem; /* 用于保证拨弹盘数据刷新不与热量计算冲突的信号量 */
static struct rt_semaphore GunData_5ms_sem;   /* 用于接收消息的信号量 */
static struct rt_timer GunData_Tim;           /* 闭环线程定时器 */

RefGundata_t RefGunData;                     // 从底盘通信获得的裁判系统原始数据
LocalHeat_t LocalHeat_Data;                  // 本地热量控制块
static LoopQueueCTRL_Type RecentShoot_CTRL;  // 枪管数据记录的循环队列控制块
static ShootRec_t RecentShoot[SHOOTREC_MAX]; // 最多能记录SHOOTREC_MAX条发射数据
static volatile rt_int16_t ShootCount_ALL;
static rt_int16_t Boost_Count;             // 记录当前未处理的发弹次数
static float Booster_POSMAX = 0;           // 历史记录中播弹盘曾经达到的最远的位置
static rt_uint32_t Booster_POSLastSLV = 0; // 热量计算中当前播弹叉的位置

#ifndef CORE_USING_HERO
// 步兵机器人的枪管性能数据, 访问时为 [发射机构类型][机器人等级]
const LairerAttribute_s LairerAttribute[4][3] =
    {{{50, 10, 15}, {50, 10, 15}, {50, 10, 15}},    // 未选择发射机构类型
     {{150, 15, 15}, {280, 25, 15}, {400, 35, 15}}, // 爆发优先
     {{50, 40, 15}, {100, 60, 18}, {150, 80, 18}},  // 冷却优先
     {{75, 15, 30}, {150, 25, 30}, {200, 35, 30}}}; // 弹速优先
// 步兵机器人的底盘性能数据, 访问时为 [底盘类型][机器人等级]
const rt_uint16_t Chassis_Performance[3][3] =
    {{40, 40, 40},  // 未选择底盘类型
     {60, 80, 100}, // 功率优先
     {45, 50, 55}}; // 血量优先
#else
// 英雄机器人的枪管性能数据, 访问时为 [发射机构类型][机器人等级]
const LairerAttribute_s LairerAttribute[4][3] =
    {{{100, 20, 10}, {100, 20, 10}, {100, 20, 10}},   // 未选择发射机构类型
     {{200, 40, 10}, {350, 80, 10}, {500, 120, 10}},  // 爆发优先
     {{100, 20, 16}, {200, 60, 16}, {300, 100, 16}}}; // 弹速优先
// 英雄机器人的底盘性能数据, 访问时为 [底盘类型][机器人等级]
const rt_uint16_t Chassis_Performance[3][3] =
    {{50, 50, 50},  // 未选择底盘类型
     {70, 90, 120}, // 功率优先
     {55, 60, 65}}; // 血量优先
#endif
Robot_Type_Enum ammobooster_mode = Unselected; // 发射机构类型
Robot_Type_Enum chassis_mode = Unselected;     // 未选择底盘类型
RobotLevel_Enum level = Level1;                // 机器人等级默认 1 级

/* GunData_Tim 超时函数 */
static void GunData_5ms_IRQHandler(void *parameter)
{
    while (rt_sem_trytake(&GunData_5ms_sem) == RT_EOK)
        continue; // 清空多余的信号量
    rt_sem_release(&GunData_5ms_sem);
}

int Debug_Com_Failure = 0; // 该变量用于调试时模拟通信断开
/**
 * @brief  从底盘接收裁判数据（实时热量）
 * @param  data:接收到的can消息
 */
void refresh_heat(rt_uint8_t data[])
{
    if (Debug_Com_Failure)
        return;
    rt_uint16_t RecTempu16;

    RefGunData.LastFresh_Tick = rt_tick_get();
    RefGunData.HeatLim = (rt_int16_t)(data[4] << 8 | data[5]);
    RefGunData.Heat = (rt_int16_t)(data[0] << 8 | data[1]);
    RecTempu16 = (rt_int16_t)(data[2] << 8 | data[3]);
    RefGunData.Cool = RecTempu16 & 0x7fff;
    RefGunData.CoolAcce_Flag = ((RecTempu16 & 0x8000) >> 15);
    RefGunData.SpeedLim = (rt_int16_t)(data[6] << 8 | data[7]);

    // 数据异常认为通信断开
    if ((RefGunData.HeatLim < LairerAttribute[(int)Unselected][(int)Level1].HeatLim) ||
        (RefGunData.Cool < LairerAttribute[(int)Unselected][(int)Level1].CoolingSpeed) ||
        (RefGunData.Cool == 0x7fff))
        RefGunData.ValidFlag.HeatLim_Valid = 0;
    else
        RefGunData.ValidFlag.HeatLim_Valid = 1;
    if (RefGunData.SpeedLim < LairerAttribute[(int)Unselected][(int)Level1].SpeedLim)
        RefGunData.ValidFlag.SpeedLim_Valid = 0;
    else
        RefGunData.ValidFlag.SpeedLim_Valid = 1;
}

// 初始化发射数据记录循环队列
static void ShootData_Init(void)
{
    QueueCtrl_Init(&RecentShoot_CTRL, SHOOTREC_MAX);
    for (int fori = 0; fori < SHOOTREC_MAX; fori++)
    {
        RecentShoot[fori].ShootTime = 0;
        RecentShoot[fori].TimeOut_CalFlag = 1; // 记为已超时
    }
    ShootCount_ALL = 0;
    Boost_Count = 0;
}

// 记录一次发弹
static void ShootData_Reg(rt_int32_t TickNow)
{
    rt_int16_t WR_p = Queue_GetWriteNum(&RecentShoot_CTRL); // 取写入地址
    RecentShoot[WR_p].ShootTime = TickNow;
    RecentShoot[WR_p].TimeOut_CalFlag = 0;
    ShootCount_ALL++; // 记录累积发弹量
}

// 判断裁判系统是否离线, 刷新计算本地热量中的裁判系统相关数据
static void LocalHeat_RefData_Fresh()
{
    // 裁判系统离线
    if (((rt_tick_get() - RefGunData.LastFresh_Tick > 200) || (!RefGunData.LastFresh_Tick)) || (!RefGunData.ValidFlag.HeatLim_Valid))
        LocalHeat_Data.RefSystem_Offline = 1;
    else
        LocalHeat_Data.RefSystem_Offline = 0;

    // 刷新时弹速上限数据弹速判断有效性
    if (RefGunData.ValidFlag.SpeedLim_Valid)
        LocalHeat_Data.RefSettings.SpeedLim = RefGunData.SpeedLim;
    else
    {
        // 否则采用本地上限数据
        if (ammobooster_mode != Unselected)
            LocalHeat_Data.RefSettings.SpeedLim = LairerAttribute[(int)ammobooster_mode - (int)AmmoBooster_Mode_Start][(int)level].SpeedLim;
        else
            LocalHeat_Data.RefSettings.SpeedLim = LairerAttribute[(int)Unselected][(int)level].SpeedLim;
    }

    if (!LocalHeat_Data.RefSystem_Offline)
    {
        // 只有当裁判系统未离线时才刷新本地裁判系统数据
        LocalHeat_Data.RefSettings.NowCooling = RefGunData.Cool; // 当前冷却数值
        LocalHeat_Data.RefSettings.HeatLim = RefGunData.HeatLim;
        // 以下对BaseCooling进行计算
        if (LocalHeat_Data.RefSettings.NowCooling != LocalHeat_Data.RefSettings.BaseCooling)
        {
            if (LocalHeat_Data.RefSettings.NowCooling < LocalHeat_Data.RefSettings.BaseCooling)
            { // 冷却速度下降了 说明之前对BaseCooling的计算值有误，立即修改
                LocalHeat_Data.RefSettings.CoolAcce_LevelUp_Confirm = 0;
                LocalHeat_Data.RefSettings.BaseCooling = LocalHeat_Data.RefSettings.NowCooling;
            }
            else
            {
                // 可能存在冷却加成或出现了升级
                if (RefGunData.CoolAcce_Flag)
                    // 确定是冷却加成
                    LocalHeat_Data.RefSettings.CoolAcce_LevelUp_Confirm = 0;
                else
                { //  可能是出现了冷却加成，也可能是升级了
                    LocalHeat_Data.RefSettings.CoolAcce_LevelUp_Confirm++;
                    if (LocalHeat_Data.RefSettings.CoolAcce_LevelUp_Confirm >= (GUNCOOL_LEVELUP_WAIT / GUNDATA_SOLVE_PERIOD))
                    {
                        LocalHeat_Data.RefSettings.CoolAcce_LevelUp_Confirm = 0;
                        LocalHeat_Data.RefSettings.BaseCooling = LocalHeat_Data.RefSettings.NowCooling;
                    }
                }
            }
        }
        else
        {
            LocalHeat_Data.RefSettings.CoolAcce_LevelUp_Confirm = 0;
        }
    }
    else
    {
        // 裁判系统离线, 进行相关的离线数据刷新
        LocalHeat_Data.RefSettings.CoolAcce_LevelUp_Confirm = 0;
        // 进行其他离线规则数据的刷新
        LocalHeat_Data_Fresh_Limit(RT_NULL);
    }
}

// 由外部更新拨弹盘位置数据，以1个弹丸为单位，函数中会对位置数据进行取整
// 外部调用时需要尽量保证ANG增大和发弹一一对应，每次发弹之后ANG整数部分增大1，没有发弹时ANG整数部分不变
void Gun_BoosterPOS_Fresh(float BoosterANG)
{
    int Boost_ADD;

    if (BoosterPOS_WR_sem_InitFlag)
    {
        rt_sem_take(&BoosterPOS_WR_sem, 5); // 取信号量 确保当前没有进行本地热量计算
        while (rt_sem_trytake(&BoosterPOS_WR_sem) == RT_EOK)
            continue; // 取信号量

        if (BoosterANG > Booster_POSMAX)
        { // 拨弹盘正转，刷新数据
            Booster_POSMAX = BoosterANG;

            Boost_ADD = (rt_int16_t)(Booster_POSMAX - Booster_POSLastSLV);
            if (Boost_ADD > 0)
            {
                Boost_Count += Boost_ADD;
                Booster_POSLastSLV += Boost_ADD;
            }
        }

        rt_sem_release(&BoosterPOS_WR_sem); // 重新释放信号量
    }
}

// 处理Gun_BoosterPOS_Fresh()接收到的发弹记录
static void Shoot_Check(rt_int32_t TickNow)
{
    rt_sem_take(&BoosterPOS_WR_sem, 5); // 取信号量 确保当前没有进行拨弹盘数据刷新
    if (Boost_Count > 5)
        // 保护性限幅，避免程序异常时一次记录大量发弹次数
        Boost_Count = 5;

    while (Boost_Count > 0)
    {
        Boost_Count--;
        ShootData_Reg(TickNow); //记录一条发弹数据
    }
    rt_sem_release(&BoosterPOS_WR_sem); // 重新释放信号量
}

// 读取和刷新拨弹盘数据和最近发弹数据
static void LocalHeat_ShootData_Fresh(rt_int32_t ticknow)
{
    rt_int16_t RD_p;
    ShootRec_t *ShootRec_Read;
    rt_int32_t DeltaTick;
    while (1)
    {
        if (RecentShoot_CTRL.Valid_Data > 0)
        { // 检查是否有过期数据 数据过期时需要计入累计热量
            RD_p = Queue_Get_ReadEnd(&RecentShoot_CTRL);
            ShootRec_Read = &RecentShoot[RD_p]; // 是一条标记未过期的数据
            DeltaTick = ticknow - ShootRec_Read->ShootTime;
            if (DeltaTick > REF_LAG_MAX_MS)
            {
                // 此数据已经过期 计入累积热量
                LocalHeat_Data.ConfirmHeatData.HeatData += SHOOT_HEAT; // 计入累积热量
                Queue_Delete_End(&RecentShoot_CTRL);
            }
            else
                // 数据未过期
                break;
        }
        else
            break;
    }
}

// 结算累积热量
static void LocalHeat_ConfirmHeat_Cal(rt_int32_t TickToCal)
{
    int Tick_Cool;
    float Heat_CoolNow; // 本次冷却掉的热量

    Tick_Cool = TickToCal - LocalHeat_Data.ConfirmHeatData.Solve_Tick;
    LocalHeat_Data.ConfirmHeatData.Solve_Tick = TickToCal;

    if (Tick_Cool <= 0)
        // 检查冷却计算的tick是否正常
        return; // 程序刚启动时会小于0，小于0时直接返回

    // 已经正常计算出冷却时间
    Heat_CoolNow = LocalHeat_Data.RefSettings.NowCooling * Tick_Cool / 1000.0f;
    if (Heat_CoolNow >= LocalHeat_Data.ConfirmHeatData.HeatData)
        LocalHeat_Data.ConfirmHeatData.HeatData = 0;
    else
        LocalHeat_Data.ConfirmHeatData.HeatData -= Heat_CoolNow;
}

#define REFDATA_FIXLOCAL_WAITTICK (800) // 连续多久后用裁判系统热量修正本地热量

static float LocalHeatErr = 0;               // 本地热量数据的偏差量
static rt_tick_t LocalHeat_TooMuch_Tick = 0; // 为0时表示近期本地热量没有偏大太多，其余情况为最近一次开始出现异常的时刻

// 如果裁判系统热量数据有效，则用其对本地热量进行修正
static void LocalHeat_ErrFix(void)
{
    float LocalHeatErr_Now;
    rt_tick_t TickNow = rt_tick_get(); // 取当前时刻

    if (!LocalHeat_Data.RefSystem_Offline)
    {
        // 裁判系统数据有效，可以进行误差检查
        LocalHeatErr_Now = RefGunData.Heat - LocalHeat_Data.NowHeat; // 热量差值
        LocalHeatErr = LocalHeatErr_Now * 0.2f + LocalHeatErr * 0.8f;
        if (LocalHeatErr_Now > 0)
        { // 本地热量偏少了，非常危险，需要将本地热量增加相应数值
            LocalHeat_Data.ConfirmHeatData.HeatData += LocalHeatErr_Now;
            LocalHeatErr = 0;
            LocalHeat_TooMuch_Tick = 0;
        }
        else
        {
            // 本地热量不少于裁判系统热量，检测是否长时间大于裁判系统热量较多
            if (LocalHeatErr < -30)
            {
                if (LocalHeat_TooMuch_Tick != 0)
                {
                    if (TickNow - LocalHeat_TooMuch_Tick > REFDATA_FIXLOCAL_WAITTICK)
                    {                                                                 // 如果长时间保持LocalHeatErr偏大 进行本地热量调整
                        LocalHeat_Data.ConfirmHeatData.HeatData += LocalHeatErr + 10; // 保险起见，预留一颗弹丸的热量
                        if (LocalHeat_Data.ConfirmHeatData.HeatData < 0)
                            LocalHeat_Data.ConfirmHeatData.HeatData = 0;
                        LocalHeatErr = 0;
                        LocalHeat_TooMuch_Tick = 0;
                    }
                }
                else
                    LocalHeat_TooMuch_Tick = TickNow; // 标记开始出现热量异常的时刻
            }
            else
                // 清零，表示近期没有出现过异常
                LocalHeat_TooMuch_Tick = 0;
        }
    }
}

// 在刚刚结算过的累积热量基础上，结合最近发弹记录，计算最近热量
static void LocalHeat_AddHeat_Cal(rt_int32_t TickNow)
{
    rt_int16_t RD_p;
    ShootRec_t *ShootRec_Read;
    rt_int32_t DeltaTick, SolveTick;
    int ShootData_Count;
    float CoolTemp;

    ShootData_Count = RecentShoot_CTRL.Valid_Data - 1; // 记录(需要计算的记录个数-1)

    if (ShootData_Count < 0)
    { // 没有需要处理的数据
        // 直接按照BaseCool进行冷却结算，结算至当前tick
        DeltaTick = TickNow - LocalHeat_Data.ConfirmHeatData.Solve_Tick;
        CoolTemp = DeltaTick * LocalHeat_Data.RefSettings.BaseCooling / 1000.0f;
        if (CoolTemp > LocalHeat_Data.ConfirmHeatData.HeatData)
            // 判断冷却量是否大于已有热量
            LocalHeat_Data.AddHeat = 0; // 冷却大于已有热量，热量直接为零
        else
            // 冷却不完，按照冷却进行计算
            LocalHeat_Data.AddHeat = LocalHeat_Data.ConfirmHeatData.HeatData - CoolTemp;
        LocalHeat_Data.NowHeat = LocalHeat_Data.AddHeat;
        return; // 不需要计算发射 直接返回
    }

    // 在累积热量基础上进行进一步计算，复制累积热量数据中的结算时刻和热量值
    LocalHeat_Data.AddHeat = LocalHeat_Data.ConfirmHeatData.HeatData;
    SolveTick = LocalHeat_Data.ConfirmHeatData.Solve_Tick;

    while (1)
    {                                                                 // 查找最早一次发弹
        RD_p = Queue_Get_ReadOld(&RecentShoot_CTRL, ShootData_Count); // 读最早的一次发射数据
        ShootData_Count--;
        ShootRec_Read = &RecentShoot[RD_p];

        // 数据有效，进行结算
        DeltaTick = ShootRec_Read->ShootTime - SolveTick;
        if (DeltaTick < 0)
            DeltaTick = 0;
        CoolTemp = DeltaTick * LocalHeat_Data.RefSettings.BaseCooling / 1000.0f;
        if (LocalHeat_Data.AddHeat < CoolTemp)
            LocalHeat_Data.AddHeat = 0; // 到发弹之前热量已经冷却完毕
        else
            LocalHeat_Data.AddHeat -= CoolTemp;
        LocalHeat_Data.AddHeat += SHOOT_HEAT; // 加入一次发弹对应的热量
        SolveTick = ShootRec_Read->ShootTime;

        if (ShootData_Count < 0)
        { // 检查是否已经处理完所有发射数据
            DeltaTick = TickNow - SolveTick;
            if (DeltaTick < 0)
                DeltaTick = 0;
            CoolTemp = DeltaTick * LocalHeat_Data.RefSettings.BaseCooling / 1000.0f;
            if (LocalHeat_Data.AddHeat < CoolTemp)
                LocalHeat_Data.AddHeat = 0; // 到发弹之前热量已经冷却完毕
            else
                LocalHeat_Data.AddHeat -= CoolTemp;

            LocalHeat_Data.NowHeat = LocalHeat_Data.AddHeat; // 完成热量计算
            break;                                           // 全部处理完了，结束计算
        }
    }
}

// 刷新本地热量记录数据
static void LocalHeat_Fresh(void)
{
    rt_int32_t Tick_Now;
    // 可以正常结算本地热量
    Tick_Now = (rt_int32_t)rt_tick_get(); // 读取当前tick

    Shoot_Check(Tick_Now);     // 处理发弹记录数据
    LocalHeat_RefData_Fresh(); // 读取和处理裁判系统数据
    LocalHeat_ErrFix();

    LocalHeat_ShootData_Fresh(Tick_Now);

    LocalHeat_ConfirmHeat_Cal((rt_int32_t)Tick_Now - REF_LAG_MAX_MS); // 计算累积热量
    LocalHeat_AddHeat_Cal(Tick_Now);                                  // 在累积热量基础上计算最近热量
    LocalHeat_Data.DataValid = 1;
}

// GunData计算线程
static void GunData_5ms_entry(void *parameter)
{
    SWDG_START(SWDG_GUNDATA_ID);
    while (1)
    {
        LocalHeat_Fresh();                                 // 刷新热量和射速上限
        rt_sem_take(&GunData_5ms_sem, RT_WAITING_FOREVER); // 等待下一个定时周期
        SWDG_FEED(SWDG_GUNDATA_ID);
    }
}

static rt_int16_t RemainShoot; // 剩余发弹量
// 按照当前的枪口热量，拨弹盘允许到达的最大弹丸数（与Gun_BoosterPOS_Fresh中给出的数据单位和大小一致）
rt_int16_t Get_GunShootMAX_Now(void)
{
    rt_int16_t RemainHeat;
    float RSV_Heat;

    if (LocalHeat_Data.DataValid == 0)
        return 0; // 当前本地热量未启动，不能发射弹丸

    // 由于裁判系统结算周期较长，此处按照连续计算的结果需要留有一些余量
    RSV_Heat = REF_CAL_LAG_RSV_MS / 1000.0f * LocalHeat_Data.RefSettings.NowCooling + HEAT_RSV_SET * SHOOT_HEAT;
    RemainHeat = (rt_int16_t)(LocalHeat_Data.RefSettings.HeatLim - LocalHeat_Data.NowHeat - RSV_Heat); // 预留一个弹丸余量
    if (RemainHeat < 0)
        RemainShoot = -1;
    else
        RemainShoot = RemainHeat / SHOOT_HEAT;
    return RemainShoot + (rt_int16_t)Booster_POSLastSLV;
}

// 初始化本地热量相关数据
static void LocalHeat_Data_Init()
{
    LocalHeat_Data.ConfirmHeatData.HeatData = 0;
    LocalHeat_Data.ConfirmHeatData.Solve_Tick = rt_tick_get(); // 初始结算时刻
    LocalHeat_Data.NowHeat = 0;                                // 默认热量为空
    LocalHeat_Data.AddHeat = 0;                                // 默认热量为空
    LocalHeat_Data.RefSettings.BaseCooling = LairerAttribute[(int)Unselected][(int)Level1].CoolingSpeed;
    LocalHeat_Data.RefSettings.CoolAcce_LevelUp_Confirm = 0;
    LocalHeat_Data.RefSettings.HeatLim = LairerAttribute[(int)Unselected][(int)Level1].HeatLim;
    LocalHeat_Data.RefSettings.NowCooling = LairerAttribute[(int)Unselected][(int)Level1].CoolingSpeed;
    LocalHeat_Data.DataValid = 0;
}

// 初始化计算线程
static void GunDataCal_TaskInit(void)
{
    /*定时器处理线程*/
    rt_sem_init(&GunData_5ms_sem, "GunDat_s", 0, RT_IPC_FLAG_FIFO);
    rt_sem_init(&BoosterPOS_WR_sem, "GunBst_s", 0, RT_IPC_FLAG_FIFO);
    rt_sem_release(&BoosterPOS_WR_sem); // 释放初始信号量
    BoosterPOS_WR_sem_InitFlag = 1;
    GunData_Thread = rt_thread_create("GunData", GunData_5ms_entry, RT_NULL, 2048, THREAD_PRIO_GUNDATA, 1);
    if (GunData_Thread != RT_NULL)
        rt_thread_startup(GunData_Thread);

    /*定时器中断*/
    rt_timer_init(&GunData_Tim, "GunD_Tim", GunData_5ms_IRQHandler,
                  RT_NULL, 5, RT_TIMER_FLAG_PERIODIC | RT_TIMER_FLAG_SOFT_TIMER);
    /* 启动定时器 */
    rt_timer_start(&GunData_Tim);
}

// 初始化本地热量和其它枪口数据更新
void GunData_Init(void)
{
    RefGunData.LastFresh_Tick = 0;
    RefGunData.ValidFlag.HeatLim_Valid = 0;  // 初始化裁判系统数据标志位，刚启动时默认没有裁判系统数据
    RefGunData.ValidFlag.SpeedLim_Valid = 0; // 初始化裁判系统数据标志位，刚启动时默认没有裁判系统数据
    ShootData_Init();                        // 初始化发射数据记录循环队列
    LocalHeat_Data_Init();                   // 初始化本地热量相关数据

    GunDataCal_TaskInit();
    BoosterRec_Reset();
}

// 发射机构重启时需要调用的函数
void BoosterRec_Reset(void)
{
    Booster_POSLastSLV = 0;
    Booster_POSMAX = 0;
}

// 用于操作手手动刷新等级数据
void Local_Robot_Level_Fresh(RobotLevel_Enum NewLevel)
{
    level = NewLevel;
}

// 用于操作手手动刷新机器人类型数据
void Local_Robot_Type_Fresh(Robot_Type_Enum NewType)
{
    switch (NewType)
    {
    case Unselected:
        ammobooster_mode = Unselected;
        chassis_mode = Unselected;
        break;
    case Outbreak_Priority:
#ifndef CORE_USING_HERO
    case Cooling_Priority:
#endif
    case BulletSpeed_Priority:
        ammobooster_mode = NewType;
        break;
    case Power_Priority:
    case Blood_Priority:
        chassis_mode = NewType;
        break;
    default:
        break;
    }
}

// 用于操作手手动刷新本地等级数据
void LocalHeat_Data_Fresh_Limit(rt_uint8_t *ChssisMaxPower)
{
    // 本地数据刷新需要裁判系统实际数据离线
    if (LocalHeat_Data.RefSystem_Offline)
    {
        if (ammobooster_mode != Unselected)
        {
            LocalHeat_Data.RefSettings.BaseCooling = LairerAttribute[(int)ammobooster_mode - (int)AmmoBooster_Mode_Start][(int)level].CoolingSpeed;
            LocalHeat_Data.RefSettings.HeatLim = LairerAttribute[(int)ammobooster_mode - (int)AmmoBooster_Mode_Start][(int)level].HeatLim;
        }
        else
        {
            LocalHeat_Data.RefSettings.BaseCooling = LairerAttribute[(int)Unselected][(int)level].CoolingSpeed;
            LocalHeat_Data.RefSettings.HeatLim = LairerAttribute[(int)Unselected][(int)level].HeatLim;
        }
        LocalHeat_Data.RefSettings.NowCooling = LocalHeat_Data.RefSettings.BaseCooling;
        LocalHeat_Data.RefSettings.CoolAcce_LevelUp_Confirm = 0;
    }
    // 弹速上限数据单独判断有效性
    if (!RefGunData.ValidFlag.SpeedLim_Valid)
    {
        if (ammobooster_mode != Unselected)
            LocalHeat_Data.RefSettings.SpeedLim = LairerAttribute[(int)ammobooster_mode - (int)AmmoBooster_Mode_Start][(int)level].SpeedLim;
        else
            LocalHeat_Data.RefSettings.SpeedLim = LairerAttribute[(int)Unselected][(int)level].SpeedLim;
    }
    // 直接读取底盘功率上限数据(判断指针非空)
    if (ChssisMaxPower)
    {
        if (chassis_mode != Unselected)
            *ChssisMaxPower = (rt_uint8_t)Chassis_Performance[(int)chassis_mode - (int)Chassis_Mode_Start][(int)level];
        else
            *ChssisMaxPower = (rt_uint8_t)Chassis_Performance[(int)Unselected][(int)level];
    }
}

// 获取当前的热量上限
rt_int16_t Read_Speed_Lim(void)
{
    return LocalHeat_Data.RefSettings.SpeedLim;
}

// 获取当前裁判系统弹速数据的有效性
rt_bool_t Read_RefSpeed_Valid(void)
{
    return RefGunData.ValidFlag.SpeedLim_Valid;
}
