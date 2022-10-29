#ifndef __DRV_GUNDATA_H__
#define __DRV_GUNDATA_H__

#include <rtthread.h>
#include "drv_GunSettings.h" // 比赛规则相关数据

#define REF_LAG_MAX_MS (500)
#define SHOOTREC_MAX (50) // 记录的发射数据最大条数

#if defined CORE_USING_INFANTRY
#define REF_CAL_LAG_RSV_MS (100) // 预留一段时间的冷却量，用于补偿裁判系统结算频率低带来的影响
#define HEAT_RSV_SET (1)         // 预留0发弹丸对应热量

#define GUNDATA_SOLVE_PERIOD (5)    // 5ms计算一次
#define GUNCOOL_LEVELUP_WAIT (1000) // 如果冷却数据提升且没有收到场地交互加成标志位，则1000ms后认为确实升级了

#define GUN_SLOWMODE_PERIOD (143) // 低射频模式下每发射一发弹丸后等待的时间 143ms - 7发/s
#define GUN_FASTMODE_FRQ (18)     // 18发/s
#elif defined CORE_USING_HERO
#define REF_CAL_LAG_RSV_MS (300) // 预留一段时间的冷却量，用于补偿裁判系统结算频率低带来的影响
#define HEAT_RSV_SET (-0.15f)    // 预留0发弹丸对应热量

#define GUNDATA_SOLVE_PERIOD (5)    // 5ms计算一次
#define GUNCOOL_LEVELUP_WAIT (1000) // 如果冷却数据提升且没有收到场地交互加成标志位，则1000ms后认为确实升级了

#define GUN_SLOWMODE_PERIOD (600) // 低射频模式下每发射一发弹丸后等待的时间 600ms - 1发/s
#endif

// 用于记录发弹数据的结构体
typedef struct
{
    rt_tick_t ShootTime;
    char TimeOut_CalFlag; // 若数据超时且已计入累计热量，则置1
} ShootRec_t;

// 用于记录累积热量数据的结构体
typedef struct
{
    rt_int16_t CoolAcce_LevelUp_Confirm; // 用于判断是否为升级带来的冷却增益
    rt_int16_t BaseCooling;              // 记录当前等级的基础冷却
    rt_int16_t NowCooling;               // 记录当前裁判系统的实际冷却
    rt_int16_t HeatLim;                  // 记录当前裁判系统的热量上限
    rt_int16_t SpeedLim;                 // 记录当前的射速上限
} RefHeatSettings_t;

// 用于记录累积热量数据的结构体
typedef struct
{
    float HeatData;        // 记录当前剩余热量
    rt_int32_t Solve_Tick; // 记录目前累计热量结算到的时刻
} ConfirmHeat_t;

typedef struct
{
    rt_uint8_t RefSystem_Offline;  // 裁判系统通信异常
    char DataValid;                // 数据是否有效
    RefHeatSettings_t RefSettings; // 计算热量时使用的计算数据
    ConfirmHeat_t ConfirmHeatData; // 累积热量
    float AddHeat;                 // 临近热量
    float NowHeat;                 // 本地热量计算结果
} LocalHeat_t;

typedef struct
{
    rt_int16_t Heat;
    rt_int16_t HeatLim;
    rt_int16_t Cool;
    rt_uint8_t CoolAcce_Flag;
    rt_int16_t SpeedLim;
    struct DataValid_s
    {
        rt_uint8_t HeatLim_Valid;
        rt_uint8_t SpeedLim_Valid;
    } ValidFlag;
    rt_tick_t LastFresh_Tick;
} RefGundata_t;

extern RefGundata_t RefGunData;
extern Robot_Type_Enum ammobooster_mode; // 发射机构类型
extern Robot_Type_Enum chassis_mode;     // 底盘类型
extern RobotLevel_Enum level;            // 机器人等级

// 机器人的枪管性能数据, 访问时为 [发射机构类型][机器人等级]
extern const LairerAttribute_s LairerAttribute[4][3];
// 机器人的底盘性能数据, 访问时为 [底盘类型][机器人等级]
extern const rt_uint16_t Chassis_Performance[3][3];

/**
 * @brief  从底盘接收裁判数据（实时热量）
 * @param  data:接收到的can消息
 */
extern void refresh_heat(rt_uint8_t data[]);

// 由外部更新拨弹盘位置数据，以1个弹丸为单位，函数中会对位置数据进行取整
// 外部调用时需要尽量保证ANG增大和发弹一一对应，每次发弹之后ANG整数部分增大1，没有发弹时ANG整数部分不变
extern void Gun_BoosterPOS_Fresh(float BoosterANG);

// 按照当前的枪口热量，拨弹盘允许到达的最大弹丸数（与Gun_BoosterPOS_Fresh中给出的数据单位和大小一致）
extern rt_int16_t Get_GunShootMAX_Now(void);

// 获取当前的热量上限
extern rt_int16_t Read_Speed_Lim(void);

// 用于操作手手动刷新等级数据
extern void Local_Robot_Level_Fresh(RobotLevel_Enum NewLevel);

// 用于操作手手动刷新机器人类型数据
extern void Local_Robot_Type_Fresh(Robot_Type_Enum NewType);

// 用于操作手手动刷新本地等级数据
extern void LocalHeat_Data_Fresh_Limit(rt_uint8_t *ChssisMaxPower);

// 发射机构重启时需要调用的函数
extern void BoosterRec_Reset(void);

// 初始化本地热量和其它枪口数据更新
extern void GunData_Init(void);

// 获取当前裁判系统弹速数据的有效性
extern rt_bool_t Read_RefSpeed_Valid(void);
#endif
