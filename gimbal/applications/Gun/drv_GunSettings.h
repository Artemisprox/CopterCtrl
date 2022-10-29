#ifndef __DRV_GUNSETTINGS_H__
#define __DRV_GUNSETTINGS_H__

#include <rtthread.h>

// 一发弹丸积累的热量
#ifdef CORE_USING_INFANTRY
#define SHOOT_HEAT (10)
#elif defined CORE_USING_HERO
#define SHOOT_HEAT (100)
#endif

typedef enum
{
    Unselected = 0, // 未选择类型
    AmmoBooster_Mode_Start,
    Outbreak_Priority, // 爆发优先
#ifndef CORE_USING_HERO
    Cooling_Priority, // 冷却优先
#endif
    BulletSpeed_Priority, // 弹速优先

    Chassis_Mode_Start,
    Power_Priority, // 功率优先
    Blood_Priority, // 血量优先
} Robot_Type_Enum;  // 发射机构类型选择

typedef enum
{
    Level1 = 0,    // 1 级机器人
    Level2,        // 2 级机器人
    Level3,        // 3 级机器人
} RobotLevel_Enum; // 机器人等级

typedef struct
{
    rt_uint16_t HeatLim;      // 枪管热量上限
    rt_uint16_t CoolingSpeed; // 枪管冷却速度
    rt_uint16_t SpeedLim;     // 射速上限
} LairerAttribute_s;          // 发射机构属性

#endif /* __DRV_GUNSETTINGS_H__ */
