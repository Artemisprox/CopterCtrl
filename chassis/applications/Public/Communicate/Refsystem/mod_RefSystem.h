#ifndef __MOD_REFSYSTEM_H__
#define __MOD_REFSYSTEM_H__
#include <rtthread.h>
#include <rtdevice.h>
#include <board.h>

#define DJI_UART "uart6"

#define HEADER_SOF 0xA5

#define LEN_HEADER 5 //帧头长度
#define LEN_CMDID 2  //命名码长度
#define LEN_TAIL 2   //帧尾CRC16

#define LEN_game_state 3               // 0x0001
#define LEN_game_result 1              // 0x0002
#define LEN_game_robot_survivors 32    // 0x0003
#define LEN_dart_status 3              // 0x0004
#define LEN_ICRA_buff_status 3         // 0x0005
#define LEN_event_data 4               // 0x0101
#define LEN_supply_projectile_action 3 // 0x0102
/* #define LEN_supply_projectile_booking 2 //0x0103   */
#define LEN_referee_warning 2     // 0x0104
#define LEN_dart_remaining_time 1 // 0x0105
#define LEN_game_robot_state 18   // 0x0201
#define LEN_power_heat_data 16    // 0x0202
#define LEN_game_robot_pos 16     // 0x0203
#define LEN_buff_musk 1           // 0x0204
#define LEN_aerial_robot_energy 3 // 0x0205
#define LEN_robot_hurt 1          // 0x0206
#define LEN_shoot_data 6          // 0x0207
#define LEN_bullet_remaining 2    // 0x0208
#define LEN_rfid_status 4         // 0x0209
#define LEN_dart_client_cmd 12    // 0x020A
#define LEN_interact 10           // 0x0301

/* 数据帧字节偏移 */
typedef enum
{
    FRAME_HEADER = 0,
    CMD_ID = 5,
    DATA = 7,

} FrameOffset_e;

/* 数据帧帧头字节偏移 */
typedef enum
{
    SOF = 0,         //数据帧起始字节
    DATA_LENGTH = 1, //数据帧中data的长度
    SEQ = 3,         //包序号
    CRC8 = 4         // CRC8

} FrameHeaderOffset_e;

/* cmd_id 命令码 ID */
typedef enum
{
    ID_game_state = 0x0001,               //比赛状态数据
    ID_game_result = 0x0002,              //比赛结果数据
    ID_game_robot_survivors = 0x0003,     //比赛机器人血量数据
    ID_dart_status = 0x0004,              //飞镖发射状态
    ID_ICRA_buff_status = 0x0005,         // ICRA
    ID_event_data = 0x0101,               //场地事件数据
    ID_supply_projectile_action = 0x0102, //场地补给站动作标志数据
//	ID_supply_projectile_booking 	= 0x0103,
    ID_referee_warning = 0x0104,          //裁判警告数据
    ID_dart_remaining_time = 0x0105,      //飞镖发射口倒计时
    ID_game_robot_state = 0x0201,         //机器人状态数据
    ID_power_heat_data = 0x0202,          //实时功率热量数据
    ID_game_robot_pos = 0x0203,           //机器人位置数据
    ID_buff_musk = 0x0204,                //机器人增益数据
    ID_aerial_robot_energy = 0x0205,      //空中机器人能量状态数据
    ID_robot_hurt = 0x0206,               //伤害状态数据
    ID_shoot_data = 0x0207,               //实时射击数据
    ID_bullet_remaining = 0x0208,         //弹丸剩余发射数
    ID_rfid_status = 0x0209,              //机器人RFID状态
    ID_dart_client_cmd = 0x020A,          //飞镖机器人客户端数据
    ID_student_interactive_data = 0x0301, //机器人间交互数据
    ID_robot_interactive_data = 0x0302,   //自定义控制器交互数据接口
    ID_robot_command_map = 0x0303,        //客户端小地图交互数据，触发发送
    ID_robot_command_remote = 0x0304,     //键盘、鼠标信息，通过图传串口发送

} CmdID_e;

typedef struct
{
    rt_tick_t game_robot_state; // 机器人状态数据
    rt_tick_t power_heat_data;  // 实时功率热量数据
    rt_tick_t buff_musk;        // 机器人增益数据
    rt_tick_t shoot_data;       // 实时射击数据
} RefReceiveTime_s;             // 各类数据接收到的时间

/* 帧头 */
typedef __packed struct
{
    uint8_t SOF;         //数据帧起始字节
    uint16_t DataLength; //数据帧中data的长度
    uint8_t Seq;         //包序号
    uint8_t CRC8;        // CRC8校验

} FrameHeader_t;

/* ID: 0x0001  Byte:  3    比赛状态数据 */
typedef __packed struct
{
    uint8_t game_type : 4;      //比赛类型
    uint8_t game_progress : 4;  //当前比赛阶段
    uint16_t stage_remain_time; //当前阶段剩余时间
    uint64_t SyncTimeStamp;     //机器人接收到该指令的精确Unix时间，当机载端收到有效的NTP服务器授时后生效

} ext_game_state_t;

/* ID: 0x0002  Byte:  1    比赛结果数据 */
typedef __packed struct
{
    uint8_t winner;

} ext_game_result_t;

/* ID: 0x0003  28bits    机器人血量数据 */
typedef __packed struct
{
    uint16_t red_1_robot_HP;
    uint16_t red_2_robot_HP;
    uint16_t red_3_robot_HP;
    uint16_t red_4_robot_HP;
    uint16_t red_5_robot_HP;
    uint16_t red_7_robot_HP;
    uint16_t red_outpost_HP;
    uint16_t red_base_HP;
    uint16_t blue_1_robot_HP;
    uint16_t blue_2_robot_HP;
    uint16_t blue_3_robot_HP;
    uint16_t blue_4_robot_HP;
    uint16_t blue_5_robot_HP;
    uint16_t blue_7_robot_HP;
    uint16_t blue_outpost_HP;
    uint16_t blue_base_HP;

} ext_game_robot_survivors_t;

/* ID: 0x0004  3bits  飞镖发射状态 */
typedef __packed struct
{
    uint8_t dart_belong;
    uint16_t stage_remaining_time;

} ext_dart_status_t;

/* ID: 0x0005 3bits ICRA */
typedef __packed struct
{
    uint8_t F1_zone_status : 1;
    uint8_t F1_zone_buff_debuff_status : 3;
    uint8_t F2_zone_status : 1;
    uint8_t F2_zone_buff_debuff_status : 3;
    uint8_t F3_zone_status : 1;
    uint8_t F3_zone_buff_debuff_status : 3;
    uint8_t F4_zone_status : 1;
    uint8_t F4_zone_buff_debuff_status : 3;
    uint8_t F5_zone_status : 1;
    uint8_t F5_zone_buff_debuff_status : 3;
    uint8_t F6_zone_status : 1;
    uint8_t F6_zone_buff_debuff_status : 3;
    uint16_t red1_bullet_left;
    uint16_t red2_bullet_left;
    uint16_t blue1_bullet_left;
    uint16_t blue2_bullet_left;

} ext_ICRA_buff_debuff_zone_status_t;

/* ID: 0x0101  Byte:  4    场地事件数据 */
typedef __packed struct
{
    uint32_t event_type;

} ext_event_data_t;

/* ID: 0x0102  Byte:  4    补给站动作标识 */
typedef __packed struct
{
    uint8_t supply_projectile_id;
    uint8_t supply_robot_id;
    uint8_t supply_projectile_step;
    uint8_t supply_projectile_num;

} ext_supply_projectile_action_t;

/* ID: 0X0104  Byte:  2    裁判警告数据 */
typedef __packed struct
{
    uint8_t level;
    uint8_t foul_robot_id;

} ext_referee_warning_t;

/* ID: 0X0105  Byte:  1    飞镖发射口倒计时 */
typedef __packed struct
{
    uint8_t dart_remaining_time;

} ext_dart_remaining_time_t;

/* ID: 0X0201  Byte: 15    比赛机器人状态 */
typedef __packed struct
{
    uint8_t robot_id;    //机器人id 可用于进行数据校验
    uint8_t robot_level; //机器人等级
    uint16_t remain_HP;  //剩余血量
    uint16_t max_HP;     //血量上限
    uint16_t shooter_id1_17mm_cooling_rate;
    uint16_t shooter_id1_17mm_cooling_limit;
    uint16_t shooter_id1_17mm_speed_limit;
    uint16_t shooter_id2_17mm_cooling_rate;
    uint16_t shooter_id2_17mm_cooling_limit;
    uint16_t shooter_id2_17mm_speed_limit;
    uint16_t shooter_id1_42mm_cooling_rate;
    uint16_t shooter_id1_42mm_cooling_limit;
    uint16_t shooter_id1_42mm_speed_limit;
    uint16_t chassis_power_limit;
    uint8_t mains_power_gimbal_output : 1;
    uint8_t mains_power_chassis_output : 1;
    uint8_t mains_power_shooter_output : 1;

} ext_game_robot_status_t;

/* ID: 0X0202  Byte: 14    实时功率热量数据 */
typedef __packed struct
{
    uint16_t chassis_volt;
    uint16_t chassis_current;
    float chassis_power;
    uint16_t chassis_power_buffer;
    uint16_t shooter_id1_17mm_cooling_heat;
    uint16_t shooter_id2_17mm_cooling_heat;
    uint16_t shooter_id1_42mm_cooling_heat;

} ext_power_heat_data_t;

/* ID: 0x0203  Byte: 16    机器人位置数据 */
typedef __packed struct
{
    float x;
    float y;
    float z;
    float yaw;

} ext_game_robot_pos_t;

/* ID: 0x0204  Byte:  1    机器人增益数据 */
typedef __packed struct
{
    uint8_t power_rune_buff;

} ext_buff_musk_t;

/* ID: 0x0205  Byte:  3    空中机器人能量状态 */
typedef __packed struct
{
    uint8_t attack_time;

} aerial_robot_energy_t;

/* ID: 0x0206  Byte:  1    伤害状态 */
typedef __packed struct
{
    uint8_t armor_id : 4;
    uint8_t hurt_type : 4;

} ext_robot_hurt_t;

/* ID: 0x0207  Byte:  6    实时射击信息 */
typedef __packed struct
{
    uint8_t bullet_type;
    uint8_t shooter_id;
    uint8_t bullet_freq;
    float bullet_speed;

} ext_shoot_data_t;

/* ID: 0x0208  Byte:  2    子弹剩余发射数 */
typedef __packed struct
{
    uint16_t bullet_remaining_num_17mm;
    uint16_t bullet_remaining_num_42mm;
    uint16_t coin_remaining_num;

} ext_bullet_remaining_t;

/* ID：0X0209  Byte:  4    机器人RFID状态 */
typedef __packed struct
{
    uint32_t rfid_status;

} ext_rfid_status_t;

/* ID: 0x020A  Byte:  12   飞镖机器人客户端指令数据 */
typedef __packed struct
{
    uint8_t dart_launch_opening_status;
    uint8_t dart_attack_target;
    uint16_t target_change_time;
    uint8_t first_dart_speed;
    uint8_t second_dart_speed;
    uint8_t third_dart_speed;
    uint8_t fourth_dart_speed;
    uint16_t last_dart_launch_time;
    uint16_t operate_launch_cmd_time;

} ext_dart_client_cmd_t;

/* ID: 0x0301  Byte:  n    机器人间交互数据 */
typedef __packed struct
{
    uint16_t data_cmd_id;
    uint16_t send_ID;
    uint16_t receiver_ID;

} ext_student_interactive_header_data_t;

/* ID: 0x0302  Byte:  n    自定义控制器交互数据 */
typedef __packed struct
{
    uint8_t data[LEN_interact];

} robot_interactive_data_t;

typedef __packed struct
{
    float data1;
    float data2;
    float data3;
    uint8_t masks;

} user_custom_data_t;

typedef __packed struct
{
    FrameHeader_t txFrameHeader; //帧头
    uint16_t CmdID;
    ext_student_interactive_header_data_t dataFrameHeader;
    robot_interactive_data_t userData;
    uint16_t FrameTail;
    uint16_t dataLength;

} ext_send_user_data_t;

/* ID: 0x0303  Byte:  15    小地图交互信息 */
typedef __packed struct
{
    float target_position_x;
    float target_position_y;
    float target_position_z;
    uint8_t commd_keyboard;
    uint16_t target_robot_ID;

} ext_robot_command_map_t;

/* ID: 0x0304  Byte:  14    客户端下发信息 */
typedef __packed struct
{
    int16_t mouse_x;
    int16_t mouse_y;
    int16_t mouse_z;
    int8_t left_button_down;
    int8_t right_button_down;
    uint16_t keyboard_value;
    uint16_t reserved;

} ext_robot_command_remote_t;

typedef __packed struct
{
    FrameHeader_t FrameHeader;
    CmdID_e CmdID;
    __packed union
    {
        ext_game_state_t ext_game_state;                             // 0x0001
        ext_game_result_t ext_game_result;                           // 0x0002
        ext_game_robot_survivors_t ext_game_robot_survivors;         // 0x0003
        ext_dart_status_t ext_dart_status;                           // 0x0004
        ext_ICRA_buff_debuff_zone_status_t ext_ICRA_buff_status;     // 0x0005
        ext_event_data_t ext_event_data;                             // 0x0101
        ext_supply_projectile_action_t ext_supply_projectile_action; // 0x0102
        // ext_supply_projectile_booking_t		  ext_supply_projectile_booking;
        ext_referee_warning_t ext_referee_warning;           // 0x0104
        ext_dart_remaining_time_t ext_dart_remaining_time;   // 0x0105
        ext_game_robot_status_t ext_game_robot_state;        // 0x0201
        ext_power_heat_data_t ext_power_heat_data;           // 0x0202
        ext_game_robot_pos_t ext_game_robot_pos;             // 0x0203
        ext_buff_musk_t ext_buff_musk;                       // 0x0204
        aerial_robot_energy_t aerial_robot_energy;           // 0x0205
        ext_robot_hurt_t ext_robot_hurt;                     // 0x0206
        ext_shoot_data_t ext_shoot_data;                     // 0x0207
        ext_bullet_remaining_t ext_bullet_remaining;         // 0x0208
        ext_rfid_status_t ext_rfid_status;                   // 0x0209
        ext_dart_client_cmd_t ext_dart_client_cmd;           // 0x020A
        ext_send_user_data_t ext_send_user_data;             // 0x0301
        robot_interactive_data_t robot_interactive_data;     // 0x0302
        ext_robot_command_map_t ext_robot_command_map;       // 0x0303
        ext_robot_command_remote_t ext_robot_command_remote; // 0x0304
    } Data;
    uint16_t CRC16;

} Frame_t;

typedef __packed struct
{
    FrameHeader_t FrameHeader;
    CmdID_e CmdID;
    ext_game_state_t ext_game_state;                             // 0x0001
    ext_game_result_t ext_game_result;                           // 0x0002
    ext_game_robot_survivors_t ext_game_robot_survivors;         // 0x0003
    ext_dart_status_t ext_dart_status;                           // 0x0004
    ext_ICRA_buff_debuff_zone_status_t ext_ICRA_buff_status;     // 0x0005
    ext_event_data_t ext_event_data;                             // 0x0101
    ext_supply_projectile_action_t ext_supply_projectile_action; // 0x0102
    /*  ext_supply_projectile_booking_t		  ext_supply_projectile_booking; */
    ext_referee_warning_t ext_referee_warning;           // 0x0104
    ext_dart_remaining_time_t ext_dart_remaining_time;   // 0x0105
    ext_game_robot_status_t ext_game_robot_state;        // 0x0201
    ext_power_heat_data_t ext_power_heat_data;           // 0x0202
    ext_game_robot_pos_t ext_game_robot_pos;             // 0x0203
    ext_buff_musk_t ext_buff_musk;                       // 0x0204
    aerial_robot_energy_t aerial_robot_energy;           // 0x0205
    ext_robot_hurt_t ext_robot_hurt;                     // 0x0206
    ext_shoot_data_t ext_shoot_data;                     // 0x0207
    ext_bullet_remaining_t ext_bullet_remaining;         // 0x0208
    ext_rfid_status_t ext_rfid_status;                   // 0x0209
    ext_dart_client_cmd_t ext_dart_client_cmd;           // 0x020A
    ext_send_user_data_t ext_send_user_data;             // 0x0301
    robot_interactive_data_t robot_interactive_data;     // 0x0302
    ext_robot_command_map_t ext_robot_command_map;       // 0x0303
    ext_robot_command_remote_t ext_robot_command_remote; // 0x0304

} DJI_Data_t;

extern RefReceiveTime_s RefReceiveTime; // 裁判系统相关有效数据接收到的时间

/**
 * @brief: 初始化裁判系统
 * @param {None}
 * @return {*}
 */
rt_err_t DJI_Init(void);

#endif
