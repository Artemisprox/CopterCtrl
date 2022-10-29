#ifndef __DRV_SMOOTH_H_
#define __DRV_SMOOTH_H_
#include <rtthread.h>

/*平滑线程周期*/
/*不能比SMOOTH_PERIOD大（不然消费者的速度小于生产者，这样实际平滑的加速度就是SMOOTH_ADD/线程的时间周期）*/
#define SMOOTH_THREAD_PERIOD    1       //单位：ms,最好设成1ms

/* 以下参数都必须 > 0 */
/*底盘速度设定值的平滑，SMOOTH_ADD/SMOOTH_PERIOD m/s^2（或100°/s^2） (类型float)*/
#define SMOOTH_PERIOD_X		    1       //设定值平滑改变周期，单位：ms，(不能小于1)
#define SMOOTH_ADD_UP_X			3       //一个平滑改变周期增加的速度，单位：mm/s
#define SMOOTH_ADD_DOWN_X		4       //一个平滑改变周期减少的速度，单位：mm/s
#define SMOOTH_MODE_X           SM_MIRROR

#define SMOOTH_PERIOD_Y		    1       //设定值平滑改变周期，单位：ms，(不能小于1)
#define SMOOTH_ADD_UP_Y			3       //一个平滑改变周期增加的速度，单位：mm/s
#define SMOOTH_ADD_DOWN_Y		4       //一个平滑改变周期减少的速度，单位：mm/s
#define SMOOTH_MODE_Y           SM_MIRROR

#define SMOOTH_PERIOD_AC		1       //设定值平滑改变周期，单位：ms，(不能小于1)
#define SMOOTH_ADD_UP_AC_1		5       //一个平滑改变周期增加的角速度，单位：0.1°/s
#define SMOOTH_ADD_DOWN_AC_1    5       //一个平滑改变周期减少的角速度，单位：0.1°/s  
#define SMOOTH_ADD_UP_AC_2		10      //第二组参数
#define SMOOTH_ADD_DOWN_AC_2    10       
#define SMOOTH_MODE_AC          SM_MIRROR


/*平滑类型*/
typedef enum
{
    //底盘速度的平滑
    WHEEL_XSPEED,
    WHEEL_YSPEED,
    //底盘角速度的平滑
    WHEEL_ACSPEED,
    SMOOTH_NUM,

} Smooth_e;

/*平滑变化方向*/
typedef enum
{
    ADD_UP = 1,     //赋值不可改
    ADD_DOWN = -1,
    
} Smooth_dir_e;

/*平滑模式*/
typedef enum
{
    SM_NORMAL,       //正常模式：值变减小时，使用SMOOTH_ADD_UP，增大时，使用SMOOTH_ADD_DOWN。
    SM_MIRROR,       //镜像模式：当值小于0时（大于0时和正常模式一样），值减小时，用的加速度大小为SMOOTH_ADD_UP，增大时，使用SMOOTH_ADD_DOWN。
    
} Smooth_mode_e;

/*平滑数值结构体*/
typedef struct
{
    float in;       //输入的设定值
    float out;      //随时间改变的平滑速度，会以恒定加速度增加或者减少
    float final;    //平滑的最终速度设定值

} Smooth_value_t;

/*平滑结构体*/
typedef struct
{
    Smooth_value_t  value;
    rt_uint32_t     count;              //平滑，随时间计数值
    rt_bool_t       if_smoothing;       //是否正在平滑
    rt_bool_t       if_start;           //是否启动平滑
    Smooth_dir_e    add_dir;            //平滑变大还是变小

    rt_uint16_t     period;             //设定值平滑改变周期，单位：ms
    rt_uint16_t     per_add_up;         //每次平滑增加的值大小
    rt_uint16_t     per_add_down;       //每次平滑减少的值大小

    Smooth_mode_e   mode;               //平滑方式

} Smooth_t;


#define SMOOTH_ADDING(add_up,add_down)      do{\
                                                if(smo->add_dir == ADD_UP)\
                                                    Smooth_Adding(smo,add_up);\
                                                else if(smo->add_dir == ADD_DOWN)\
                                                    Smooth_Adding(smo,add_down);\
                                                } while(0);


/**
 * @brief   平滑相关设备初始化
 * @param   None
 * @return  rt_err_t 是否初始化成功
 * @author  lfp
 */
rt_err_t Smooth_Init(void);

/**
 * @brief  平滑输入
 * @param  kind 平滑类型
 * @param  in   数值输入
 * @return None
 */
void Smooth_In(Smooth_e kind,float in);

/**
 * @brief  平滑输出,同时输出等于输入
 * @param  kind 平滑类型
 * @return 数值输出
 */
float Smooth_Out(Smooth_e kind);

/**
 * @brief  重置当前值，
 *         当smooth[kind].value的实际值与上一次最终设定值不符合，
 *         重新刷新上一次最终值。
 * @param  kind  平滑类型
 * @param  value 数值
 * @return None
 */
void Smooth_Refresh_Nowvalue(Smooth_e kind,float value);

/**
 * @brief  启动平滑
 * @param  kind 平滑类型
 * @return None
 */
void Smooth_Start(Smooth_e kind);

/**
 * @brief  关闭平滑,同时输出等于输入
 * @param  kind 平滑类型
 * @return None
 */
void Smooth_Close(Smooth_e kind);

/**
 * @brief  修改平滑加速度
 * @param  kind 平滑类型
 * @param  add_up   一次周期的数值增加量,必须>0
 * @param  add_down 一次周期的数值减少量,必须>0
 * @return None
 */
void Smooth_Modify_Add(Smooth_e kind,rt_int16_t add_up,rt_int16_t add_down);


#endif

