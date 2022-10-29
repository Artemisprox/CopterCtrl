#include "drv_smooth.h"
#include "HThread_data.h"

static Smooth_t smooth[SMOOTH_NUM];

static struct rt_timer smooth_timer;
static void Smooth_IRQHandler(void *parameter)
{
    static rt_uint16_t _cnt[SMOOTH_NUM] = {0};

    for(int i = 0; i < SMOOTH_NUM; i++)
    {
        _cnt[i] ++;

        /*以SMOOTH_PERIOD周期增加smo->count值*/
        if(_cnt[i] % smooth[i].period == 0)
        {
            /*是否在平滑*/
            if(smooth[i].if_smoothing == RT_TRUE)
                smooth[i].count ++;//生产
            else
                smooth[i].count = 0;

            _cnt[i] = 0;
        }
    }
}


/**
 * @brief   平滑的处理函数，在Smooth_Value内被调用
 * @param   smo Smooth_t指针
 * @param   per_add 每次增加的大小，输入>0
 * @return  None
 * @author  lfp
 */
static void Smooth_Adding(Smooth_t* smo,rt_uint16_t per_add)
{
    /*smo->value.out平滑速度设定值是否增加到最终值附近*/
    if(smo->value.out - smo->value.final <   per_add * SMOOTH_THREAD_PERIOD  &&
       smo->value.out - smo->value.final > - per_add * SMOOTH_THREAD_PERIOD)
    {
        smo->if_smoothing = RT_FALSE;
        smo->value.out = smo->value.final;
    }
    else/*如果未完成平滑*/
    {
        if(smo->count > 0)
        {
            smo->value.out += (float)per_add * (float)smo->add_dir;
            smo->count --;//消费
        }
    }
}


/**
 * @brief   平滑函数
 * @param   smo Smooth_t指针
 * @return  None
 * @author  lfp
 */
static void Smooth_Value(Smooth_t* smo)
{
    /*改变速度设定值，平滑初始阶段*/
    if(smo->value.final != smo->value.in)
    {
        /*启动平滑*/
        smo->if_smoothing = RT_TRUE;

        /*记录这次的输入值*/
        smo->value.final = smo->value.in;

        /*判断加速还是减速*/
        if(smo->value.in > smo->value.out)
            smo->add_dir = ADD_UP;
        else
            smo->add_dir = ADD_DOWN;
    }

    /*平滑进行阶段*/
    if(smo->if_smoothing == RT_TRUE)
    {
        switch (smo->mode)
        {
        case SM_NORMAL:
            SMOOTH_ADDING(smo->per_add_up,smo->per_add_down)
            break;

        case SM_MIRROR:
            if(smo->value.out >= 0)
                SMOOTH_ADDING(smo->per_add_up,smo->per_add_down)
            else//当值小于0时，上升和下降的加速度倒置
                SMOOTH_ADDING(smo->per_add_down,smo->per_add_up)
            break;
        
        default:
            break;
        } 
    }
}


/**
 * @brief   平滑线程
 * @param   parameter
 * @return  None
 * @author  lfp
 */
static void Smooth_Thread(void* parameter)
{
    while(1)
    {
        for(int i = 0; i < SMOOTH_NUM; i++)
        {
            /*是否启动平滑*/
            if(smooth[i].if_start == RT_TRUE)
                Smooth_Value(&smooth[i]);
        }

        rt_thread_mdelay(SMOOTH_THREAD_PERIOD);
    }
}


/**
 * @brief   平滑结构体初始化
 * @param   smo      Smooth_t指针
 * @param   period   一次平滑改变周期
 * @param   add_up   一次周期的数值增加量,必须>0
 * @param   add_down 一次周期的数值减少量,必须>0
 * @return  None
 * @author  lfp
 */
static void Smooth_Struct_Init(Smooth_t* smo,rt_uint16_t period,rt_int16_t add_up,rt_int16_t add_down,Smooth_mode_e mode)
{
    smo->value.in = 0;
    smo->value.out = 0;
    smo->value.final = 0;

    smo->count = 0;
    smo->if_smoothing = RT_FALSE;
    smo->if_start = RT_TRUE;//默认启动
    smo->add_dir = ADD_UP;//默认增大

    smo->period = period;

    /*输入正负限制*/
    if(add_up < 0)
        add_up = -add_up;
    if(add_down < 0)
        add_down = -add_down;   

    smo->per_add_up = add_up;
    smo->per_add_down = add_down;

    smo->mode = mode;
}


/**
 * @brief   平滑相关设备初始化
 * @param   None
 * @return  rt_err_t 是否初始化成功
 * @author  lfp
 */
rt_err_t Smooth_Init(void)
{
    rt_err_t res;
    rt_thread_t thread = RT_NULL;

    //初始化结构体
    Smooth_Struct_Init(&smooth[WHEEL_XSPEED],SMOOTH_PERIOD_X,SMOOTH_ADD_UP_X,SMOOTH_ADD_DOWN_X,SMOOTH_MODE_X);
    Smooth_Struct_Init(&smooth[WHEEL_YSPEED],SMOOTH_PERIOD_Y,SMOOTH_ADD_UP_Y,SMOOTH_ADD_DOWN_Y,SMOOTH_MODE_Y);
    Smooth_Struct_Init(&smooth[WHEEL_ACSPEED],SMOOTH_PERIOD_AC,SMOOTH_ADD_UP_AC_1,SMOOTH_ADD_DOWN_AC_1,SMOOTH_MODE_AC);

    //初始化底盘线程
    thread = rt_thread_create("Smooth_Thread",		    //线程名
                               Smooth_Thread,	        //线程入口
                               RT_NULL,					//入口参数无
                               THREAD_STACK_SMOOTH, 	//线程栈
                               THREAD_PRIO_SMOOTH,	    //线程优先级
                               THREAD_TICK_SMOOTH);	    //线程时间片大小

    //线程创建失败返回false
    if(thread == RT_NULL)
        return RT_ERROR;

    //线程启动失败返回false
    if(rt_thread_startup(thread) != RT_EOK)
        return RT_ERROR;

    //创建线程定时器
    rt_timer_init(&smooth_timer,
                  "smooth_timer",
                  Smooth_IRQHandler,
                  RT_NULL,
                  1,
                  RT_TIMER_FLAG_PERIODIC | RT_TIMER_FLAG_SOFT_TIMER);

    //启动定时器
    res = rt_timer_start(&smooth_timer);
    if(res != RT_EOK)
        return res;

    return RT_EOK;
}
///////////////////////////////////////对外接口/////////////////////////////////////////
/**
 * @brief  平滑输入
 * @param  kind 平滑类型
 * @param  in   数值输入
 * @return None
 */
void Smooth_In(Smooth_e kind,float in)
{
    if(kind != SMOOTH_NUM)
        smooth[kind].value.in = in;
    else
        return;
}


/**
 * @brief  平滑输出
 * @param  kind 平滑类型
 * @return 数值输出
 */
float Smooth_Out(Smooth_e kind)
{
    if(kind != SMOOTH_NUM)
    {
        if(smooth[kind].if_start == RT_TRUE)
            return smooth[kind].value.out;
        else
            return smooth[kind].value.in;//未启动平滑，输出输入值
    }
    else
        return 0;
}


/**
 * @brief  重置当前值，
 *         当smooth[kind].value的实际值与上一次最终设定值不符合，
 *         重新刷新上一次最终值。
 * @note   应用于外部强行切断了对某个值的平滑，同时改变了该值。此时切回来需要同步该值结构体的某些值
 * @param  kind  平滑类型
 * @param  value 数值
 * @return None
 */
void Smooth_Refresh_Nowvalue(Smooth_e kind,float value)
{
    if(kind != SMOOTH_NUM)
    {
        smooth[kind].value.final = value;
        smooth[kind].if_smoothing = RT_FALSE;//重新开始平滑
    }
    else
        return ;
}


/**
 * @brief  启动平滑
 * @param  kind 平滑类型
 * @return None
 */
void Smooth_Start(Smooth_e kind)
{
    if(kind != SMOOTH_NUM)
        smooth[kind].if_start = RT_TRUE;
    else
        return;
}


/**
 * @brief  关闭平滑,同时输出等于输入
 * @param  kind 平滑类型
 * @return None
 */
void Smooth_Close(Smooth_e kind)
{
    if(kind != SMOOTH_NUM)
    {
        smooth[kind].if_start = RT_FALSE;
        smooth[kind].if_smoothing = RT_FALSE;
        smooth[kind].count = 0;
    }
    else
        return;
}


/**
 * @brief  修改平滑加速度
 * @param  kind 平滑类型
 * @param  add_up   一次周期的数值增加量,必须>0
 * @param  add_down 一次周期的数值减少量,必须>0
 * @return None
 */
void Smooth_Modify_Add(Smooth_e kind,rt_int16_t add_up,rt_int16_t add_down)
{
    if(kind != SMOOTH_NUM)
    {
        /*输入正负限制*/
        if(add_up < 0)
            add_up = -add_up;
        if(add_down < 0)
            add_down = -add_down;   

        smooth[kind].per_add_up = add_up;
        smooth[kind].per_add_down = add_down;
    }
    else
        return;
}

