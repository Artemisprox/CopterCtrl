#include "mod_motion.h"
#include "drv_smooth.h"
#include "pid.h"
#include "drv_utils.h"
#include "drv_wheel.h"
#include "drv_EnergyConservation.h"

pid_t angfol_pid;

static int AngFollow_PID_Init(void)
{
    pid_init(&angfol_pid, 50, 0, 0, 0, 2200, -2200);
    return RT_EOK;
}
INIT_BOARD_EXPORT(AngFollow_PID_Init);

/**
 * @brief   输入xy速度平滑模块
 * @param   Vxy xy速度矢量
 * @return  平滑后值
 */
static Vector2_t MotModule_Smooth_InputXY(Vector2_t Vxy)
{
    Smooth_In(WHEEL_XSPEED, Vxy.x);
    Smooth_In(WHEEL_YSPEED, Vxy.y);
    Vxy.x = Smooth_Out(WHEEL_XSPEED);
    Vxy.y = Smooth_Out(WHEEL_YSPEED);
    return Vxy;
}

/**
 * @brief   输出角速度平滑模块
 * @param   angvel 角速度
 * @return  平滑后值
 */
static float MotModule_Smooth_OutputW(float angvel)
{
    Smooth_In(WHEEL_ACSPEED, angvel);
    return Smooth_Out(WHEEL_ACSPEED);
}

/**
 * @brief   低电量输出增益保护
 * @param   VxyW   速度矢量和角速度矢量
 * @return  增益后Mot_base_t数据
 */
static Mot_base_t MotModule_Low_OutGain(Mot_base_t VxyW, float fol_angle)
{
    const float gain_range = 30;         //增益的角度范围，单位°
    const float k_w = 0.8f, k_xy = 0.4f; //增益系数/增益幅度

    /*将error变换到设定值左右的(-180.0°,180.0°]*/
    float error = CIRCLE_SHORTEST_DIS(MotModule_Get_YawGc(), fol_angle, -180.0f, 180.0f);

    /*当跟随角度偏差量在gain_range~180°之间时，增大角速度，偏差越大增益越大*/
    float w_gain = utils_clamp(fabsf(error) - gain_range, 0, 180) / utils_clamp(180.0f - gain_range, 0, 180) * k_w;
    VxyW.angvel += VxyW.angvel * w_gain;

    /*当跟随角度偏差量在gain_range~180°之间时，减小角速度，偏差越大衰减越大*/

    float xy_gain = utils_clamp(180.0f - fabsf(error), 0, 180) / utils_clamp(180.0f - gain_range, 0, 180) * k_xy;
    VxyW.vel.x -= VxyW.vel.x * xy_gain;
    VxyW.vel.y -= VxyW.vel.y * xy_gain;

    return VxyW;
}

/**
 * @brief   输入幅度限制模块1
 * @param   VxyW   速度矢量和角速度矢量
 * @return  限制后Mot_base_t数据
 */
static Mot_base_t MotModule_Clamp_Base(Mot_base_t VxyW)
{
    Mot_base_t temp =
        {
            .vel.y = utils_clamp(VxyW.vel.y, -M_MAX_YSPEED, M_MAX_YSPEED),
            .vel.x = utils_clamp(VxyW.vel.x, -M_MAX_XSPEED, M_MAX_XSPEED),
            .angvel = utils_clamp(VxyW.angvel, -M_MAX_ROTATE_AC, M_MAX_XSPEED)};
    return temp;
}

/**
 * @brief   输入幅度限制模块2
 * @param   pos   位置坐标
 * @return  限制后Vector2_t数据
 */
static Vector2_t MotModule_Clamp_Pos(Vector2_t pos)
{
    Vector2_t temp =
        {
            .y = utils_clamp(pos.y, -M_MAX_POSITION_Y, M_MAX_POSITION_Y),
            .x = utils_clamp(pos.x, -M_MAX_POSITION_X, M_MAX_POSITION_X)};
    return temp;
}

/**
 * @brief   矢量坐标转换模块(默认:云台坐标系->底盘坐标系)
 * @param   speed   速度矢量,单位：mm/s
 * @param   theta   坐标系之间的夹角(默认：底盘xy坐标系y轴 相对于云台xy坐标系y轴 旋转的角度，逆时针为正方向。单位°，范围(-180°~180°].)
 * @return  投影后的速度矢量,单位：mm/s
 */
static Vector2_t MotModule_Transform(Vector2_t speed, float theta)
{
    Vector2_t pro_speed;

    theta = theta / 180.0f * Lx_PI; //转化为弧度

    pro_speed.x = speed.y * sinf(theta) + speed.x * cosf(theta); // x'=ysinθ+xcosθ
    pro_speed.y = speed.y * cosf(theta) - speed.x * sinf(theta); // y'=ycosθ-xsinθ

    return pro_speed;
}

/**
 * @brief   矢量坐标逆转换模块(默认:底盘坐标系->云台坐标系)
 * @param   speed   速度矢量,单位：mm/s
 * @param   theta   坐标系之间的夹角(默认：底盘xy坐标系y轴 相对于云台xy坐标系y轴 旋转的角度，逆时针为正方向。单位°，范围(-180°~180°].)
 * @return  投影后的速度矢量,单位：mm/s
 */
static Vector2_t MotModule_Inverse_Transform(Vector2_t speed, float theta)
{
    Vector2_t pro_speed;

    theta = theta / 180.0f * Lx_PI; //转化为弧度

    pro_speed.x = -speed.y * sinf(theta) + speed.x * cosf(theta); // x'=-ysinθ+xcosθ
    pro_speed.y = speed.y * cosf(theta) + speed.x * sinf(theta);  // y'=ycosθ+xsinθ

    return pro_speed;
}

/**
 * @brief   偏心运动模块（将绕某点旋转转化为底盘中心的速度矢量）
 * @param   pos    旋转轴坐标，单位 mm。在二维的底盘坐标系上，正前方为 +y轴，右手边为 +x轴。
 * @param   angvel 自旋速度大小，单位 0.1°/s，正方向俯视图逆时针。
 * @return  中心的速度矢量,单位：mm/s
 */
static Vector2_t MotModule_Eccentric(Vector2_t pos, float angvel) // Vector2_t postion,float angvel)
{
    /*角速度矢量 ω*/
    Vector3_t vector_w = {.x = 0, .y = 0, .z = angvel, 0, 0};

    /*底盘中心相对于以旋转点为原点的xy坐标系的位置矢量，单位mm*/
    Vector3_t vector_axis = Get_Vector_3From2(pos, -1);

    /*υ = ω x r，计算底盘中心的速度矢量，单位 0.1°*mm/s*/
    vector_axis = Vector3_X(vector_w, vector_axis);

    /*单位转换:0.1°*mm/s -> mm/s*/
    return Get_Vector_2From3(vector_axis, Lx_PI / 1800.0f);
}

/**
 * @brief   公转运动模块（将绕某点旋转转化为底盘中心随时间自旋的速度矢量）
 * @param   pos   旋转轴坐标，单位 mm。在二维的底盘坐标系上，正前方为 +y轴，右手边为 +x轴。
 * @param   angvel  公转角速度大小，单位 0.1°/s，正方向俯视图逆时针。
 * @param   record   由调用MotModule_Revolve函数的上下文提供一个静态或者全局的变量,record为该变量的指针
 * @return  中心随时间旋转的速度矢量,单位：mm/s
 */
static Vector2_t MotModule_Revolve(Vector2_t pos, float angvel, Mot_Record_t *record)
{
    /*如果旋转点和公转速度发生改变时，重置*/
    if (!VECTOR2_CMP(record->last_pos, pos) || record->last_angvel != angvel)
    {
        record->incre_theta = 0;
        record->last_pos = pos;
        record->last_angvel = angvel;
    }

    /*计算切向的速度矢量*/
    Vector3_t vector_center = Get_Vector_3From2(MotModule_Eccentric(pos, angvel), 1);
    Get_Vector3_A_theta(&vector_center);

    /* ∆θ = ω * ∆t ,计算速度矢量的角度增量，单位°*/
    float period = RT_TICK_PROBE(&record->tick);             //单位ms
    record->incre_theta += period / 1000.0f * angvel * 0.1f; //公转角速度，单位°/s
    utils_norm_circle_number(&record->incre_theta, -180.0f, 360.f);

    /*计算旋转后的速度矢量x,y*/
    vector_center.xy_theta += record->incre_theta;
    Get_Vector3_From_A_theta(&vector_center);

    /*转化为二维矢量*/
    return Get_Vector_2From3(vector_center, 1);
}

/**
 * @brief   跟随角pid计算模块
 * @note    set和now angle只要是同一个循环范围就行，例：set范围0~360°；now范围-180~180°
 * @param   set_angle  设定角度值，单位°
 * @param   now_angle  当前yaw角度值，单位°
 * @return  角速度：单位0.1°/s
 */
static float MotModule_FollowPid(float set_angle, float now_angle)
{
    /*将error变换到设定值左右的(-180.0°,180.0°]*/
    float error = CIRCLE_SHORTEST_DIS(now_angle, set_angle, -180.0f, 180.0f);

    /*pid计算*/
    PID_Calculate(&angfol_pid, error);

    return angfol_pid.out;
}

/**
 * @brief    只有底盘运动
 * @param    VxyW.vel   底盘xy速度矢量，单位mm/s
 * @param    VxyW.angvel   底盘自转角速度大小，单位0.1°/s
 * @return   输出到Resolve函数的VxyW
 */
Mot_base_t MotPack_Only_Chass(Mot_base_t VxyW)
{
    VxyW = MotModule_Clamp_Base(VxyW);
    return VxyW;
}

/**
 * @brief    小陀螺运动
 * @note     当 VxyW.angvel == 0时,为不跟随的底盘全向移动运动
 * @param    VxyW.angvel 小陀螺的自转角速度，单位0.1°/s
 * @param    VxyW.vel    云台方向的平移速度矢量，单位mm/s
 * @return   输出到Resolve函数的VxyW
 */
Mot_base_t MotPack_Small_Top(Mot_base_t VxyW)
{
    /*输入限制*/
    VxyW = MotModule_Clamp_Base(VxyW);

    /*获取云台和底盘的夹角*/
    float yaw_angle = MotModule_Get_YawGc();

    /*将云台的速度矢量转化到底盘坐标系上*/
    VxyW.vel = MotModule_Transform(VxyW.vel, yaw_angle);
    return VxyW;
}

/**
 * @brief    底盘跟随运动
 * @note     输入的VxyW.angvel没有使用
 * @param    VxyW.vel    云台方向的平移速度矢量，单位mm/s
 * @param    fol_angle   跟随角,(俯视图下，云台枪管在底盘车头的顺时针方位时 >0。范围：0-360°)
 * @return   输出到Resolve函数的VxyW
 */
Mot_base_t MotPack_Follow_Gim(Mot_base_t VxyW, float fol_angle)
{
    /*输入限制*/
    VxyW = MotModule_Clamp_Base(VxyW);
    utils_norm_circle_number(&fol_angle, -180.0f, 360.f);

    /*获取云台和底盘的夹角*/
    float yaw_angle = MotModule_Get_YawGc();

    /*若移动速度较大说明需要节能处理*/
    Follow_Gimbal_Energy_Fix(&VxyW.vel, &fol_angle, yaw_angle);

    /*将云台的速度矢量转化到底盘坐标系上*/
    VxyW.vel = MotModule_Transform(VxyW.vel, yaw_angle);

    /*计算跟随时的动态角速度大小*/
    VxyW.angvel = MotModule_FollowPid(fol_angle, yaw_angle);
    return VxyW;
}

/**
 * @brief    偏心的只有底盘运动
 * @param    VxyW.vel   底盘xy速度矢量，单位mm/s
 * @param    VxyW.angvel   底盘自转角速度大小，单位0.1°/s
 * @param    Pxy    偏心坐标点，单位(mm,mm)
 * @return   输出到Resolve函数的VxyW
 */
Mot_base_t MotPack_Offset_OnlyChass(Mot_base_t VxyW, Vector2_t pos)
{
    /*输入限制*/
    VxyW = MotModule_Clamp_Base(VxyW);
    pos = MotModule_Clamp_Pos(pos);

    /*计算切向速度*/
    Vector2_t V_offset = MotModule_Eccentric(pos, VxyW.angvel);

    /*矢量叠加*/
    VxyW.vel = Vector2_Add(VxyW.vel, V_offset);
    return VxyW;
}

/**
 * @brief    偏心小陀螺运动
 * @param    VxyW.vel   云台的xy速度矢量，单位mm/s
 * @param    VxyW.angvel   小陀螺的自转角速度，单位0.1°/s
 * @param    Pxy    偏心坐标点，单位(mm,mm)
 * @return   输出到Resolve函数的VxyW
 */
Mot_base_t MotPack_Offset_SmallTop(Mot_base_t VxyW, Vector2_t pos)
{
    /*输入限制*/
    VxyW = MotModule_Clamp_Base(VxyW);
    pos = MotModule_Clamp_Pos(pos);

    /* 两个MotPack互相独立,直接输出叠加 */
    /* MotPack：MotPack_Small_Top */
    float yaw_angle = MotModule_Get_YawGc();
    VxyW.vel = MotModule_Transform(VxyW.vel, yaw_angle);

    /* MotPack：Motion_Offset_OnlyChass */
    Vector2_t V_offset = MotModule_Eccentric(pos, VxyW.angvel);
    VxyW.vel = Vector2_Add(VxyW.vel, V_offset);
    return VxyW;
}

/**
 * @brief    偏心跟随运动
 * @param    VxyW.vel    云台方向的平移速度矢量，单位mm/s
 * @param    Pxy    偏心坐标点，单位(mm,mm)
 * @param    fol_angle   跟随角,(俯视图下，云台枪管在底盘车头的顺时针方位时 >0。范围：0-360°)
 * @return   输出到Resolve函数的VxyW
 * @author   lfp
 */
Mot_base_t MotPack_Offset_FollowGim(Mot_base_t VxyW, Vector2_t pos, float fol_angle)
{
    /*输入限制*/
    VxyW = MotModule_Clamp_Base(VxyW);
    pos = MotModule_Clamp_Pos(pos);
    utils_norm_circle_number(&fol_angle, -180.0f, 360.f);

    /* V_offset随 Follow_Gim模块输出变化 */
    /* MotPack：MotPack_Follow_Gim */
    float yaw_angle = MotModule_Get_YawGc();
    VxyW.vel = MotModule_Transform(VxyW.vel, yaw_angle);

    /*移动速度较大说明需要节能处理*/
    Follow_Gimbal_Energy_Fix(&VxyW.vel, &fol_angle, yaw_angle);

    VxyW.angvel = MotModule_FollowPid(fol_angle, yaw_angle);

    /* MotPack：Motion_Offset_OnlyChass */
    Vector2_t V_offset = MotModule_Eccentric(pos, VxyW.angvel);
    VxyW.vel = Vector2_Add(VxyW.vel, V_offset);
    return VxyW;
}

/**
 * @brief    绕点旋转运动
 * @param    Pxy 偏心坐标点，单位(mm,mm)
 * @param    dot_angvel 公转角速度大小，单位0.1°/s
 * @return   输出到Resolve函数的VxyW
 */
Mot_base_t MotPack_Spin_Dot(Vector2_t pos, float dot_angvel)
{
    static Mot_Record_t storage;

    pos = MotModule_Clamp_Pos(pos);
    dot_angvel = utils_clamp(dot_angvel, -M_MAX_REVOLVE_AC, M_MAX_REVOLVE_AC);

    Vector2_t velocity = MotModule_Revolve(pos, dot_angvel, &storage);
    return (Mot_base_t){.vel = velocity, .angvel = 0};
    ;
}

/**
 * @brief    修改跟随的角度pid参数,暂时用在test调试上
 * @param    kp-ki-kd    pid参数
 * @return   None
 */
void ExMotMod_Modify_Apid(float kp, float ki, float kd)
{
    angfol_pid.kp = kp;
    angfol_pid.ki = ki;
    angfol_pid.kd = kd;
}

/**
 * @brief   获取xy设定的距离增量
 * @param   None
 * @return  Vector2_t 距离增量，单位μm,使用时要打开SMOOTH_VELOCITY_ENABLE宏
 */
Vector2_t ExMotMod_Get_xyDelta(void)
{
    Vector2_t Delta;
    float period;
    static Vector2_t last_speed = {0, 0};
    TICK_PROBE_STORAGE(store);

    /*读取经过平滑后的速度设定值*/
    Vector2_t now_speed = {Smooth_Out(WHEEL_XSPEED), Smooth_Out(WHEEL_YSPEED)};

    /*获取时间间隔*/
    period = RT_TICK_PROBE(&store);

    /*计算时间间隔内运动的距离*/
    Delta = Vector2_Add(last_speed, now_speed);
    Delta = Vector2_Gain(Delta, period / 2.0f);

    last_speed = now_speed;
    return Delta;
}

/**
 * @brief   统一的Mot_base_t类型数据输入接口
 * @param   VxyW    xy速度,角速度
 * @return  处理后的VxyW值
 */
Mot_base_t ExMotMod_Input(Mot_base_t VxyW, Motion_mode_e mode)
{
    float yaw_angle;
    /*当只有底盘模式时，先将设定值投影到云台系（防止因平滑时的系不同导致错误的运动情况）*/
    if (mode == CHASSIS_ONLY || mode == OFFSET_ONLY)
    {
        yaw_angle = MotModule_Get_YawGc();
        VxyW.vel = MotModule_Inverse_Transform(VxyW.vel, yaw_angle);
    }

    /*如果开启了输入平滑宏，在切换到没有使用该函数的输出作为输入的模式时，
    (实际的xy设定速度与smooth里存储的变量不同)必须在退出触发函数里重置平滑实际值，否则平滑会中断*/
    VxyW.vel = SMOOTH_INPUTXY(VxyW.vel);

    /*当只有底盘模式时，再从云台系重新投影回底盘系*/
    if (mode == CHASSIS_ONLY || mode == OFFSET_ONLY)
    {
        VxyW.vel = MotModule_Transform(VxyW.vel, yaw_angle);
    }

    return VxyW;
}

float SetSpeed[WHEELS_NUM] = {0.f, 0.f, 0.f, 0.f}; // 底盘电机转速设定值

/**
 * @brief   统一的Mot_base_t类型数据输出接口
 * @param   VxyW    xy速度,角速度
 * @return  None
 */
void ExMotMod_Output(Mot_base_t VxyW, Motion_mode_e mode, float fol_angle)
{
#if defined(LOW_POWER_PROTECT_ENABLE)
    /*如果当前为跟随模式，且超级电容没电了则对输出进行控制*/
    if (Wheels_Read_ScProtectState() && (mode == FOLLOW_GIMBAL || mode == OFFSET_FOLLOW))
        VxyW = MotModule_Low_OutGain(VxyW, fol_angle);
#endif
    // xy 轴数据换系平滑
    VxyW = ExMotMod_Input(VxyW, mode);

    // 角速度设定设定值平滑
    VxyW.angvel = SMOOTH_OUTPUTW(VxyW.angvel);

    /*将底盘的设定运动转化到底盘各电机的运动上*/
    MotModule_Resolve(VxyW, SetSpeed);
    /*将计算出来的转速设定值写入底盘控制线程*/
    for (int i = 0; i < (int)WHEELS_NUM; ++i)
        Wheel_Speed_Set((Wheel_local_e)i, SetSpeed[i]);
}
