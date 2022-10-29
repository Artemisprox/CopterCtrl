#include "func_GimbalPID.h"
#include "pid.h"
#include "robodata.h"

static rt_int16_t Count_Max = PAL_SCALE_SET * ANG_SCALE_SET;

// 定义用来记录云台角度环PID输出限幅的结构体
// 在上电缓动阶段结束后，程序会将PID结构体中的数据恢复到此结构体中的数值
static struct
{
    float gyro_Angle_pid_OutPutMaxUp_set;
    float gyro_Angle_pid_OutPutMaxDown_set;
} Gimbal_Pitch_Para_Rem_str;

void Gimbal_SoftStart_Ctrl(pid_t *Motor_Pitch_PID);

float AnglePID_OUT;//角度环输出
/**
* @brief pitch云台PID运行函数（包括角度环角速度环）
* @param [gimbalmotor_t*] motor 云台电机结构体
* @param [float] nowPal 当前角速度值
* @param [float] nowAngle 当前角度值
* @param [float] FeedForwardRate_Set 前馈系数
* @return 无
* @author zzj
*/
void Gimbal_Pitch_PID_RUN(Motor_t *motor,float nowPal,float nowAngle, float FeedForwardRate_Set)
{
    static rt_uint8_t Count = 1;//计数值初始值设为1

    Gimbal_SoftStart_Ctrl(&motor->ang);//上电缓启动
	
    /* 运行PID */
    if(Count%PAL_SCALE_SET == 0)
    {//角速度环
        Motor_SpeedPIDCalculate(motor, nowPal);
    }
    if(Count%ANG_SCALE_SET == 0)
    {//角度环
        AnglePID_OUT = Motor_FrontAnglePIDCalculate(motor, nowAngle, FeedForwardRate_Set, ANG_SCALE_SET);
        Motor_Write_SetSpeed_ABS(motor, AnglePID_OUT);
    }
    Count ++; // 计数值加一
    if(Count >= Count_Max)
    {
        Count = 0;
    }
}

/**
* @brief yaw云台PID运行函数（包括角度环角速度环）
* @param [gimbalmotor_t*] motor 云台电机结构体
* @param [float] nowPal 当前角速度值
* @param [float] nowAngle 当前角度值
* @param [float] FeedForwardRate_Set 前馈系数
* @return 无
* @author zzj
*/
void Gimbal_Yaw_PID_RUN(Motor_t *motor,float nowPal,float nowAngle, float FeedForwardRate_Set)
{
    static rt_uint8_t Count = 1;//计数值初始值设为1

    float AnglePID_OUT;//角度环输出

    /* 运行PID */
    if(Count%PAL_SCALE_SET == 0)
    {//角速度环
        Motor_SpeedPIDCalculate(motor, nowPal);
    }
    if(Count%ANG_SCALE_SET == 0)
    {//角度环
        AnglePID_OUT = Motor_FrontAnglePIDCalculate(motor, nowAngle, FeedForwardRate_Set, ANG_SCALE_SET);
        Motor_Write_SetSpeed_ABS(motor, AnglePID_OUT);
    }
    Count ++; // 计数值加一
    if(Count >= Count_Max)
    {
        Count = 0;
    }
}

/**
* @brief：用于控制上电时的云台缓动，通过修改角度环PID输出限幅实现
* @param [pid_t*]	Motor_Pitch_PID：云台Pitch闭环PID结构体
* @author：ych
*/
void Gimbal_SoftStart_PitchPara_Set(pid_t *Motor_Pitch_PID)
{
    // 备份参数
    Gimbal_Pitch_Para_Rem_str.gyro_Angle_pid_OutPutMaxUp_set = Motor_Pitch_PID->out_limit_up;
    Gimbal_Pitch_Para_Rem_str.gyro_Angle_pid_OutPutMaxDown_set = Motor_Pitch_PID->out_limit_down;
    // 修改参数
    Motor_Pitch_PID->out_limit_up = SOFTSTART_PALLIM_SET;
    Motor_Pitch_PID->out_limit_down = -SOFTSTART_PALLIM_SET;
}

/**
* @brief：用于控制上电时的云台缓动, 缓动结束后调用此函数恢复正常云台参数
* @param [pid_t*]	Motor_Pitch_PID：云台Pitch闭环PID结构体
* @author：ych
*/
void Gimbal_SoftStart_PitchPara_Recover(pid_t *Motor_Pitch_PID)
{
    // 恢复参数
    Motor_Pitch_PID->out_limit_up = Gimbal_Pitch_Para_Rem_str.gyro_Angle_pid_OutPutMaxUp_set;
    Motor_Pitch_PID->out_limit_down = Gimbal_Pitch_Para_Rem_str.gyro_Angle_pid_OutPutMaxDown_set;
}

/**
* @brief：用于控制上电时的云台缓动, 在PID每次运行之前调用此函数即可，此函数在完成缓启动之后会自动恢复PID参数
* @param [pid_t*]	Motor_Pitch_PID：云台Pitch闭环PID结构体
* @author：ych
*/
void Gimbal_SoftStart_Ctrl(pid_t *Motor_Pitch_PID)
{
    static rt_int8_t First_Flag = 1; // 记录是否为第一次运行此函数
    static int8_t Start_Flag = 1;   //记录是否处于上电后的缓动阶段
    static int16_t Start_count = 0; //用于上电缓动过程计时，如果超时则强制结束上电缓动阶段
    if (Start_Flag)
    { //此时仍然处于缓动模式
        if (First_Flag)
        { // 首次运行，将参数修改为启动缓动参数
            Gimbal_SoftStart_PitchPara_Set(Motor_Pitch_PID);
            First_Flag = 0;
        }
        else
        {   // 不是首次运行，此时PID参数应已经被修改为缓启动参数了
            // 根据缓动模式记录标志位判断是否已经可以恢复到正常参数
            Start_count++; //每ms自增1，用于计时
            if (Start_count < 3000 && Start_count > 300)
            { // 如果当前缓动时间没有超时，说明可以继续等待电机角度达到设定值
                if ((Motor_Pitch_PID->err >= 0 && Motor_Pitch_PID->err < 2) || (Motor_Pitch_PID->err < 0 && Motor_Pitch_PID->err > -2))
                {
                    //PID已经基本达到设定值，可以准备停止缓动阶段，恢复正常参数
                    if (Start_count < 2800)
                        Start_count = 2800;
                }
            }
            else if (Start_count > 3000)
            {
                //缓动结束，恢复正常参数
                Gimbal_SoftStart_PitchPara_Recover(Motor_Pitch_PID);
                Start_Flag = 0;
            }
        }
    }
}
