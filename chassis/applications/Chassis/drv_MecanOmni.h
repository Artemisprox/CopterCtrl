#ifndef __DRV_MECANOMNI_H__
#define __DRV_MECANOMNI_H__
#include <rtthread.h>
#include "drv_vector.h"


typedef struct
{
    Vector2_t   vel;        //平移速度矢量，单位mm/s
    float       angvel;     //角速度，单位0.1°/s

} MecanOmni_t;


/**
 * @brief   麦轮/全向底盘运动解算,并输出到wheel层
 * @param   chs.vel     底盘xy方向速度大小,单位mm/s
 * @param   chs.angvel  自转角速度,单位0.1°/s,底盘逆时针旋转为正
 * @param   output      输出的底盘每个电机的转速设定值
 * @return  None
 * @author  lfp
 */
extern void MecanOmni_Resolve(MecanOmni_t chs, float output[]);


#endif
