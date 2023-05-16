#ifndef _DRV_CUSTCTRLER_DATA_H_
#define _DRV_CUSTCTRLER_DATA_H_
#include <rtthread.h>
typedef struct
{
    float x;
    float y;
    float z;
} VectorXYZ_float_Str;

/**
 * @brief   从数据服务器读取三边长度
 * @param   Out
 */
void CustCtrler_Data_L_Read(VectorXYZ_float_Str *Out);

/**
 * @brief
 * @param data 自定义控制器数据
 * @param encoder_num
 */
void CustCtrler_Data_L_Write_Single(float data,
                                    Encoder_Num encoder_num);

/**
 * @brief  自定义控制器数据包序列号读取
 */
void CustCtrler_Datanum_Find(void);

#endif //_DRV_CUSTCTRLER_DATA_H_
