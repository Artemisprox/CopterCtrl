#include "drv_CustCtrler_Data.h"
#include "drv_dataserve.h"
#include <string.h>

static void *CustCtrler_Data; // 承接数据服务器的中间变量

/*序列号*/
/*线当前伸出的长度*/
static rt_int8_t L_a_num = 0;
static rt_int8_t L_b_num = 0;
static rt_int8_t L_c_num = 0;

/**
 * @brief 自定义控制器数据服务器初始化函数
 * @brief 自动初始化
 */
static int CustCtrler_Data_Init(void)
{
    /*线当前伸出的长度 单位；mm*/
    Request_Add_Package("L_a", strlen("L_a"),
                        &CustCtrler_Data, sizeof(CustCtrler_Data));
    Request_Add_Package("L_b", strlen("L_b"),
                        &CustCtrler_Data, sizeof(CustCtrler_Data));
    Request_Add_Package("L_c", strlen("L_c"),
                        &CustCtrler_Data, sizeof(CustCtrler_Data));
    return 0;
}
DATASERVER_DATAPACKAGE_INIT(CustCtrler_Data_Init);

/**
 * @brief   从数据服务器读取三边长度
 * @param   Out
 */
void CustCtrler_Data_L_Read(VectorXYZ_float_Str *Out)
{
    float *address;

    address = Package_Pionter_Single(L_a_num, float);
    Out->x = *address;
    Package_Write_Pionter_End(L_a_num, float);

    address = Package_Pionter_Single(L_b_num, float);
    Out->y = *address;
    Package_Write_Pionter_End(L_b_num, float);

    address = Package_Pionter_Single(L_c_num, float);
    Out->z = *address;
    Package_Write_Pionter_End(L_c_num, float);
}

/**
 * @brief
 * @param data 自定义控制器数据
 * @param encoder_num
 */
void CustCtrler_Data_L_Write_Single(float data,
                                    Encoder_Num encoder_num)
{
    float *address;
    switch (encoder_num)
    {
    case encoder_1:
        address = Package_Pionter_Single(L_a_num, float);
        *address = data;
        Package_Write_Pionter_End(L_a_num, float);
        break;
    case encoder_2:
        address = Package_Pionter_Single(L_b_num, float);
        *address = data;
        Package_Write_Pionter_End(L_b_num, float);
        break;
    case encoder_3:
        address = Package_Pionter_Single(L_c_num, float);
        *address = data;
        Package_Write_Pionter_End(L_c_num, float);
        break;
    default:
        break;
    }
}

/**
 * @brief  自定义控制器数据包序列号读取
 */
void CustCtrler_Datanum_Find(void)
{
    L_a_num = Package_Find_Num("L_a");
    L_b_num = Package_Find_Num("L_b");
    L_c_num = Package_Find_Num("L_c");
}
