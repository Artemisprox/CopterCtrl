#include <math.h>
#include "drv_Vector.h"


/**
* @brief 二维矢量加法 
* @brief { a + b = (m,n) + (p,q) = (m+p,n+q) }
* @param v_a 矢量 a
* @param v_b 矢量 b 
* @return 相加后的矢量
**/
Vector2_t Vector2_Add(Vector2_t v_a,Vector2_t v_b)
{
    Vector2_t temp = 
    {
        .x = v_a.x + v_b.x,
        .y = v_a.y + v_b.y,
    };
    return temp;
}

/**
* @brief 二维矢量标量乘法 
* @brief { a * g = (m,n) * g = (m*g,n*g) }
* @param v_a 矢量 a
* @param gain 矢量大小增益
* @return 增益后的矢量
**/
Vector2_t Vector2_Gain(Vector2_t v_a,float gain)
{
    Vector2_t temp = 
    {
        .x = v_a.x * gain,
        .y = v_a.y * gain,
    };
    return temp;
}

/**
* @brief 矢量加法 
* @brief { a + b = (l,m,n) + (o,p,q) = (l+0,m+p,n+q) }
* @param v_a 矢量 a
* @param v_b 矢量 b 
* @return 相加后的矢量
**/
Vector3_t Vector3_Add(Vector3_t v_a,Vector3_t v_b)
{
    Vector3_t temp = 
    {
        .x = v_a.x + v_b.x,
        .y = v_a.y + v_b.y,
        .z = v_a.z + v_b.z //不要注释掉，不然返回的Vector3_t的z成员值是未知的，容易出现nan bug
    };
    return temp;
}

/**
* @brief 矢量叉乘 
* @brief  { a X b = (l,m,n) X (o,p,q) = (mq-np,no-lq,lp-mo) }
* @param v_a 矢量 a
* @param v_b 矢量 b 
* @return 叉乘后的矢量
**/
Vector3_t Vector3_X(Vector3_t v_a,Vector3_t v_b)
{
    Vector3_t temp = 
    {
        .x = v_a.y * v_b.z - v_a.z * v_b.y,
        .y = v_a.z * v_b.x - v_a.x * v_b.z,
        .z = v_a.x * v_b.y - v_a.y * v_b.x //不要注释掉，不然返回的Vector3_t的z成员值是未知的，容易出现nan bug
    };
    return temp;
}

/**
* @brief  获得矢量大小 
* @param  temp 待处理矢量
* @return None
**/
void Get_Vector3_A(Vector3_t *temp)
{
    temp->A = sqrt(temp->x*temp->x + temp->y*temp->y + temp->z*temp->z);
}

/**
* @brief  转向角解算
*          返回的转向角范围为(-180°,180°]，0°为车体坐标系的y轴，角度值增大方向为z轴旋转正方向
* @param  temp 待处理矢量
* @return None
**/
void Get_Vector3_theta(Vector3_t *temp)
{
    //反tan函数，范围(-180°,180°]
    temp->xy_theta = atan2f( -temp->x, temp->y)/V_PI*180.0f;
}

/**
* @brief  获得矢量大小和转向角
* @param  temp 待处理矢量
* @return None
**/
void Get_Vector3_A_theta(Vector3_t *temp)
{
    /*合成速度大小计算*/
    Get_Vector3_A(temp);
    /*转向角计算*/
    Get_Vector3_theta(temp);
}

/**
* @brief  通过大小和转向角获得矢量,角度单位为度
* @param  temp 待处理矢量
* @return None
**/
void Get_Vector3_From_A_theta(Vector3_t *temp)
{
    temp->y = temp->A * cosf(temp->xy_theta /180.0f *V_PI);
    temp->x = - temp->A * sinf(temp->xy_theta /180.0f *V_PI);
}

/**
* @brief  通过二维矢量获得三维矢量
* @param  temp2 二维矢量
* @param  gain  增益
* @return None
**/
Vector3_t Get_Vector_3From2(Vector2_t temp2,float gain)
{
    Vector3_t temp3 = 
    {
        .x = temp2.x * gain,
        .y = temp2.y * gain,
        .z = 0,.A = 0,.xy_theta = 0
    };
    return temp3;
}

/**
* @brief  通过三维矢量获得二维矢量
* @param  temp3 三维矢量
* @param  gain  增益
* @return None
**/
Vector2_t Get_Vector_2From3(Vector3_t temp3,float gain)
{
    Vector2_t temp2 = 
    {
        .x = temp3.x * gain,
        .y = temp3.y * gain
    };
    return temp2;
}

/**
* @brief  初始化矢量结构体(全部赋0)
* @param  temp 待处理矢量
* @return None
**/
void Vector3_Init(Vector3_t *temp)
{
    temp->x = 0;
    temp->y = 0;
    temp->z = 0;
    temp->A = 0;
    temp->xy_theta = 0;
}
