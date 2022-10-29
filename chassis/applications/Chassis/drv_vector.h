#ifndef __DRV_Vector3_H_
#define __DRV_Vector3_H_

#define V_PI 3.1415926f


/*二维矢量*/
typedef struct 
{
	float x;
    float y;

} Vector2_t;

/*三维矢量*/
typedef struct 
{
    float x;
    float y;
    float z;
    float A;        //矢量大小
    float xy_theta; //二维矢量偏角,范围为(-180°,180°]，0°为坐标系的y轴，角度值增大方向为z轴旋转正方向

} Vector3_t;

/* fcuri */

/**
* @brief  二维矢量比较
* @param  v1/2 矢量1/2
* @return 当v1 == v2时,返回1;否则返回0
**/
#define VECTOR2_CMP(v1,v2)    (v1.x == v2.x && v1.y == v2.y)


/**
* @brief 二维矢量加法 
* @brief { a + b = (m,n) + (p,q) = (m+p,n+q) }
* @param v_a 矢量 a
* @param v_b 矢量 b 
* @return 相加后的矢量
**/
Vector2_t Vector2_Add(Vector2_t v_a,Vector2_t v_b);
/**
* @brief 二维矢量标量乘法 
* @brief { a * g = (m,n) * g = (m*g,n*g) }
* @param v_a 矢量 a
* @param gain 矢量大小增益
* @return 增益后的矢量
**/
Vector2_t Vector2_Gain(Vector2_t v_a,float gain);
/**
* @brief 矢量加法  { a + b = (l,m,n) + (o,p,q) = (l+0,m+p,n+q) }
* @param V_a 矢量 a
* @param V_b 矢量 b 
* @return 相加后的矢量
**/
Vector3_t Vector3_Add(Vector3_t V_a,Vector3_t V_b);
/**
* @brief 矢量叉乘  { a X b = (l,m,n) X (o,p,q) = (mq-np,no-lq,lp-mo) }
* @param V_a 矢量 a
* @param V_b 矢量 b 
* @return 叉乘后的矢量
**/
Vector3_t Vector3_X(Vector3_t V_a,Vector3_t V_b);
/**
* @brief  获得矢量大小 
* @param  temp 待处理矢量
* @return None
**/
void Get_Vector3_A(Vector3_t *temp);
/**
* @brief  转向角解算
*          返回的转向角范围为(-180°,180°]，0°为车体坐标系的y轴，角度值增大方向为z轴旋转正方向
* @param  temp 待处理矢量
* @return None
**/
void Get_Vector3_theta(Vector3_t *temp);
/**
* @brief  获得矢量大小和转向角
* @param  temp 待处理矢量
* @return None
**/
void Get_Vector3_A_theta(Vector3_t *temp);
/**
* @brief  通过大小和转向角获得矢量
* @param  temp 待处理矢量
* @return None
**/
void Get_Vector3_From_A_theta(Vector3_t *temp);
/**
* @brief  通过二维矢量获得三维矢量
* @param  temp2 二维矢量
* @param  gain  增益
* @return None
**/
Vector3_t Get_Vector_3From2(Vector2_t temp2,float gain);
/**
* @brief  通过三维矢量获得二维矢量
* @param  temp3 三维矢量
* @param  gain  增益
* @return None
**/
Vector2_t Get_Vector_2From3(Vector3_t temp3,float gain);
/**
* @brief  初始化矢量结构体(全部赋0)
* @param  temp 待处理矢量
* @return None
**/
void Vector3_Init(Vector3_t *temp);

#endif
