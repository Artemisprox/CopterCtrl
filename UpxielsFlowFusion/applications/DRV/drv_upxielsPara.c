#include "drv_upxielsPara.h"
#include <rtthread.h>

void UP_Flow_Process(uint8_t *pData,uint8_t rec_length , upxiels_rawdata* flow_data )
{
    //获取数据长度检查
	if( rec_length != 14 )
	{
		return ;
	}

    //包头、包尾校验
    if(( pData[0] != 0xFE ) || ( pData[1] != 0x0A) || ( pData[13] != 0x55 ))
    {
        return ;
    }
	
	//异或校验位校验
	uint8_t xor_check = 0;
	uint16_t i;
	for(i = 2 ; i < 12 ; i++)
	{
		xor_check ^= pData[i];
    }
    if( xor_check != pData[12] )
        return ;

    //解包
    flow_data->flow_x_integral = ( ((int16_t)pData[3] << 8) | (int16_t)pData[2] );//x方向移动像素点数据
    flow_data->flow_y_integral = ( ((int16_t)pData[5] << 8) | (int16_t)pData[4] );//y方向移动像素点数据
    flow_data->integration_timespan = ( ((int16_t)pData[7] << 8) | (int16_t)pData[6] );//数据更新时间，us
		flow_data->quality = pData[10];
    flow_data->x_radians = flow_data->flow_x_integral/10000.0f;
    flow_data->y_radians = flow_data->flow_y_integral/10000.0f;

}
