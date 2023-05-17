#include "func_FlowFusion.h"
#include "func_IMUCOM.h"
#include <stdlib.h>
#include <math.h>

static double variance(float data1 , float data2)
{
    return (data1 - data2)*(data1 - data2);
}

void FlowDataFusion(upxiels_rawdata *data1 , upxiels_rawdata *data2 , upxiels_rawdata *data_out)
{
    /*计算平均值*/
    rt_int16_t average_x , average_y;
    average_x = (data1->flow_x_integral + data2->flow_x_integral)/2;
    average_y = (data1->flow_y_integral + data2->flow_y_integral)/2;

    /*无人机未旋转且两个光流数据差异大时，认为一个光流数据有问题，拒绝相信*/
    rt_int8_t IMU_flag = 0, Flow_flag = 0;
    
		/*陀螺仪判断部分*/
	if( fabsf(IMU_RawData.YawSpe) > IMU_THRESHOLD )//正在进行yaw轴方向机动，光流计数据偏差是正常的
		IMU_flag = 1;
	else IMU_flag = 0;
		
    /*光流数据比较*/
    if( ( variance(data1->x_radians , data2->x_radians) + variance(data1->y_radians , data2->y_radians))  > FLOW_THRESHOLD)//在正常悬停下出现光流计数据失真
        Flow_flag = 1;
    else Flow_flag = 0;
		
		if ((data1->quality == 245) && (data2->quality == 245))
		{
			if((Flow_flag == 1) && (IMU_flag == 0))//光流计数据失真
			{
				/*在失真情况下，认为移动过大的情况是地面光线变化导致的，选用位移较小的传感器*/
					data_out->flow_x_integral = (abs((int)data1->flow_x_integral) > abs((int)data2->flow_x_integral)) ? data1->flow_x_integral : data2->flow_x_integral;
					data_out->flow_y_integral = (abs((int)data1->flow_y_integral) > abs((int)data2->flow_y_integral)) ? data1->flow_y_integral : data2->flow_y_integral;
			}else
			{
				/*正常情况下使用均值作为数据输出*/
					data_out->flow_x_integral = average_x;
					data_out->flow_y_integral = average_y;
			}
			data_out->quality = 245;
		}else if(data1->quality == 245)
		{
			data_out->flow_x_integral = data1->flow_x_integral;
			data_out->flow_y_integral = data2->flow_y_integral;
			data_out->quality = 245;
		}else if(data2->quality == 245)
		{
			data_out->flow_x_integral = data2->flow_x_integral;
			data_out->flow_y_integral = data2->flow_y_integral;
			data_out->quality = 245;
		}else
		{
			data_out->quality = 0;
		}
		
}
