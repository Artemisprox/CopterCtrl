#include "Filter.h"

/*低通滤波*/
int16_t low_pass_filter_i(int16_t data_now , int16_t data_last , float param)
{  
	return data_now*param + data_last*(1-param);
}

float low_pass_filter_f(float data_now , float data_last , float param)
{  
	return data_now*param + data_last*(1-param);
}

//希尔排序
static void ShellSort(float* arr, int n)
{
	int gap = n;
	while (gap>1)
	{
		//每次对gap折半操作
		gap = gap / 2;
		//单趟排序
		for (int i = 0; i < n - gap; ++i)
		{
			int end = i;
			float tem = arr[end + gap];
			while (end >= 0)
			{
				if (tem < arr[end])
				{
					arr[end + gap] = arr[end];
					end -= gap;
				}
				else
				{
					break;
				}
			}
			arr[end + gap] = tem;
		}
	}
}

//中值滤波
float mid_value_filter(mid *in_sample,float data)
{
	static float order[step_size] = {0};
	static float temp=0;
	if(in_sample->num < step_size)
	{
		in_sample->sample[in_sample->num] = data;
		in_sample->num++;
	}
	else
	{
		in_sample->num = 0;
		in_sample->sample[in_sample->num] = data;
	}
	for(int i =0;i<step_size;i++)
		order[i] = in_sample->sample[i];

	ShellSort(order , step_size);
	
	return order[(step_size - 1)/2];
}

////中值滤波
//static mid_value sample[step_size]={0};
//static float mid_filter(float data)
//{
//	static int num = 0;
//	static int i;
//	static int fiirst_flag = 1;
//	if(first_flag)
//	{
//		first_flag = 0;
//		for(int j = 0;j < step_size;j++)
//			sample[i].time = j;
//	}
//	
//	for(i = 0;i<step_size;i++)
//	{
//		if(sample[i].time == num)
//		{
//			sample[i].size = data;
//			break;
//		}
//	}
//	
//	if(num < step_size - 1)
//		num++;
//	else
//		num=0;
//		
//		while(i != (step_size - 1) && sample[i].size > sample[i+1].size)
//		{
//			float temp1 = sample[i].size;
//			sample[i].size = sample[i+1].size;
//			sample[i+1].size = temp1;
//			int temp2 = sample[i].time;
//			sample[i].time = sample[i+1].time;
//			sample[i+1].time = temp2;
//			i++;
//		}
//			
//		while(i != 0 && sample[i].size < sample[i-1].size)
//		{
//			float temp1 = sample[i].size;
//			sample[i].size = sample[i-1].size;
//			sample[i-1].size = temp1;
//			int temp2 = sample[i].time;
//			sample[i].time = sample[i-1].time;
//			sample[i-1].time = temp2;
//			i--;
//		}
//	
//	return sample[(step_size - 1)/2].size;
//}
