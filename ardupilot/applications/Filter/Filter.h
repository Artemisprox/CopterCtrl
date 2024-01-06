#ifndef __FILTER_H__
#define __FILTER_H__

#include <rtthread.h>
#include <rtdevice.h>
#include <board.h>

#define step_size 5

typedef struct
{
	int time;
	float size;
}mid_value;

typedef struct 
{
	float sample[step_size];
	int num;
}mid;


extern int16_t low_pass_filter_i(int16_t data_now , int16_t data_last , float param);
extern float low_pass_filter_f(float data_now , float data_last , float param);
extern float mid_value_filter(mid *in_sample,float data);

#endif