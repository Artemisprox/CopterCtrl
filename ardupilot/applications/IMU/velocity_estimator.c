#include "velocity_estimator.h"
#include "drv_utils.h"
#include "roboselect.h"
#include "Kalman.h"
#include "func_sensor.h"
#include "INS_FLOW.h"
#include "math.h"
#include "Filter.h"
#include "drv_dataserve.h"
#include "func_remote.h"

//static float test_integate = 0;
int accl_num ;
float accl_buffer[30];
Kalman_Height_t Height_KF;
Kalman_Forward_v_one_dimension V_height_KF;
filter_test filter_test_data = {0};
vec_fusion ins_data,flow_data;//融合数据源
float weight = 0;//置信权重
pos_sensor ins,test_ins;
filter_data imu_data[3];
struct rt_semaphore sem_ins;
float height = 0, v_height = 0;
variance_data var_height,var_accl;


/**
 * @brief 角度转弧度
 * 
 * @param data_in 输入角度
 * @param data_out 输出角度
 */
void deg_to_rad(IMU_t data_in , IMU_t *data_out )
{
	data_out->pitch = data_in.pitch/360.0f*2.0f*3.1415926f;
	data_out->roll = data_in.roll/360.0f*2.0f*3.1415926f;
	data_out->yaw = data_in.yaw/360.0f*2.0f*3.1415926f;
}

/**
 * @brief 旋转至大地系
 * 
 * @param imu_data_in 当前机体欧拉角
 * @param accl_data 当前机体加速度
 * @param ground_accl 大地系加速度
 */
void rotation_imu_to_ground(IMU_t imu_data_in , float accl_data[3], float *ground_accl)
{
	IMU_t imu_data;
	deg_to_rad(imu_data_in , &imu_data);

	float Accl_R[3][3] = {0};
    Accl_R[0][0] = arm_cos_f32(imu_data.pitch);
    Accl_R[0][1] = arm_sin_f32(imu_data.roll)*arm_sin_f32(imu_data.pitch);
    Accl_R[0][2] = arm_sin_f32(imu_data.pitch)*arm_cos_f32(imu_data.roll);
	Accl_R[1][0] = 0;
    Accl_R[1][1] = arm_cos_f32(imu_data.roll);
    Accl_R[1][2] = -arm_sin_f32(imu_data.roll);
	Accl_R[2][0] = -arm_sin_f32(imu_data.pitch);
    Accl_R[2][1] = arm_sin_f32(imu_data.roll)*arm_cos_f32(imu_data.pitch);
    Accl_R[2][2] = arm_cos_f32(imu_data.roll)*arm_cos_f32(imu_data.pitch);
		
	ground_accl[0] = Accl_R[0][0]*accl_data[0] + Accl_R[0][1]*accl_data[1] + Accl_R[0][2]*accl_data[2];
	ground_accl[1] = Accl_R[1][0]*accl_data[0] + Accl_R[1][1]*accl_data[1] + Accl_R[1][2]*accl_data[2];
	ground_accl[2] = Accl_R[2][0]*accl_data[0] + Accl_R[2][1]*accl_data[1] + Accl_R[2][2]*accl_data[2] - g;
}

/**
 * @brief 滤波集合
 * 
 * @param Fil_Flag 滤波标志位
 * @param data 输入数据结构体
 * @param k1 高通滤波系数
 * @param k2 第一次低通滤波系数
 * @param k3 第二次低通滤波系数
 */
void Filter_Combination(int Fil_Flag, filter_data *data,float k1,float k2,float k3)
{
	if (Fil_Flag)
	{
		data->HP_temp = UTILS_HP_FAST(data->HP_temp, data->Raw_data, data->Last_data, k1);
		data->LP1_temp = UTILS_LP_FAST(data->LP1_temp, data->HP_temp, k2);
		data->LP2_temp = UTILS_LP_FAST( data->LP2_temp ,data->LP1_temp , k3);
	}
	else
	{
		data->LP1_temp = UTILS_LP_FAST(data->LP1_temp, data->Raw_data , k2);
		data->LP2_temp = UTILS_LP_FAST( data->LP2_temp ,data->LP1_temp ,k3);
	}
}

float variance_calculation(variance_data *data, float data_now)
{

	data->sample_sum -= data->sample[data->point];
	data->sample[data->point] = data_now;
	data->sample_sum += data->sample[data->point];

	if(data->first_flag)
	{
		data->num = (data->point+1);
	}

	data->average = data->sample_sum / data->num;
	for(int i = 0; i < data->num;i++)
	{
		data->sample_deviation_sum -= data->sample_deviation[i];
		data->sample_deviation[i] = pow(data->sample[i] - data->average,2);
		data->sample_deviation_sum += data->sample_deviation[i];
	}
	data->variance = data->sample_deviation_sum / data->num;
	
	if(data->point < data->const_num-1)
	{
		data->point++;
	}
	else
	{
		data->first_flag = 0;
		data->point = 0;
	}

	return data->variance;
}

extern remote_data copter_remote;
/**
 * @brief 置信加权计算
 * 
 * @param ins 惯导数据
 * @param flow 光流计数据
 * @return 权重
 */
float Confidence_Weighting(vec_fusion *ins, vec_fusion *flow, int flag)
{
    float  q1, q2, param1, param2;
    float weight;
	
		if(copter_remote.throttle_middle_flag == 1)//高度位置控制，速度方差阈值小，识别凹凸平面更灵敏
		{
			param1 = 0.1f;
			param2 = 0.0017f;			
		}
		else//高度速度控制，有加减速，速度方差的阈值增大，
		{
			param1 = 0.1f;
			param2 = 0.0023f;
		}

    q1= fabs(variance_calculation(&var_accl,ins->accl_delta) / param1);
    q2= fabs(variance_calculation(&var_height,flow->dis_delta) / param2);
		if(flag == DATA_FUSE)
		{			
			if(q2 > 1) //完全不相信光流速度
			{
				weight = 1;
			}
			else if(q2 < 0.1) //考虑静态时光流计数据抖动，降低光流计权重
			{
				weight = sqrt(q2);
			}
			else//正常加减速
			{
				if(q2 >1)	q2 = 1;
				weight = q2;
			}
		}
		else if(flag == SIMPLE_FUSE)
		{
			if (q2 > 1)//完全不相信光流速度
			{
				weight = 0;
			}
			else
				weight = 1;			
		}
//		if(q1>1)	v_height = 0;//降落时触地直接速度给0
			
	return weight;
}
 
float ground_accl[3];
void accl_vec_estimator(IMU_t angle_data , AHRS_Accl_t accl_data ,float datarate)
{
	
	float accl_vec[3] = { accl_data.x , accl_data.y , accl_data.z };
	rotation_imu_to_ground(angle_data , accl_vec , ground_accl);

}

/**
 * @brief 获取平均加速度
 * 
 * @return float 
 */
float get_ground_accl(void)
{
	//由于高度估计更新频率为50Hz，加速度计的更新时间为1khz，使用周期内加速度计平均数据与高度数据进行适配
	int num_temp = accl_num;
	float aver_accl = 0.0f;
	if(accl_num != 0) 
		while(accl_num --)
		{
			aver_accl = accl_buffer[accl_num]/1.0f/num_temp; 
		}
    
	return aver_accl;
}

/**
 * @brief 惯导积分
 * 
 * @param accl 大地系加速度
 * @param ave_accl 
 * @param datarate 积分速率
 */
void Ins_deal(float *accl, float ave_accl,float datarate)
{
    static uint8_t first_flag=1;
    float delta = 1/datarate;
	
//		imu_data[0].Raw_data = accl[0];
//		imu_data[1].Raw_data = accl[1];
		imu_data[2].Raw_data = accl[2];

    if(first_flag)
    {
      first_flag=0;
			ins_data.velocity = 0;
			ins_data.last_velocity = 0;
			ins_data.last_accl = 0;
			
//			imu_data[0].HP_temp = imu_data[0].Raw_data;
//			imu_data[1].HP_temp = imu_data[1].Raw_data;
			imu_data[2].HP_temp = imu_data[2].Raw_data;
					
    }
    else 
    {
//				Filter_Combination(1,&imu_data[0],0.998,0.8,0.8);
//				Filter_Combination(1,&imu_data[1],0.998,0.8,0.8);
				Filter_Combination(1,&imu_data[2],0.996,0.8,0.8);
			
				ins_data.accl = imu_data[2].HP_temp;
				ins_data.accl_delta = ins_data.accl - ins_data.last_accl;
				ins_data.last_accl = ins_data.accl;

				ins_data.velocity += ins_data.accl*delta;
				ins_data.vec_delta = ins_data.velocity - ins_data.last_velocity;
				ins_data.last_velocity = ins_data.velocity;

    }
//					imu_data[0].Last_data = imu_data[0].Raw_data;
//					imu_data[1].Last_data = imu_data[1].Raw_data;
					imu_data[2].Last_data = imu_data[2].Raw_data;
}

void integral_tim_1ms_IRQHandler(void *paramete)
{
	rt_sem_release(&sem_ins);
}

void ins_thread_entry(void *parameter)
{
	static int ver_num = 0;
    while (1)
    {
				rt_sem_take(&sem_ins, RT_WAITING_FOREVER);

			
				accl_buffer[accl_num] = imu_data[2].LP1_temp;
	
			accl_num++;
			ver_num++;
			if( accl_num == 30 )//防止数组越界
			accl_num = 0;	

			if(ver_num%20 == 0)
			{
				Ins_deal(ground_accl,0.0,50);
				height += v_height * 0.02f;
				rt_device_write(serial, 0, ins_flow_data.cha, NUOFDATA*4);
				rt_device_write(serial, 0, just_float_tail, 4);
			}
    }
}

/**
 * @brief 数据融合估计
 * 
 * @param flow_height 光流计高度
 * @param flow_height_v 光流计速度
 * @param height_observe 观测值
 */
void Velocity_estimate(float flow_height , float flow_height_v , float height_observe[2] , int fuse_flag)
{
	static int  first_flag = 1;

	if(first_flag)
	{
		first_flag = 0;
		flow_data.last_distance = 0;
		flow_data.last_velocity = 0;
		height_observe[0] = 0;
	}

		flow_data.distance = flow_height;
		flow_data.dis_delta = flow_data.distance - flow_data.last_distance;
//		flow_height_v = flow_data.dis_delta/0.02f;
		flow_data.last_distance = flow_data.distance;

		flow_data.velocity = flow_height_v;
		flow_data.vec_delta = flow_data.velocity - flow_data.last_velocity;
		flow_data.last_velocity = flow_data.velocity;
		ins_flow_data.flo[9] = flow_data.distance;
		ins_flow_data.flo[4] = flow_data.velocity;
		ins_flow_data.flo[5] = ins_data.velocity;
		weight = Confidence_Weighting(&ins_data,&flow_data , fuse_flag);
		if(fuse_flag == DATA_FUSE)
			v_height = weight*ins_data.velocity + (1 - weight)*flow_data.velocity;
		else if(fuse_flag == SIMPLE_FUSE)
			v_height = weight*flow_data.velocity;
		
		v_height = Kalman_calculate(&V_height_KF, v_height, get_ground_accl());

		height_observe[1] = v_height;
		ins_data.last_velocity = height_observe[1];
		height_observe[0] = height;
	
	// Kalman_height_calculate(&Height_KF,height,height_v,get_ground_accl());
	// height_observe[0] = Height_KF.State_observe[0];//高度估计
	// height_observe[1] = Height_KF.State_observe[1];//速度估计
}

static struct rt_timer integral_tim;
void Ins_Init(void)
{
	
	  rt_sem_init(&sem_ins, "sem_ins", 0, RT_IPC_FLAG_FIFO);
	  rt_thread_t ins_thread = rt_thread_create("ins_thread", ins_thread_entry, RT_NULL, 1024, 23, 1);

    rt_timer_init(&integral_tim, "integral_tim", integral_tim_1ms_IRQHandler, RT_NULL, 1,
                  RT_TIMER_FLAG_PERIODIC | RT_TIMER_FLAG_SOFT_TIMER);

    rt_timer_start(&integral_tim);
		rt_thread_startup(ins_thread);
}

void Velocity_estimator_init(void)
{	
	const float Q = 0.3f, R = 2.0f;
	var_accl.const_num = VARIANCE_NUM;
	var_height.const_num = VARIANCE_NUM;
	var_accl.first_flag = 1;
	var_height.first_flag = 1;
	Kalman_one_dimension_init(&V_height_KF, 0.02f, Q, R);

	// const float Q1 = 0.01f , Q2 = 0.3f , R1 = 0.002f , R2 = 2.0f;
	// Kalman_height_init(&Height_KF , 0.02f , Q1 , Q2 , R1 , R2 );
	// accl_num = 0;
	Ins_Init();
}
