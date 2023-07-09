#include "velocity_estimator.h"
#include "drv_utils.h"
#include "roboselect.h"
#include "Kalman.h"
#include "func_sensor.h"

static float test_integate = 0;
static int accl_num ;
static float accl_buffer[30];
Kalman_Height_t Height_KF;
filter_data accl_raw_data , accl_process_data;  
filter_test filter_test_data = {0};
float v_height = 0;

void deg_to_rad(IMU_t data_in , IMU_t *data_out )
{
	data_out->pitch = data_in.pitch/360.0f*2.0f*3.1415926f;
	data_out->roll = data_in.roll/360.0f*2.0f*3.1415926f;
	data_out->yaw = data_in.yaw/360.0f*2.0f*3.1415926f;
}

//旋转至大地系
void rotation_imu_to_ground(IMU_t imu_data_in , float accl_data[3], float ground_accl[3])
{
	IMU_t imu_data	;
	deg_to_rad(imu_data_in , &imu_data);
/*
    float R_gimbalyaw[3][3] = {{arm_cos_f32(imu_data.yaw), -arm_sin_f32(imu_data.yaw), 0},
						   {arm_sin_f32(imu_data.yaw), arm_cos_f32(imu_data.yaw), 0},
						   {0, 0, 1}};
    float R_gimbalpitch[3][3] = {{arm_cos_f32(imu_data.roll), 0, -arm_sin_f32(imu_data.roll)},
							{0, 1, 0},
							{arm_sin_f32(imu_data.roll), 0, arm_cos_f32(imu_data.roll)}};
    float R_gimbalroll[3][3] = {{1, 0, 0},
							 {0, arm_cos_f32(imu_data.pitch), -arm_sin_f32(imu_data.pitch)},
							 {0, arm_sin_f32(imu_data.pitch), arm_cos_f32(imu_data.pitch)}};

    float R_temp1[3][3] , R[3][3];
    utils_get_matrix_mutiply(&R_gimbalroll, &R_gimbalpitch, &R_temp1, 3, 3, 3, 3);
    utils_get_matrix_mutiply(&R_temp1, &R_gimbalyaw, &R, 3, 3, 3, 3);

    float imu_accl[3][1] = {accl_data[0] , accl_data[1] , accl_data[2]} , accl_temp[3][1];
    utils_get_matrix_mutiply(&R, &imu_accl, &accl_temp, 3, 3, 3, 1);
	ground_accl[2] = accl_temp[2][0];
*/
	float Accl_R[3][3] = {0};
    Accl_R[0][0] = arm_cos_f32(imu_data.pitch);
    Accl_R[0][1] = arm_sin_f32(imu_data.roll)*arm_sin_f32(imu_data.pitch);
    Accl_R[0][2] = arm_sin_f32(imu_data.pitch)*arm_cos_f32(imu_data.roll);
    Accl_R[1][1] = arm_cos_f32(imu_data.roll);
    Accl_R[1][2] = -arm_sin_f32(imu_data.roll);
		Accl_R[2][0] = -arm_sin_f32(imu_data.pitch);
    Accl_R[2][1] = arm_sin_f32(imu_data.roll)*arm_cos_f32(imu_data.pitch);
    Accl_R[2][2] = arm_cos_f32(imu_data.roll)*arm_cos_f32(imu_data.pitch);
		
	ground_accl[2] = Accl_R[2][0]*accl_data[0] + Accl_R[2][1]*accl_data[1] + Accl_R[2][2]*accl_data[2];

}

void Filter_Combination(int Fil_Flag, filter_data *data)
{
	if (Fil_Flag)
	{
		data->HP_temp = UTILS_HP_FAST(data->HP_temp, data->Raw_data, data->Last_data, 0.9985f);
		data->LP1_temp = UTILS_LP_FAST(data->LP1_temp, data->HP_temp, 0.65f);
		data->LP2_temp = UTILS_LP_FAST( data->LP2_temp ,data->LP1_temp , 0.0125f);
	}
	else
	{
		data->LP1_temp = UTILS_LP_FAST(data->LP1_temp, data->Raw_data , 0.65f);
		data->LP2_temp = UTILS_LP_FAST( data->LP2_temp ,data->LP1_temp , 0.0125f);
	}
}
/*
float Confidence_Weighting(float accl_raw, float accl_fil)
{
    float  p1, p2, q, q1, q2 , q1q2;
    float weight;
		  
    q = 5.0f;
	
    q1= fabsf( accl_raw / q) ;
    q2= fabs( accl_fil / q );

    if(q1 > 1) p1 = 0; else p1 = 1;
    if(q2 > 1) p2 = 0; else p2 = 1;
    
    q1q2 = arm_sqrt_f32(q1 * q2, &q1q2);
		
    if (p1 && p2)
    {
       	weight = q1q2;
        return weight;
    }
    else
        return 0;
}
*/
void accl_vec_estimator( IMU_t angle_data , AHRS_Accl_t accl_data )
{
	float ground_accl[3];
	static int  first_flag = 1;
	
	float accl_vec[3] = { accl_data.x , accl_data.y , accl_data.z };
	rotation_imu_to_ground(angle_data , accl_vec , ground_accl);

	accl_raw_data.Raw_data = ground_accl[2] - g;
	accl_process_data.Raw_data = ground_accl[2] - g;
	
	if(first_flag)
	{
		first_flag = 0;
		accl_raw_data.Last_data = accl_raw_data.Raw_data;
		accl_process_data.Last_data = accl_raw_data.Raw_data;
	}	
	else
	{
		Filter_Combination(1 , &accl_process_data);
		test_integate += accl_process_data.HP_temp*0.001f;
		//filter_test_data.v_raw += accl_raw_data.Raw_data * 0.001f;
		//filter_test_data.v_hp += accl_process_data.HP_temp * 0.001f;
		//filter_test_data.v_hp_lp += accl_process_data.LP1_temp * 0.001f;
		//v_height += Confidence_Weighting(accl_raw_data.LP2_temp , accl_process_data.LP2_temp)*0.001f;
		//v_height += accl_raw_data.LP2_temp*0.001f;
	}
	accl_buffer[accl_num] = accl_process_data.HP_temp;
	accl_num++;

	if( accl_num == 30 )//防止数组越界
		accl_num = 0;

	accl_raw_data.Last_data = accl_raw_data.Raw_data;
	accl_process_data.Last_data = accl_raw_data.Raw_data;
}

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

void Velocity_estimate(float height , float height_v , float height_observe[2] )
{
	Kalman_height_calculate(&Height_KF,height,height_v,get_ground_accl());
	height_observe[0] = Height_KF.State_observe[0];//高度估计
	height_observe[1] = Height_KF.State_observe[1];//速度估计
}

void Velocity_estimator_init(void)
{	
	const float Q1 = 0.01f , Q2 = 0.3f , R1 = 0.002f , R2 = 2.0f;
	Kalman_height_init(&Height_KF , 0.02f , Q1 , Q2 , R1 , R2 );
	accl_num = 0;
}
