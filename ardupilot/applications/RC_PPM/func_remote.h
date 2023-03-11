#include <rtthread.h>

#define f_max            30//�������
#define pitch_max_degree 10//���Ƕ�
#define roll_max_degree  10//���Ƕ�
#define yaw_max_spe      30//�����ת�ٶ�

#define rocker_min       36.0f  //��Сռ�ձ�
#define rocker_max       72.0f  //���ռ�ձ�
#define rocker_middle    50.0f
#define rocker_inter     2.0f		//����ͬλ����ҡ�˵��ź����

#define stabilization    1   //����ģʽ
#define height           2   //����ģʽ
#define position         3   //����ģʽ

#define ready                 1   //׼��
#define armed                 0   //���
#define emergency_stop        2   //����ֹͣ

#define LPF_k                0.9

typedef struct
{
	float f;
	float pitch_deg;
	float roll_deg;
	float yaw_spe;

	uint8_t pitch_middle_flag;
	uint8_t roll_middle_flag;
	uint8_t yaw_middle_flag;
	uint8_t throttle_low_flag;

	uint8_t switch_arm;
	uint8_t switch_mode;
	uint8_t switch_arm_change;
	uint8_t switch_mode_change;

}remote_data;
