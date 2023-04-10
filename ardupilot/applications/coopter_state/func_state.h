#include <rtthread.h>

#define STABILIZATION    1   //����ģʽ
#define HEIGHT           2   //�߶�ģʽ
#define POSITION         3   //λ��ģʽ

#define READY                 0x01   //��
#define ARMED                 0x00   //���
#define FLYING                0x11   //����

#define LAND_DELAG_TIME       2000//�����ж�ʱ�䣬��λms
typedef struct 
{
    uint8_t flight_status;
    uint8_t emergency;
    uint8_t recoil_compensate_enable;
    uint8_t mode;

    uint8_t rc_status;

    uint8_t power_ready;

}status;

typedef struct 
{
    uint8_t pos_valid;
    uint8_t height_valid;
    uint8_t atti_valid;
    uint8_t rc_valid;
    uint8_t battery_OK;
    uint8_t gimbal_OK;

    uint8_t copter_OK;
}data_check;

extern rt_err_t StateDecide_Init(void);
