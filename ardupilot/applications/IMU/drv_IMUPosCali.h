
#define IMUDataColletNum 500

typedef struct
{
    float Pitch;
    float Roll;
}IMU_InstallPos;

extern IMU_InstallPos IMU1_pos;
extern void IMU_PosCali_Init(void);
