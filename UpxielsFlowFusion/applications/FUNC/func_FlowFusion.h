#include <rtthread.h>
#include "drv_upxielsPara.h"

#define FLOW_THRESHOLD 500
#define IMU_THRESHOLD  15.0f

extern void FlowDataFusion(upxiels_rawdata *data1 , upxiels_rawdata *data2 , upxiels_rawdata *data_out);
