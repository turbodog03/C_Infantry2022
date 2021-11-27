//
// Created by turboDog on 2021/11/27.
//

#include "bsp_imu.h"
#include "Ins.h"

void get_imu_data(imu_t *imu_data){
    const fp32 *pAngle = get_INS_angle_point();
    const fp32 *pGyro = get_gyro_data_point();
    const fp32 *pAcc = get_accel_data_point();
    imu_data->acc_x = pAcc[0];
    imu_data->acc_y = pAcc[1];
    imu_data->acc_z = pAcc[2];
    imu_data->angle_x = pAngle[0];
    imu_data->angle_y = pAngle[1];
    imu_data->angle_z = pAngle[2];
    imu_data->gyro_x = pGyro[0];
    imu_data->gyro_y = pGyro[1];
    imu_data->gyro_z = pGyro[2];
}