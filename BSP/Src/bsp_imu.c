//
// Created by turboDog on 2021/11/27.
//

#include "bsp_imu.h"
#include "Ins.h"
#include "math.h"




void get_imu_data(imu_t *imu_data){
    imu_data->acc_x = bmi088_real_data.accel[0];
    imu_data->acc_y = bmi088_real_data.accel[1];
    imu_data->acc_z = bmi088_real_data.accel[2];
    imu_data->angle_x = INS_angle[0]* 180 / PI;
    imu_data->angle_y = INS_angle[1]* 180 / PI;
    imu_data->angle_z = INS_angle[2]* 180 / PI;
    imu_data->gyro_x = bmi088_real_data.gyro[0];
    imu_data->gyro_y = bmi088_real_data.gyro[1];
    imu_data->gyro_z = bmi088_real_data.gyro[2];
}