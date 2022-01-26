//
// Created by turboDog on 2021/11/27.
//

#include "bsp_imu.h"
#include "Ins.h"
#include "math.h"
#include "BMI088driver.h"
#include "user_lib.h"
#include "QuaternionAHRS.h"


void get_imu_data(imu_t *imu_data){

    imu_data->acc_x = BMI088.Accel[0];
    imu_data->acc_y = BMI088.Accel[1];
    imu_data->acc_z = BMI088.Accel[2];
    imu_data->angle_x = -AHRS.Pitch;
    imu_data->angle_y = -AHRS.Roll;
    imu_data->angle_z = -AHRS.Yaw;
    imu_data->gyro_x = BMI088.Gyro[0];
    imu_data->gyro_y = BMI088.Gyro[1];
    imu_data->gyro_z = BMI088.Gyro[2];
}