//
// Created by turboDog on 2021/11/27.
//

#ifndef BOARD_C_INFANTRY_BSP_IMU_H
#define BOARD_C_INFANTRY_BSP_IMU_H
/**
  * @brief     IMU 数据结构体
  */
typedef struct
{
    float acc_x;   //m/s^2
    float acc_y;   //m/s^2
    float acc_z;   //m/s^2
    float gyro_x;  //degree/s
    float gyro_y;  //degree/s
    float gyro_z;  //degree/s
    float angle_x; //degree
    float angle_y; //degree
    float angle_z; //degree
} imu_t;

/**
  * @brief     读取 IMU 数据
  * @param     imu_data: 接收 IMU 数据的结构体指针
  * @usage     需要在循环任务中调用，用来刷新 IMU 数据
  */
void get_imu_data(imu_t *imu_data);
#endif //BOARD_C_INFANTRY_BSP_IMU_H
