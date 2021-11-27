//
// Created by turboDog on 2021/11/27.
//

#ifndef BOARD_C_INFANTRY_BSP_IMU_H
#define BOARD_C_INFANTRY_BSP_IMU_H
/**
  * @brief     IMU ���ݽṹ��
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
  * @brief     ��ȡ IMU ����
  * @param     imu_data: ���� IMU ���ݵĽṹ��ָ��
  * @usage     ��Ҫ��ѭ�������е��ã�����ˢ�� IMU ����
  */
void get_imu_data(imu_t *imu_data);
#endif //BOARD_C_INFANTRY_BSP_IMU_H
