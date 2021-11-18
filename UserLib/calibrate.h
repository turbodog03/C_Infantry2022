//
// Created by turboDog on 2021/11/17.
//

#ifndef BOARD_C_INFANTRY_CALIBRATE_H
#define BOARD_C_INFANTRY_CALIBRATE_H
/****************************************************************************
 *  Copyright (C) 2018 RoboMaster.
 *
 *  This program is free software: you can redistribute it and/or modify
 *  it under the terms of the GNU General Public License as published by
 *  the Free Software Foundation, either version 3 of the License, or
 *  (at your option) any later version.
 *
 *  This program is distributed in the hope that it will be useful,
 *  but WITHOUT ANY WARRANTY; without even the implied warranty of?
 *  MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.? See the
 *  GNU General Public License for more details.
 *
 *  You should have received a copy of the GNU General Public License
 *  along with this program. If not, see <http://www.gnu.org/licenses/>.
 ***************************************************************************/

/**
  *********************** (C) COPYRIGHT 2018 DJI **********************
  * @update
  * @history
  * Version     Date              Author           Modification
  * V1.0.0      January-15-2018   ric.luo
  * @verbatim
  *********************** (C) COPYRIGHT 2018 DJI **********************
  */



#include "stm32f4xx_hal.h"

#define CALIED_FLAG 0x55

/**
  * @brief     ��̨У׼���ݽṹ��
  */
typedef __packed struct
{
    int16_t yaw_offset;
    int16_t pit_offset;
    uint8_t cali_cmd;
    uint8_t calied_flag;
} gimbal_cali_t;
/**
  * @brief     ȫ��У׼���ݽṹ��
  */
typedef __packed struct
{
    uint8_t       saved_flag;       //У׼���ݱ����־
    uint32_t      firmware_version; //�̼��汾��
    gimbal_cali_t gimbal_cali_data; //��̨У׼����
} global_cali_t;

/**
  * @brief     ���û� flash �ж�ȡУ׼����
  */
void read_cali_data(void);
/**
  * @brief     ��̨У׼���ݱ��溯��
  */
void gimbal_cali_hook(void);

/* ȫ��У׼���ݽṹ�� */
extern global_cali_t glb_cali_data;

#endif //BOARD_C_INFANTRY_CALIBRATE_H
