//
// Created by turboDog on 2021/11/17.
//

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




#include <bsp_can.h>
#include <bsp_flash.h>
#include "calibrate.h"
/* ȫ��У׼���ݽṹ�� */
global_cali_t glb_cali_data;

/**
  * @brief     ����У׼���ݵ��û� flash ��
  */
static void save_cali_data(void)
{
    write_flash((uint8_t*)&glb_cali_data, sizeof(global_cali_t));
}
/**
  * @brief     ���û� flash �ж�ȡУ׼����
  */
void read_cali_data(void)
{
//  read_flash((uint8_t*)&glb_cali_data, sizeof(global_cali_t));
    glb_cali_data.gimbal_cali_data.pit_offset=0x1BD0;
    glb_cali_data.gimbal_cali_data.yaw_offset=0x1730;
    glb_cali_data.gimbal_cali_data.calied_flag=CALIED_FLAG;
}

/**
  * @brief     ��̨У׼���ݱ��溯��
  */
void gimbal_cali_hook(void)
{
    if (glb_cali_data.gimbal_cali_data.cali_cmd == 1)
    {
        glb_cali_data.gimbal_cali_data.pit_offset = PitMotor.RawAngle;
        glb_cali_data.gimbal_cali_data.yaw_offset = YawMotor.RawAngle;
        glb_cali_data.gimbal_cali_data.calied_flag = CALIED_FLAG;
        glb_cali_data.gimbal_cali_data.cali_cmd  = 0;
        save_cali_data();
    }
}
