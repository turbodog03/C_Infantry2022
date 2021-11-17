//
// Created by turboDog on 2021/11/16.
//

#ifndef BOARD_C_INFANTRY_BSP_UART_H
#define BOARD_C_INFANTRY_BSP_UART_H
#include "Transmission.h"
#include "sys.h"


/* �������ң�������� */
extern rc_type_t rc;

#define SBUS_RX_BUF_NUM 36u

/**
  * @brief          remote control init
  * @param[in]      none
  * @retval         none
  */
/**
  * @brief          ң������ʼ��
  * @param[in]      none
  * @retval         none
  */
extern void remote_control_init(void);
extern void RC_init(uint8_t *rx1_buf, uint8_t *rx2_buf, uint16_t dma_buf_num);


/**
  * @brief     ����ң��������
  * @param     rc: �������ң�������ݽṹ��ָ��
  * @param     buff: ���ڽ��յ���ң����ԭʼ����ָ��
  */
static void remote_data_handle(rc_type_t *rc, uint8_t *buff);
/**
  * @brief     ����ң�������ݸ��ϰ�
  * @param     _hcan: ��Ҫ���͵�can
  * @param     rc_data: ���ڽ��յ���ң����ԭʼ����ָ��
  */
void Send_RC_Data(CAN_HandleTypeDef *_hcan, uint8_t *rc_data);
#endif //BOARD_C_INFANTRY_BSP_UART_H