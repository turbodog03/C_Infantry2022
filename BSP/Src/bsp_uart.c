//
// Created by turboDog on 2021/11/16.
//

#include "bsp_uart.h"
#include "stdint.h"
#include "stm32f4xx_hal.h"
#include <string.h>
#include <can.h>
#include <bsp_can.h>
#include <calibrate.h>
#include <Gimbal.h>
#include <stdio.h>
#include <usart.h>
#include "Detect.h"

extern UART_HandleTypeDef huart3;
extern DMA_HandleTypeDef hdma_usart3_rx;
extern global_cali_t glb_cali_data;
extern gimbal_yaw_t gim;
extern float yaw_angle_ref,pit_angle_ref;
extern uint8_t send_flag;
char print_buf[50];

//ext_power_heat_data_t heat_data;


uint8_t		nuc_recv[NUC_FRAME_SIZE];
uint8_t		referee_recv[REFEREE_FRAME_SIZE];
uint8_t		bluetooth_recv[BLUETOOTH_FRAME_SIZE];
//float pc_angle[2]={0.0,0.0};
float amplitude=50.0f;
/**
  * @brief     �����жϻص������������� UART ����ʱע��
  */
extern uint16_t crc16_check(uint8_t* data, uint32_t length);
int PcWatchdog=0;
recv_frame data_recv;
void nuc_uart_callback(void)
{
    if(((uint16_t*)nuc_recv)[0]==0xaaaa)
    {
        PcWatchdog++;
        if(gim.ctrl_mode==GIMBAL_AUTO)
        {
            //PE4��λ
            //GPIOE->BSRR=0x10;
            memcpy((void*)&data_recv,nuc_recv,sizeof(data_recv));
            //PE4��λ
            //GPIOE->BSRR=0x100000;

        }
    }
}

void bluetooth_uart_callback(void)
{
    float value=0.0f;
    char type=0;

    bluetooth_recv[BLUETOOTH_FRAME_SIZE-1]='\n';
    sscanf((const char *)bluetooth_recv,"%c%f",&type,&value);
    switch(type)
    {
////		case 'A':
////			amplitude=value;
//		case 'p':
//			pid_pit.p=value;
//			break;
//		case 'i':
//			pid_pit.i=value;
//			break;
//		case 'd':
//			pid_pit.d=value;
//			break;
//		case 'P':
//			pid_pit_speed.p=value;
//			break;
//		case 'I':
//			pid_pit_speed.i=value;
//			break;
//		case 'D':
//			pid_pit_speed.d=value;
//			break;
//��д��ĸ���ڻ���Сд��ĸ���⻷
    }
    //memcpy(nmb,bluetooth_recv,sizeof(nmb));
    //if (glb_cali_data.gimbal_cali_data.calied_flag != CALIED_FLAG)
    //glb_cali_data.gimbal_cali_data.cali_cmd = 1;
//	sprintf(mmp_buf,"%f\n",pid_yaw_speed.p);
//	write_uart(BLUETOOTH_UART,(uint8_t *)mmp_buf,20);
//	sprintf(mmp_buf,"%f\n",pid_yaw_speed.i);
//	write_uart(BLUETOOTH_UART,(uint8_t *)mmp_buf,20);
//	sprintf(mmp_buf,"%f\n",pid_yaw_speed.d);
//	write_uart(BLUETOOTH_UART,(uint8_t *)mmp_buf,20);

//	if(nmb[0]>0)
//	{
//		pid_yaw.p=nmb[0];
//		pid_yaw.i=nmb[1];
//		pid_yaw.d=nmb[2];
//	}
}
/**
  * @brief          remote control protocol resolution
  * @param[in]      sbus_buf: raw data point
  * @param[out]     rc_ctrl: remote control data struct point
  * @retval         none
  */


//remote control data
/* �������ң�������� */
rc_type_t rc;


//receive data, 18 bytes one frame, but set 36 bytes
//����ԭʼ���ݣ�Ϊ18���ֽڣ�����36���ֽڳ��ȣ���ֹDMA����Խ��
uint8_t sbus_rx_buf[2][SBUS_RX_BUF_NUM];
static void remote_data_handle(rc_type_t *rc, uint8_t *buff);
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
void remote_control_init(void)
{
    RC_init(sbus_rx_buf[0], sbus_rx_buf[1], SBUS_RX_BUF_NUM);
}
/**
  * @brief          get remote control data point
  * @param[in]      none
  * @retval         remote control data point
  */


//�����ж�
void USART3_IRQHandler(void)
{
    if(huart3.Instance->SR & UART_FLAG_RXNE)//���յ�����
    {
        __HAL_UART_CLEAR_PEFLAG(&huart3);
    }
    else if(USART3->SR & UART_FLAG_IDLE)
    {
        static uint16_t this_time_rx_len = 0;

        __HAL_UART_CLEAR_PEFLAG(&huart3);

        if ((hdma_usart3_rx.Instance->CR & DMA_SxCR_CT) == RESET)
        {
            /* Current memory buffer used is Memory 0 */

            //disable DMA
            //ʧЧDMA
            __HAL_DMA_DISABLE(&hdma_usart3_rx);

            //get receive data length, length = set_data_length - remain_length
            //��ȡ�������ݳ���,���� = �趨���� - ʣ�೤��
            this_time_rx_len = SBUS_RX_BUF_NUM - hdma_usart3_rx.Instance->NDTR;

            //reset set_data_lenght
            //�����趨���ݳ���
            hdma_usart3_rx.Instance->NDTR = SBUS_RX_BUF_NUM;

            //set memory buffer 1
            //�趨������1
            hdma_usart3_rx.Instance->CR |= DMA_SxCR_CT;

            //enable DMA
            //ʹ��DMA
            __HAL_DMA_ENABLE(&hdma_usart3_rx);

            if(this_time_rx_len == DBUS_FRAME_SIZE)
            {
                remote_data_handle(&rc, sbus_rx_buf[0]);
//                Send_RC_Data(&hcan2, sbus_rx_buf[0]);
//                Send_RC_Data(&hcan1, sbus_rx_buf[0]);
                HAL_GPIO_WritePin(GPIOH,GPIO_PIN_11,GPIO_PIN_SET);
            }
        }
        else
        {
            /* Current memory buffer used is Memory 1 */
            //disable DMA
            //ʧЧDMA
            __HAL_DMA_DISABLE(&hdma_usart3_rx);

            //get receive data length, length = set_data_length - remain_length
            //��ȡ�������ݳ���,���� = �趨���� - ʣ�೤��
            this_time_rx_len = SBUS_RX_BUF_NUM - hdma_usart3_rx.Instance->NDTR;

            //reset set_data_lenght
            //�����趨���ݳ���
            hdma_usart3_rx.Instance->NDTR = SBUS_RX_BUF_NUM;

            //set memory buffer 0
            //�趨������0
            DMA1_Stream1->CR &= ~(DMA_SxCR_CT);

            //enable DMA
            //ʹ��DMA
            __HAL_DMA_ENABLE(&hdma_usart3_rx);

            if(this_time_rx_len == DBUS_FRAME_SIZE)
            {
                //����ң��������
                remote_data_handle(&rc, sbus_rx_buf[1]);
//                Send_RC_Data(&hcan2, sbus_rx_buf[1]);
//                Send_RC_Data(&hcan1, sbus_rx_buf[1]);
                HAL_GPIO_WritePin(GPIOH,GPIO_PIN_12,GPIO_PIN_SET);

            }
        }
    }
}

/**
  * @brief     ����ң��������
  * @param     rc: �������ң�������ݽṹ��ָ��
  * @param     buff: ���ڽ��յ���ң����ԭʼ����ָ��
  */
static void remote_data_handle(rc_type_t *rc, uint8_t *buff)
{
    /* ����������ң�������ݵĴ��� */
    rc->ch1 = (buff[0] | buff[1] << 8) & 0x07FF;
    rc->ch1 -= 1024;
    rc->ch2 = (buff[1] >> 3 | buff[2] << 5) & 0x07FF;
    rc->ch2 -= 1024;
    rc->ch3 = (buff[2] >> 6 | buff[3] << 2 | buff[4] << 10) & 0x07FF;
    rc->ch3 -= 1024;
    rc->ch4 = (buff[4] >> 1 | buff[5] << 7) & 0x07FF;
    rc->ch4 -= 1024;

    /* ��ֹң���������ƫ�� */
    if(rc->ch1 <= 5 && rc->ch1 >= -5)
        rc->ch1 = 0;
    if(rc->ch2 <= 5 && rc->ch2 >= -5)
        rc->ch2 = 0;
    if(rc->ch3 <= 5 && rc->ch3 >= -5)
        rc->ch3 = 0;
    if(rc->ch4 <= 5 && rc->ch4 >= -5)
        rc->ch4 = 0;

    /* ����ֵ��ȡ */
    rc->sw1 = ((buff[5] >> 4) & 0x000C) >> 2;
    rc->sw2 = (buff[5] >> 4) & 0x0003;

    /* ң�����쳣ֵ��������ֱ�ӷ��� */
    if ((abs(rc->ch1) > 660) || \
      (abs(rc->ch2) > 660) || \
      (abs(rc->ch3) > 660) || \
      (abs(rc->ch4) > 660))
    {
        memset(rc, 0, sizeof(rc_type_t));
        return ;
    }

    /* ����ƶ��ٶȻ�ȡ */
    rc->mouse.x = buff[6] | (buff[7] << 8);
    rc->mouse.y = buff[8] | (buff[9] << 8);

    /* ������Ұ�����ֵ��ȡ */
    rc->mouse.l = buff[12];
    rc->mouse.r = buff[13];

    /* ���̰�����ֵ��ȡ */
    rc->kb.key_code = buff[14] | buff[15] << 8;

    /* ң��������Ϸ��������ݻ�ȡ����ң�����汾�йأ��е��޷��ش��������� */
    /* ��δ�����°� */
//    rc->wheel = buff[16] | buff[17] << 8;
//    rc->wheel -= 1024;
}

void Send_RC_Data(CAN_HandleTypeDef *_hcan, uint8_t *rc_data)
{
    static CAN_TxHeaderTypeDef TX_MSG;
    static uint8_t CAN_Send_Data[8];
    uint32_t send_mail_box;

    TX_MSG.StdId = CAN_RC_DATA_Frame_0;
    TX_MSG.IDE = CAN_ID_STD;
    TX_MSG.RTR = CAN_RTR_DATA;
    TX_MSG.DLC = 0x08;
    TX_MSG.TransmitGlobalTime = DISABLE;
    CAN_Send_Data[0] = rc_data[0];
    CAN_Send_Data[1] = rc_data[1];
    CAN_Send_Data[2] = rc_data[2];
    CAN_Send_Data[3] = rc_data[3];
    CAN_Send_Data[4] = rc_data[4];
    CAN_Send_Data[5] = rc_data[5];
    CAN_Send_Data[6] = rc_data[6];
    CAN_Send_Data[7] = rc_data[7];
    HAL_CAN_AddTxMessage(_hcan, &TX_MSG, CAN_Send_Data, &send_mail_box);

    TX_MSG.StdId = CAN_RC_DATA_Frame_1;
    CAN_Send_Data[0] = rc_data[8];
    CAN_Send_Data[1] = rc_data[9];
    CAN_Send_Data[2] = rc_data[10];
    CAN_Send_Data[3] = rc_data[11];
    CAN_Send_Data[4] = rc_data[12];
    CAN_Send_Data[5] = rc_data[13];
    CAN_Send_Data[6] = rc_data[14];
    CAN_Send_Data[7] = rc_data[15];
    HAL_CAN_AddTxMessage(_hcan, &TX_MSG, CAN_Send_Data, &send_mail_box);
}

void RC_init(uint8_t *rx1_buf, uint8_t *rx2_buf, uint16_t dma_buf_num)
{
    //enable the DMA transfer for the receiver request
    //ʹ��DMA���ڽ���
    SET_BIT(huart3.Instance->CR3, USART_CR3_DMAR);

    //enalbe idle interrupt
    //ʹ�ܿ����ж�
    __HAL_UART_ENABLE_IT(&huart3, UART_IT_IDLE);

    //disable DMA
    //ʧЧDMA
    __HAL_DMA_DISABLE(&hdma_usart3_rx);
    while(hdma_usart3_rx.Instance->CR & DMA_SxCR_EN)
    {
        __HAL_DMA_DISABLE(&hdma_usart3_rx);
    }

    hdma_usart3_rx.Instance->PAR = (uint32_t) & (USART3->DR);
    //memory buffer 1
    //�ڴ滺����1
    hdma_usart3_rx.Instance->M0AR = (uint32_t)(rx1_buf);
    //memory buffer 2
    //�ڴ滺����2
    hdma_usart3_rx.Instance->M1AR = (uint32_t)(rx2_buf);
    //data length
    //���ݳ���
    hdma_usart3_rx.Instance->NDTR = dma_buf_num;
    //enable double memory buffer
    //ʹ��˫������
    SET_BIT(hdma_usart3_rx.Instance->CR, DMA_SxCR_DBM);

    //enable DMA
    //ʹ��DMA
    __HAL_DMA_ENABLE(&hdma_usart3_rx);

}

void write_uart(uint8_t uart_id, uint8_t *send_data, uint16_t size){
    switch(uart_id){
        case 1:
        {
            HAL_UART_Transmit(&huart1,send_data,size,HAL_MAX_DELAY);
            break;
        }
        case 3:
        {
            HAL_UART_Transmit(&huart3,send_data,size,HAL_MAX_DELAY);
            break;
        }
        case 6:
        {
            HAL_UART_Transmit(&huart6,send_data,size,HAL_MAX_DELAY);
            break;
        }
        default:
            break;
    }
}

