//
// Created by HLiamso on 2021-11-14.
//

#include <can.h>
#include <Detect.h>
#include "bsp_can.h"
#include "Transmission.h"
#define CAN_DOWN_TX_INFO 0x133
chassis_mode_e chassis_mode;
/* 云台电机 */
Motor_t PitMotor;
Motor_t YawMotor;
/* 拨弹电机 */
moto_measure_t moto_trigger;
/* 底盘电机 */
Motor_t ChassisMotor[4];
/* 3508摩擦轮电机 */
moto_measure_t moto_shoot[2];//0左，1右；

void CAN_Device_Init(void){
    CAN_FilterTypeDef can_filter_st;
    can_filter_st.FilterActivation = ENABLE;
    can_filter_st.FilterMode = CAN_FILTERMODE_IDMASK;
    can_filter_st.FilterScale = CAN_FILTERSCALE_32BIT;
    can_filter_st.FilterIdHigh = 0x0000;
    can_filter_st.FilterIdLow = 0x0000;
    can_filter_st.FilterMaskIdHigh = 0x0000;
    can_filter_st.FilterMaskIdLow = 0x0000;
    can_filter_st.FilterBank = 0;
    can_filter_st.FilterFIFOAssignment = CAN_RX_FIFO0;
    // CAN1 过滤器初始化
    while (HAL_CAN_ConfigFilter(&hcan1, &can_filter_st) != HAL_OK)
    {
    }
    // 启动CAN1
    while (HAL_CAN_Start(&hcan1) != HAL_OK)
    {
    }
    // 启动通知
    while (HAL_CAN_ActivateNotification(&hcan1, CAN_IT_RX_FIFO0_MSG_PENDING) != HAL_OK)
    {

    }

    can_filter_st.SlaveStartFilterBank = 14;
    can_filter_st.FilterBank = 14;
    // CAN2 过滤器初始化
    while (HAL_CAN_ConfigFilter(&hcan2, &can_filter_st) != HAL_OK)
    {
    }
    // 启动CAN2
    while (HAL_CAN_Start(&hcan2) != HAL_OK)
    {
    }
    // 启动通知
    while (HAL_CAN_ActivateNotification(&hcan2, CAN_IT_RX_FIFO0_MSG_PENDING) != HAL_OK)
    {
    }
}
void send_chassis_moto_zero_current(void)
{
//    static uint8_t data[8];
//
//    data[0] = 0;
//    data[1] = 0;
//    data[2] = 0;
//    data[3] = 0;
//    data[4] = 0;
//    data[5] = 0;
//    data[6] = 0;
//    data[7] = 0;
//
//    write_can(hcan1, CAN_CHASSIS_ID, data);
    stop_chassis = 1;
}

void HAL_CAN_RxFifo0MsgPendingCallback(CAN_HandleTypeDef *hcan){
    CAN_RxHeaderTypeDef rx_header;
    uint8_t rx_data[8];
    HAL_CAN_GetRxMessage(hcan, CAN_RX_FIFO0, &rx_header, rx_data);
    if(hcan==&COM_CAN){
        switch (rx_header.StdId){
            case CAN_DOWN_TX_INFO :
                chassis_mode = rx_data[0];
                break;
            default:
                break;
        }
    }
    if(hcan==&CONTROL_CAN) {
        switch (rx_header.StdId) {
            case CAN_YAW_MOTOR_ID: {
                if (YawMotor.msg_cnt++ <= 50)
                    get_motor_offset(&YawMotor, rx_data);
                else
                    get_moto_info(&YawMotor, rx_data);
                err_detector_hook(GIMBAL_YAW_OFFLINE);
            }
                break;
            case CAN_PIT_MOTOR_ID: {
                if (PitMotor.msg_cnt++ <= 50)
                    get_motor_offset(&PitMotor, rx_data);
                else
                    get_moto_info(&PitMotor, rx_data);
                err_detector_hook(GIMBAL_PIT_OFFLINE);
            }
            case CAN_3508_M1_ID:
            {
                moto_shoot[0].msg_cnt++ <= 50 ? get_moto_offset(&moto_shoot[0], rx_data) : \
                encoder_data_handle(&moto_shoot[0], rx_data);
                //通过转速变化判断是否有子弹射出,同时计算射出数量
//			last_state = shoot_status;
//			if(-moto_shoot[0].speed_rpm < SHOT_SUCCESS_FRIC_WHEEL_SPEED && -moto_shoot[0].speed_rpm > SHOT_ABLE_FRIC_WHEEL_SPEED){
//				shoot_status = 1;
//				GPIOE->BSRR=0x40;			//PE6置位
//			}
//			else{
//				shoot_status = 0;
//				GPIOE->BSRR=0x400000;	//PE6复位
//			}
//			if(last_state == 0 && shoot_status == 1){
//				shoot_cnt++;
//			}

                err_detector_hook(AMMO_BOOSTER1_OFFLINE);
            }
                break;
            case CAN_3508_M2_ID:
            {
                //PE4置位
                //GPIOE->BSRR=0x10;
                moto_shoot[1].msg_cnt++ <= 50 ? get_moto_offset(&moto_shoot[1], rx_data) : \
      encoder_data_handle(&moto_shoot[1], rx_data);
                err_detector_hook(AMMO_BOOSTER2_OFFLINE);
                //PE4复位
                //GPIOE->BSRR=0x100000;
            }
                break;
                //拨弹电机
            case CAN_3508_M3_ID:
            {
                moto_trigger.msg_cnt++;
                moto_trigger.msg_cnt <= 10 ? get_moto_offset(&moto_trigger, rx_data) : encoder_data_handle(&moto_trigger, rx_data);
                err_detector_hook(TRIGGER_MOTO_OFFLINE);
            }
                break;
            default:
            {
            }
                break;
        }

        }
    }
//        case CAN_3508_M1_ID:
//        {
//            ChassisMotor[0].msg_cnt++ <= 50 ? get_motor_offset(&ChassisMotor[0], rx_data) : \
//            get_moto_info(&ChassisMotor[0], rx_data);
//            err_detector_hook(CHASSIS_M1_OFFLINE);
//        }
//        break;
//        case CAN_3508_M2_ID:
//        {
//            ChassisMotor[1].msg_cnt++ <= 50 ? get_motor_offset(&ChassisMotor[1], rx_data) : \
//            get_moto_info(&ChassisMotor[1], rx_data);
//            err_detector_hook(CHASSIS_M2_OFFLINE);
//        }
//        break;
//
//        case CAN_3508_M3_ID:
//        {
//            ChassisMotor[2].msg_cnt++ <= 50 ? get_motor_offset(&ChassisMotor[2], rx_data) : \
//            get_moto_info(&ChassisMotor[2], rx_data);
//            err_detector_hook(CHASSIS_M3_OFFLINE);
//        }
//        break;
//        case CAN_3508_M4_ID:
//        {
//            ChassisMotor[3].msg_cnt++ <= 50 ? get_motor_offset(&ChassisMotor[3], rx_data) : \
//            get_moto_info(&ChassisMotor[3], rx_data);
//            err_detector_hook(CHASSIS_M4_OFFLINE);
//        }
//        break;
//
//        case CAN_SUPERCAP_RECV:
//            //PowerDataResolve(rx_data);
//        default:
//        {
//        }
//        break;
//        }
//    }

void write_can(CAN_HandleTypeDef can, uint32_t send_id, uint8_t send_data[]){
    uint32_t send_mail_box;
    tx_message.StdId = send_id;
    tx_message.IDE = CAN_ID_STD;
    tx_message.RTR = CAN_RTR_DATA;
    tx_message.DLC = 0x08;
    for(int i=0;i<8;i++)
        can_send_data[i] = send_data[i];
    HAL_CAN_AddTxMessage(&can, &tx_message, can_send_data, &send_mail_box);
}



/**
  * @brief     获得电机初始偏差
  * @param     ptr: 电机参数 moto_measure_t 结构体指针
  * @param     data: 接收到的电机 CAN 数据指针
  */
static void get_moto_offset(moto_measure_t *ptr, uint8_t data[])
{
    ptr->ecd        = (uint16_t)(data[0] << 8 | data[1]);
    ptr->offset_ecd = ptr->ecd;
}

/**
  * @brief     计算电机的转速rmp 圈数round_cnt
  *            总编码器数值total_ecd 总旋转的角度total_angle
  * @param     ptr: 电机参数 moto_measure_t 结构体指针
  * @param     data: 接收到的电机 CAN 数据指针
  */
static void encoder_data_handle(moto_measure_t *ptr, uint8_t data[])
{
    int32_t temp_sum = 0;

    ptr->last_ecd      = ptr->ecd;
    //转子机械角度
    ptr->ecd           = (uint16_t)(data[0] << 8 | data[1]);
    //转子转速
    ptr->speed_rpm     = (int16_t)(data[2] << 8 | data[3]);
    //转矩电流没有处理
    if (ptr->ecd - ptr->last_ecd > 4096)
    {
        ptr->round_cnt--;
        ptr->ecd_raw_rate = ptr->ecd - ptr->last_ecd - 8192;
    }
    else if (ptr->ecd - ptr->last_ecd < -4096)
    {
        ptr->round_cnt++;
        ptr->ecd_raw_rate = ptr->ecd - ptr->last_ecd + 8192;
    }
    else
    {
        ptr->ecd_raw_rate = ptr->ecd - ptr->last_ecd;
    }

    ptr->total_ecd = ptr->round_cnt * 8192 + ptr->ecd - ptr->offset_ecd;
    ptr->total_angle = ptr->total_ecd * 360 / 8192;


    ptr->rate_buf[ptr->buf_cut++] = ptr->ecd_raw_rate;
    if (ptr->buf_cut >= FILTER_BUF)
        ptr->buf_cut = 0;
    for (uint8_t i = 0; i < FILTER_BUF; i++)
    {
        temp_sum += ptr->rate_buf[i];
    }
    ptr->filter_rate = (int32_t)(temp_sum/FILTER_BUF);
}


/**
  * @brief     发送云台电机电流数据到电调
  */
extern int16_t trigger_moto_current;

//#ifdef TEST_ON_ICRA
//void send_gimbal_moto_current(int16_t yaw_current, int16_t pit_current)
//{
//    static uint8_t data[8];
//    static uint8_t data_yaw[8];
//
//    data[0] = 0;
//    data[1] = 0;
//    data[2] = pit_current >> 8;
//    data[3] = pit_current;
//    //data[4] = trigger_current >> 8;
//    //data[5] = trigger_current;
//    data[6] = 0;
//    data[7] = 0;
//
//    data_yaw[0] = 0;
//    data_yaw[1] = 0;
//    data_yaw[2] = 0;
//    data_yaw[3] = 0;
//    data_yaw[4] = yaw_current >> 8;
//    data_yaw[5] = yaw_current;
//    data_yaw[6] = 0;
//    data_yaw[7] = 0;
//
//    write_can(CONTROL_CAN, CAN_GIMBAL_ID_PITCH, data);
//    write_can(CONTROL_CAN, CAN_GIMBAL_ID_YAW, data_yaw);
//}
//void send_gimbal_moto_zero_current(void)
//{
//    static uint8_t data[8];
//
//    data[0] = 0;
//    data[1] = 0;
//    data[2] = 0;
//    data[3] = 0;
//    data[4] = 0;
//    data[5] = 0;
//    data[6] = 0;
//    data[7] = 0;
//
//    write_can(CONTROL_CAN, CAN_GIMBAL_ID_YAW, data);
//    write_can(CONTROL_CAN, CAN_GIMBAL_ID_PITCH, data);
//}
//
//#else
void send_gimbal_moto_current(int16_t yaw_current, int16_t pit_current)
{
    static uint8_t data[8];

    data[0] = yaw_current >> 8;
    data[1] = yaw_current;
    data[2] = pit_current >> 8;
    data[3] = pit_current;
    //data[4] = trigger_current >> 8;
    //data[5] = trigger_current;
    data[6] = 0;
    data[7] = 0;

    write_can(CONTROL_CAN, CAN_GIMBAL_ID, data);
}
void send_gimbal_moto_zero_current(void)
{
    static uint8_t data[8];

    data[0] = 0;
    data[1] = 0;
    data[2] = 0;
    data[3] = 0;
    data[4] = 0;
    data[5] = 0;
    data[6] = 0;
    data[7] = 0;

    write_can(CONTROL_CAN, CAN_GIMBAL_ID, data);
}
//#endif

void send_shoot_moto_current(int16_t left_current,int16_t right_current, int16_t trigger_current)
{
    static uint8_t data[8];

    data[0] = left_current >> 8;
    data[1] = left_current;
    data[2] = right_current >> 8;
    data[3] = right_current;
    data[4] = trigger_current >> 8;
    data[5] = trigger_current;
    data[6] = 0;
    data[7] = 0;

    write_can(CONTROL_CAN, CAN_CHASSIS_ID, data);
}

void sendSuperCap(void)
{
    uint16_t temPower =9000;//功率设定步进0.01W，范围为3000-13000（30W-130W）
    uint8_t sendbuf[8];//发送的数据内容
    sendbuf[0]=temPower >> 8;
    sendbuf[1]=temPower;
    write_can(COM_CAN, CAN_SUPER_CAP_ID, sendbuf);

}
