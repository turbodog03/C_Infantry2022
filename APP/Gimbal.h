//
// Created by turboDog on 2021/11/17.
//

#ifndef BOARD_C_INFANTRY_GIMBAL_H
#define BOARD_C_INFANTRY_GIMBAL_H

#include "stm32f4xx_hal.h"
#include "bsp_imu.h"
/* ��̨�������� (ms) */
#define GIMBAL_PERIOD 5
/* ��̨���г�ʼ��ʱ�� (ms) */
#define BACK_CENTER_TIME 1000

#define YAW_MOTOR_MAXOUT 30000
#define PITCH_MOTOR_MAXOUT 30000

// Gimbal attitude define
#define GIMBAL_MAX_DEPRESSION -10
#define GIMBAL_MAX_ELEVATION 30

// Gimbal control define
#define RC_STICK_YAW_RATIO 3
#define RC_STICK_PITCH_RATIO 1.5
#define RC_MOUSE_YAW_RATIO 5
#define RC_MOUSE_PITCH_RATIO -5

#define YAW_V_PID_MAXOUT_M 8000
#define YAW_V_PID_MAXINTEGRAL_M 3000
#define YAW_V_PID_KP_M 300
#define YAW_V_PID_KI_M 20.0
#define YAW_V_PID_KD_M 0.01
#define YAW_V_PID_LPF_M 0.000001
#define YAW_V_PID_D_LPF_M 0.000001

#define YAW_A_PID_MAXOUT_M 320
#define YAW_A_PID_MAXINTEGRAL_M 320
#define YAW_A_PID_KP_M 30
#define YAW_A_PID_KI_M 0.5
#define YAW_A_PID_KD_M 0.001

#define YAW_V_PID_MAXOUT_A 8000
#define YAW_V_PID_MAXINTEGRAL_A 3000
#define YAW_V_PID_KP_A 100
#define YAW_V_PID_KI_A 2.0
#define YAW_V_PID_KD_A 0.2
#define YAW_V_PID_LPF_A 0.000001
#define YAW_V_PID_D_LPF_A 0.000001

#define YAW_A_PID_MAXOUT_A 4000
#define YAW_A_PID_MAXINTEGRAL_A 50
#define YAW_A_PID_KP_A 30
#define YAW_A_PID_KI_A 300
#define YAW_A_PID_KD_A 0.05

#define PITCH_V_PID_MAXOUT 16000
#define PITCH_V_PID_MAXINTEGRAL 2000
#define PITCH_V_PID_KP 16
#define PITCH_V_PID_KI 2.0
#define PITCH_V_PID_KD 0.015
#define PITCH_V_PID_LPF 0.000001
#define PITCH_V_PID_D_LPF 0.08

#define PITCH_A_PID_MAXOUT 3000
#define PITCH_A_PID_MAXINTEGRAL 600
#define PITCH_A_PID_KP 90.0
#define PITCH_A_PID_KI 180
#define PITCH_A_PID_KD 3.5
#define PITCH_A_PID_LPF 0.1
#define PITCH_A_PID_D_LPF 0.01

#define PITCH_V_FFC_MAXOUT 30000
#define PITCH_V_FCC_C0 25
#define PITCH_V_FCC_C1 100 * 0
#define PITCH_V_FCC_C2 0
#define PITCH_V_FCC_LPF 0.005

#define PITCH_A_FFC_MAXOUT 320
#define PITCH_A_FCC_C0 0
#define PITCH_A_FCC_C1 1
#define PITCH_A_FCC_C2 0
#define PITCH_A_FCC_LPF 0.00001

/**
  * @brief     ��̨����������
  */
void gimbal_task(const void* argu);

/**
  * @brief     ��̨����ģʽö��
  */
typedef enum
{
    GIMBAL_INIT = 0,         //��̨��ʼ��
    GIMBAL_RELAX = 1,            //��̨�ϵ�
    GIMBAL_CLOSE_LOOP_ZGYRO = 2, //��̨����imu z��Ƕ�
    GIMBAL_NO_ACTION = 3,        //���ֶ��ź�����״̬
    GIMBAL_AUTO	= 4						 //��̨����ģʽ
} gimbal_mode_e;

/**
  * @brief     ��̨�����ź�����״̬ö��
  */
typedef enum
{
    NO_ACTION = 0,           //�޿����ź�����
    IS_ACTION,               //�п����ź�����
} action_mode_e;

/**
  * @brief     ��̨����״̬ö��
  */
typedef enum
{
    BACK_PIT_STEP,           //��̨ pitch �����
    YAW_BACK_STEP,           //��̨ yaw �����
    BACK_IS_OK,              //��̨�������
} gimbal_back_e;

/**
  * @brief     ��̨�������ݽṹ��
  */
typedef struct
{
    gimbal_mode_e ctrl_mode; //��̨��ǰ����ģʽ
    gimbal_mode_e last_mode; //��̨�ϴο���ģʽ

    action_mode_e ac_mode;   //��̨�����ź�����ģʽ

    uint8_t  no_action_flag; //�޿����źű�־
    uint32_t no_action_time; //�޿����ź�ʱ��

    float ecd_offset_angle;  //��̨��ʼ������ֵ
    float yaw_offset_angle;  //��̨��ʼ yaw ��Ƕ�
    float pit_offset_angle;  //��̨��ʼ yaw ��Ƕ�
} gimbal_yaw_t;

//���鷢�ͽṹ��
typedef __packed struct
{
    uint16_t head;						//֡ͷ
    float pitchAngleGet;    	//pitch��Ƕ�
    float yawAngleGet;      	//yaw��Ƕ�
    uint8_t rotateDricetion;  //��ת����
    float timeBais;         	//Ԥ��ʱ��ƫ��
    float compensateBais;   	//��������ƫ��
    uint8_t gimbal_mode;	 		//��̨ģʽ
}send_frame;


//������սṹ��
typedef __packed struct
{
    uint16_t head;  				 //֡ͷ
    float pitchAngleSet;    //pitch��Ƕ��趨ֵ
    float yawAngleSet;      //yaw��Ƕ��趨ֵ
    float targetAngle;      //Ŀ��װ�װ�Ƕ�
    uint8_t shootCommand;   //����ָ��
}recv_frame;

extern gimbal_yaw_t gim;
extern imu_t        imu;

/* ��̨ PID ����������� */
extern float yaw_angle_ref;
extern float pit_angle_ref;
extern float yaw_angle_fdb;
extern float pit_angle_fdb;
extern float yaw_speed_ref;
extern float pit_speed_ref;
/* ��̨��ԽǶ� */
extern float yaw_relative_angle;
extern float pit_relative_angle;
extern char color_flag;


/**
  * @brief     ��ȡ��̨�е�ı�����ƫ������
  * @param     pit_offset: pitch �����������ָ��
  * @param     yaw_offset: yaw �����������ָ��
  */
static void read_gimbal_offset(int32_t *pit_offset, int32_t *yaw_offset);
/**
  * @brief     ��ȡ��̨��������Ϣ
  */
static void get_gimbal_information(void);
/**
  * @brief     ��ȡ��̨����ģʽ
  */
static void get_gimbal_mode(void);
/**
  * @brief     ��̨���Ʋ�����ʼ��
  */
static void gimbal_init_param(void);
/**
  * @brief     ��̨���г�ʼ��ģʽ������
  */
static void gimbal_init_handle(void);
/**
  * @brief     ��̨�޿����ź�����ģʽ������
  */

static void gimbal_noaction_handle(void);
/**
  * @brief     ��̨����������ջ����ƴ�����
  */
static void gimbal_loop_handle(void);

/**
  * @brief     ��̨�Զ�����ƴ���ӿ�
  */
void gimbal_custom_control(void);
/**
  * @brief     ��̨������ƴ���ӿ�
  */
void gimbal_auto_control(void);
/**
  * @brief     ��̨ yaw ��λ�ñջ�����
  */
void gimbal_yaw_control(void);
/**
  * @brief     ��̨ pitch ��λ�ñջ�����
  */
void gimbal_pitch_control(void);
/**
  * @brief     ��������Զ�����ƴ���ӿ�
  */
void shoot_custom_control(void);
/**
  * @brief     ����Ħ���ֿ��ƺ���
  */
void turn_on_off_friction_wheel(void);

void pid_reset_manual(void);

void pid_reset_auto(void);

extern send_frame auto_tx_data;
#endif //BOARD_C_INFANTRY_GIMBAL_H
