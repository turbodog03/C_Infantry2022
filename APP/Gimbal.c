//
// Created by turboDog on 2021/11/17.
//


#include "Gimbal.h"
#include "chassis_task.h"
#include "Detect.h"

#include "bsp_can.h"
#include "bsp_uart.h"

#include "calibrate.h"
#include "keyboard.h"
#include "pid.h"
#include "ramp.h"
#include "sys.h"

#include "cmsis_os.h"
#include "stdlib.h"
#include "stdio.h"
#include "math.h"


/* ��������̨������ʹ�ñ����Ķ��壬�����޸� */
/* gimbal global information */
gimbal_yaw_t gim;
imu_t        imu;

char color_flag=0;//0Ϊ��ɫ��1Ϊ��ɫ
static _Bool auto_pid_flag = 1;
static _Bool manual_pid_flag = 0;

send_frame auto_tx_data =
        {
                .head=0xbbbb,
                .pitchAngleGet=0.0f,
                .yawAngleGet=0.0f,
                .rotateDricetion=0,  //��ת����
                .timeBais=0.0f,        	//Ԥ��ʱ��ƫ��
                .compensateBais=0.0f,   	//��������ƫ��
                .gimbal_mode=0	 		//��̨ģʽ
        };

uint8_t send_flag=0;
/* gimbal pid parameter */

float yaw_angle_fdb = 0;
float pit_angle_fdb = 0;
float c[3] = {0};

/* read from flash */
int32_t   pit_center_offset = 0;
int32_t   yaw_center_offset = 0;

/* for debug */
uint32_t gimbal_time_last;
int gimbal_time_ms;
/* ִ����̨��������ĺ��� */
void gimbal_task(const void* argu)
{
    //��̨���������ʼ��
    gimbal_init_param();

    //��flash�ж�ȡ��̨�е�λ��
    read_gimbal_offset(&pit_center_offset, &yaw_center_offset);
    //��ʼ�����鷢��֡
    auto_tx_data.head = 0xbbbb;
    auto_tx_data.rotateDricetion = 1; //Ĭ��˳ʱ��

    //��̨��������ѭ��
    uint32_t gimbal_wake_time = osKernelSysTick();
    while (1)
    {
        gimbal_time_ms = HAL_GetTick() - gimbal_time_last;
        gimbal_time_last = HAL_GetTick();

        //��ȡ��̨��������Ϣ
        get_gimbal_information();

        //���ݿ��������л���̨״̬
        get_gimbal_mode();
        //��ȡ�������жϵĴ��ת��
        if(rc.kb.bit.G)
        {
            if(rc.kb.bit.SHIFT)
                auto_tx_data.rotateDricetion=1;//��ת
            else
                auto_tx_data.rotateDricetion=0;//��ת
        }

        switch (gim.ctrl_mode)
        {
            //��̨��ʼ��״̬
            case GIMBAL_INIT:
            {
                gimbal_init_handle();
            }break;

                //��̨������״̬
            case GIMBAL_NO_ACTION:
            {
                gimbal_noaction_handle();
            }break;

                //��̨�ջ�����ģʽ
            case GIMBAL_CLOSE_LOOP_ZGYRO:
            {
                if(manual_pid_flag == 0){
                    pid_reset_manual();
                    manual_pid_flag = 1;
                    auto_pid_flag = 0;
                }
                gimbal_loop_handle();
            }break;
            case GIMBAL_AUTO:
            {
                if(auto_pid_flag == 0){
                    pid_reset_auto();
                    auto_pid_flag = 1;
                    manual_pid_flag = 0;
                }
                gimbal_auto_control();

            }break;
            default:
                break;
        }

        //��̨���ƣ������ģ�����ߣ����ж���̨������
        if (gim.ctrl_mode != GIMBAL_RELAX                 //��̨�ͷ�ģʽ
            && !glb_err.err_list[REMOTE_CTRL_OFFLINE].err_exist //ң��������
            && !glb_err.err_list[GIMBAL_YAW_OFFLINE].err_exist  //yaw��������
            && !glb_err.err_list[GIMBAL_PIT_OFFLINE].err_exist) //pitch��������
        {
            gimbal_custom_control();
        }
        else
        {
            gim.ctrl_mode = GIMBAL_RELAX;
            pid_trigger.iout = 0;
            send_gimbal_moto_zero_current();
        }
        //���ͷ�������
        //frame_send data_send={0xaa,yaw_relative_angle,color_flag,pit_relative_angle,0,1,0,0xbb};
        //frame_send data_send={0xaa,yaw_angle_ref,0,pit_angle_ref,0,1,0,0xbb};
        auto_tx_data.pitchAngleGet = pit_relative_angle;
        auto_tx_data.yawAngleGet = yaw_relative_angle;
        auto_tx_data.gimbal_mode = gim.ctrl_mode;
//		if(shoot_cnt>last_cnt)
//			auto_tx_data.shootStatusGet = 1;
//		else
//			auto_tx_data.shootStatusGet = 0;
//		last_cnt=shoot_cnt;

        write_uart(NUC_UART,(uint8_t*)&auto_tx_data,sizeof(auto_tx_data));
        //��̨�������ڿ��� 5ms
        osDelayUntil(&gimbal_wake_time, GIMBAL_PERIOD);
    }
}

/* ����Ϊ��̨������õ��ڲ�����������Ķ� */

/**
  * @brief     get relative position angle to center
  * @param[in] raw_ecd: gimbal motor encoder raw angle
  * @param[in] center_offset: read gimbal_cali_data from chip flash
  * @retval    relative angle, unit is degree.
  */
static int16_t get_relative_pos(int16_t raw_ecd, int16_t center_offset)
{
    int16_t tmp = 0;
    if (center_offset >= 4096)
    {
        if (raw_ecd > center_offset - 4096)
            tmp = raw_ecd - center_offset;
        else
            tmp = raw_ecd + 8192 - center_offset;
    }
    else
    {
        if (raw_ecd > center_offset + 4096)
            tmp = raw_ecd - 8192 - center_offset;
        else
            tmp = raw_ecd - center_offset;
    }
    return tmp;
}

static action_mode_e remote_is_action(void)
{
    if ((abs(rc.ch1) >= 10)
        || (abs(rc.ch2) >= 10)
        || (abs(rc.ch3) >= 10)
        || (abs(rc.ch4) >= 10)
        || (abs(rc.mouse.x) >= 5)
        || (abs(rc.mouse.y) >= 5))
    {
        return IS_ACTION;
    }
    else
    {
        return NO_ACTION;
    }
}

/* gimbal relative position param */
float     pit_relative_angle;
float     yaw_relative_angle;  //unit: degree
void get_gimbal_information(void)
{
    //��ȡ imu ����
    get_imu_data(&imu);

    //��̨��ԽǶȻ�ȡ
    yaw_relative_angle = get_relative_pos(YawMotor.RawAngle, yaw_center_offset)/22.75f;
    pit_relative_angle = get_relative_pos(PitMotor.RawAngle, pit_center_offset)/22.75f;

    //��̨ģʽ��ȡ
    gim.ac_mode = remote_is_action();
    gim.last_mode = gim.ctrl_mode;

    //��ȡң�����Ŀ�������
    pc_kb_hook();
}

uint8_t last_cali_cmd;
extern TaskHandle_t task1_t;
void read_gimbal_offset(int32_t *pit_offset, int32_t *yaw_offset)
{

    if (glb_cali_data.gimbal_cali_data.calied_flag == CALIED_FLAG)
    {
        *pit_offset = glb_cali_data.gimbal_cali_data.pit_offset;
        *yaw_offset = glb_cali_data.gimbal_cali_data.yaw_offset;

        //һ�����������ѵ�������
        osThreadResume(task1_t);
    }
    else
    {
        //��̨û��У׼������У׼����
        while (1)
        {
//		int tmp=sprintf(print_buf,"%d %d\n",moto_pit.ecd,moto_yaw.ecd);
//		write_uart(BLUETOOTH_UART,(uint8_t*)print_buf,tmp);
            gimbal_cali_hook();
            send_chassis_moto_zero_current();
            send_gimbal_moto_zero_current();

            osDelay(10);
        }
    }
}

gimbal_back_e gimbal_back_step;

static void gimbal_back_param(void)
{
    gimbal_back_step = BACK_PIT_STEP;
    gim.ecd_offset_angle = yaw_relative_angle;

    ramp_init(&pit_ramp, BACK_CENTER_TIME/GIMBAL_PERIOD);
    ramp_init(&yaw_ramp, BACK_CENTER_TIME/GIMBAL_PERIOD);

    //pid_pit_speed.max_output = 15000;

}


float chassis_angle_ac = 5;
void get_gimbal_mode(void)
{
    switch (rc.sw2)
    {
        case (RC_UP):
        {
            if (gim.ac_mode == NO_ACTION)
            {
                if (gim.ctrl_mode == GIMBAL_CLOSE_LOOP_ZGYRO)
                {
                    //if (fabs(chassis.vw) <= chassis_angle_ac)
                    if (fabs(yaw_relative_angle) <= chassis_angle_ac)
                    {
                        //begin no action handle
                        gim.ctrl_mode = GIMBAL_NO_ACTION;

                        gim.no_action_flag = 1;
                        gim.no_action_time = HAL_GetTick();
                    }
                }
            }
            else  //IS_ACTION mode
            {
                if (gim.ctrl_mode == GIMBAL_NO_ACTION||gim.ctrl_mode == GIMBAL_AUTO)
                {
                    gim.ctrl_mode = GIMBAL_CLOSE_LOOP_ZGYRO;
                    gim.no_action_flag = 0;

                    yaw_angle_ref = 0;
                    gim.yaw_offset_angle = imu.angle_z;
                }

            }

            if (chassis.mode == CHASSIS_SPIN)
            {
                if (gim.ctrl_mode == GIMBAL_NO_ACTION)
                {
                    gim.ctrl_mode = GIMBAL_CLOSE_LOOP_ZGYRO;
                    gim.no_action_flag = 0;

                    yaw_angle_ref = 0;
                    gim.yaw_offset_angle = imu.angle_z;
                }
            }

            if (gim.last_mode == GIMBAL_RELAX)
            {
                gim.ctrl_mode = GIMBAL_INIT;
                gimbal_back_param();
            }
        }
            break;
        case (RC_MI)://����ģʽ
        {
            //���ң��������
            if (gim.ac_mode == NO_ACTION)
            {
                gim.ctrl_mode=GIMBAL_AUTO;
            }
            else  //IS_ACTION mode
            {
                gim.ctrl_mode = GIMBAL_CLOSE_LOOP_ZGYRO;
                gim.no_action_flag = 0;

                yaw_angle_ref = 0;
                gim.yaw_offset_angle = imu.angle_z;
            }
        }break;
        default:
        {
            gim.ctrl_mode = GIMBAL_RELAX;
            gimbal_cali_hook();
        }
            break;
    }

}

void gimbal_loop_handle(void)
{
    //static float chassis_angle_tmp;
    //fdb for feedback
    pit_angle_fdb = pit_relative_angle;
    //pit_angle_fdb = gim.pit_offset_angle-imu.angle_y;
    yaw_angle_fdb = imu.angle_z - gim.yaw_offset_angle;

    //chassis_angle_tmp = yaw_angle_fdb - yaw_relative_angle;
    if (chassis.mode != CHASSIS_FIXED_ROUTE)
    {
        gimbal_yaw_control();
        gimbal_pitch_control();
//    //����yaw��Ļ�Ƕ�
//    if ((yaw_relative_angle >= YAW_ANGLE_MIN) && (yaw_relative_angle <= YAW_ANGLE_MAX))
//    {
//      VAL_LIMIT(yaw_angle_ref, chassis_angle_tmp + YAW_ANGLE_MIN, chassis_angle_tmp + YAW_ANGLE_MAX);
//    }
        //����pitch��Ļ�Ƕ�
        if ((pit_relative_angle >= PIT_ANGLE_MIN) && (pit_relative_angle <= PIT_ANGLE_MAX))
        {
            VAL_LIMIT(pit_angle_ref, PIT_ANGLE_MIN, PIT_ANGLE_MAX);
        }
    }

}

//��̨��ʼ��
void gimbal_init_handle(void)
{
    pit_angle_fdb = pit_relative_angle;
    yaw_angle_fdb = yaw_relative_angle;

    /* gimbal pitch back center */
    pit_angle_ref = pit_relative_angle * (1 - ramp_calc(&pit_ramp));

    switch (gimbal_back_step)
    {
        case BACK_PIT_STEP:
        {
            /* keep yaw unmove this time */
            yaw_angle_ref = gim.ecd_offset_angle;

            if(fabs(pit_angle_fdb) <= 2.0f)
                gimbal_back_step = YAW_BACK_STEP;
        }break;

        case YAW_BACK_STEP:
        {
            /* yaw back center after pitch arrive */
            yaw_angle_ref = yaw_relative_angle * ( 1 - ramp_calc(&yaw_ramp));

            if (fabs(yaw_angle_fdb) <= 2.0f)
                gimbal_back_step = BACK_IS_OK;
        }break;

        case BACK_IS_OK:
        {
            /* yaw arrive and switch gimbal state */
            gim.ctrl_mode = GIMBAL_CLOSE_LOOP_ZGYRO;

            gim.yaw_offset_angle = imu.angle_z;
            gim.pit_offset_angle = imu.angle_y;
            pit_angle_ref = 0;
            yaw_angle_ref = 0;

            //pid_pit_speed.max_output = 8000;
        }break;
    }
}


void gimbal_noaction_handle(void)
{
    if (gim.no_action_flag == 1)
    {
        if ((HAL_GetTick() - gim.no_action_time) < 1500)
        {
            gimbal_loop_handle();
        }
        else
        {
            gim.no_action_flag = 2;
            //�����������ִֻ��һ��
            yaw_angle_ref = 0;
            gim.yaw_offset_angle = imu.angle_z;//���ó�ʼ�Ƕ�
        }
    }

    if (gim.no_action_flag == 2)
    {
        pit_angle_fdb = pit_relative_angle;
        yaw_angle_fdb = imu.angle_z - gim.yaw_offset_angle;;
    }
}

/**
  * @brief     initialize gimbal pid parameter, such as pitch/yaw/trigger motor,
  *            imu temperature
  * @attention before gimbal control loop in gimbal_task() function
  */
void gimbal_init_param(void)
{
    //��� bug
    osDelay(3000);

    /* ��̨pitch����PID������ʼ�� */
    PID_Init(&PitMotor.PID_Velocity, PITCH_V_PID_MAXOUT, PITCH_V_PID_MAXINTEGRAL, 0,
             PITCH_V_PID_KP, PITCH_V_PID_KI, PITCH_V_PID_KD, 1000, 5000, PITCH_V_PID_LPF, PITCH_V_PID_D_LPF, 0,
             Integral_Limit | Trapezoid_Intergral | OutputFilter | DerivativeFilter);
    c[0] = PITCH_V_FCC_C0;
    c[1] = PITCH_V_FCC_C1;
    c[2] = PITCH_V_FCC_C2;
    Feedforward_Init(&PitMotor.FFC_Velocity, PITCH_V_FFC_MAXOUT, c, PITCH_V_FCC_LPF, 4, 4);
    PID_Init(&PitMotor.PID_Angle, PITCH_A_PID_MAXOUT, PITCH_A_PID_MAXINTEGRAL, 0,
             PITCH_A_PID_KP, PITCH_A_PID_KI, PITCH_A_PID_KD, 5, 2, PITCH_A_PID_LPF, PITCH_A_PID_D_LPF, 0,
             Integral_Limit | Trapezoid_Intergral| DerivativeFilter|Derivative_On_Measurement);
    c[0] = PITCH_A_FCC_C0;
    c[1] = PITCH_A_FCC_C1;
    c[2] = PITCH_A_FCC_C2;
    Feedforward_Init(&PitMotor.FFC_Angle, PITCH_A_FFC_MAXOUT, c, PITCH_A_FCC_LPF, 3, 3);
    PitMotor.Max_Out = PITCH_MOTOR_MAXOUT * 0.9f;

    /* ��̨yaw����PID������ʼ�� */
    PID_Init(&YawMotor.PID_Velocity, YAW_V_PID_MAXOUT_M, YAW_V_PID_MAXINTEGRAL_M, 0,
             YAW_V_PID_KP_M, YAW_V_PID_KI_M, YAW_V_PID_KD_M, 1000, 5000,
             YAW_V_PID_LPF_M, YAW_V_PID_D_LPF_M, 0,
             Integral_Limit | Trapezoid_Intergral);
    PID_Init(&YawMotor.PID_Angle, YAW_A_PID_MAXOUT_M, YAW_A_PID_MAXINTEGRAL_M, 0.0,
             YAW_A_PID_KP_M, YAW_A_PID_KI_M, YAW_A_PID_KD_M, 5, 2, 0, 0, 0,
             Integral_Limit | Trapezoid_Intergral);
    YawMotor.Max_Out = YAW_MOTOR_MAXOUT * 0.9f;
    /* ����̨�ĳ�ʼ��״̬����Ϊ�ͷ� */
    gim.ctrl_mode = GIMBAL_RELAX;
}

void pid_reset_manual()
{
    PID_Init(&YawMotor.PID_Velocity, YAW_V_PID_MAXOUT_M, YAW_V_PID_MAXINTEGRAL_M, 0,
             YAW_V_PID_KP_M, YAW_V_PID_KI_M, YAW_V_PID_KD_M, 1000, 5000,
             YAW_V_PID_LPF_M, YAW_V_PID_D_LPF_M, 0,
             Integral_Limit | Trapezoid_Intergral);
    PID_Init(&YawMotor.PID_Angle, YAW_A_PID_MAXOUT_M, YAW_A_PID_MAXINTEGRAL_M, 0.0,
             YAW_A_PID_KP_M, YAW_A_PID_KI_M, YAW_A_PID_KD_M, 5, 2, 0, 0, 0,
             Integral_Limit | Trapezoid_Intergral);
//	YawMotor.PID_Velocity.MaxOut=YAW_V_PID_MAXOUT_M;
//	YawMotor.PID_Velocity.IntegralLimit=YAW_V_PID_MAXINTEGRAL_M;
//	YawMotor.PID_Velocity.Kp=YAW_V_PID_KP_M;
//	YawMotor.PID_Velocity.Ki=YAW_V_PID_KI_M;
//	YawMotor.PID_Velocity.Kd=YAW_V_PID_KD_M;
//
//	YawMotor.PID_Angle.MaxOut=YAW_A_PID_MAXOUT_M;
//	YawMotor.PID_Angle.IntegralLimit=YAW_A_PID_MAXINTEGRAL_M;
//	YawMotor.PID_Angle.Kp=YAW_A_PID_KP_M;
//	YawMotor.PID_Angle.Ki=YAW_A_PID_KI_M;
//	YawMotor.PID_Angle.Kd=YAW_A_PID_KD_M;
}

void pid_reset_auto()
{
//	YawMotor.PID_Velocity.MaxOut=YAW_V_PID_MAXOUT_A;
//	YawMotor.PID_Velocity.IntegralLimit=YAW_V_PID_MAXINTEGRAL_A;
//	YawMotor.PID_Velocity.Kp=YAW_V_PID_KP_A;
//	YawMotor.PID_Velocity.Ki=YAW_V_PID_KI_A;
//	YawMotor.PID_Velocity.Kd=YAW_V_PID_KD_A;
//
//	YawMotor.PID_Angle.MaxOut=YAW_A_PID_MAXOUT_A;
//	YawMotor.PID_Angle.IntegralLimit=YAW_A_PID_MAXINTEGRAL_A;
//	YawMotor.PID_Angle.Kp=YAW_A_PID_KP_A;
//	YawMotor.PID_Angle.Ki=YAW_A_PID_KI_A;
//	YawMotor.PID_Angle.Kd=YAW_A_PID_KD_A;
    PID_Init(&YawMotor.PID_Velocity, YAW_V_PID_MAXOUT_A, YAW_V_PID_MAXINTEGRAL_A, 0,
             YAW_V_PID_KP_A, YAW_V_PID_KI_A, YAW_V_PID_KD_A, 1000, 5000,
             YAW_V_PID_LPF_A, YAW_V_PID_D_LPF_A, 0,
             Integral_Limit | Trapezoid_Intergral);
    PID_Init(&YawMotor.PID_Angle, YAW_A_PID_MAXOUT_A, YAW_A_PID_MAXINTEGRAL_A, 0.0,
             YAW_A_PID_KP_A, YAW_A_PID_KI_A, YAW_A_PID_KD_A, 5, 2, 0, 0, 0,
             Integral_Limit | Trapezoid_Intergral);
}
#endif //BOARD_C_INFANTRY_GIMBAL_H
