//
// Created by turboDog on 2021/11/17.
//


#include <bsp_imu.h>
#include <usart.h>
#include "Gimbal.h"
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

#include "math.h"




/* 以下是云台任务所使用变量的定义，请勿修改 */
/* gimbal global information */
gimbal_yaw_t gim;
imu_t        imu;

char color_flag=0;//0为红色，1为蓝色
static _Bool auto_pid_flag = 1;
static _Bool manual_pid_flag = 0;

send_frame auto_tx_data =
        {
                .head=0xbbbb,
                .pitchAngleGet=0.0f,
                .yawAngleGet=0.0f,
                .rotateDricetion=0,  //旋转方向
                .timeBais=0.0f,        	//预测时间偏置
                .compensateBais=0.0f,   	//弹道补偿偏置
                .gimbal_mode=0	 		//云台模式
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
/* 执行云台功能任务的函数 */
void gimbal_task(const void* argu)
{
    //云台电机参数初始化
    gimbal_init_param();

    //从flash中读取云台中点位置
    read_gimbal_offset(&pit_center_offset, &yaw_center_offset);
    //初始化自瞄发送帧
    auto_tx_data.head = 0xbbbb;
    auto_tx_data.rotateDricetion = 1; //默认顺时针

    //云台控制任务循环
    uint32_t gimbal_wake_time = osKernelSysTick();
    while (1)
    {
        gimbal_time_ms = HAL_GetTick() - gimbal_time_last;
        gimbal_time_last = HAL_GetTick();

        //获取云台传感器信息
        get_gimbal_information();

        //根据控制命令切换云台状态
        get_gimbal_mode();
        //获取操作手判断的大符转向
        if(rc.kb.bit.G)
        {
            if(rc.kb.bit.SHIFT)
                auto_tx_data.rotateDricetion=1;//正转
            else
                auto_tx_data.rotateDricetion=0;//反转
        }

        switch (gim.ctrl_mode)
        {
            //云台初始化状态
            case GIMBAL_INIT:
            {
                gimbal_init_handle();
            }break;

                //云台无输入状态
            case GIMBAL_NO_ACTION:
            {
                gimbal_noaction_handle();
            }break;

                //云台闭环控制模式
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

        //云台控制，如果有模块离线，就切断云台电机输出
        if (gim.ctrl_mode != GIMBAL_RELAX                 //云台释放模式
            && !glb_err.err_list[REMOTE_CTRL_OFFLINE].err_exist //遥控器离线
            && !glb_err.err_list[GIMBAL_YAW_OFFLINE].err_exist  //yaw轴电机离线
            && !glb_err.err_list[GIMBAL_PIT_OFFLINE].err_exist) //pitch轴电机离线
        {
            gimbal_custom_control();
        }
        else
        {
            gim.ctrl_mode = GIMBAL_RELAX;
            pid_trigger.iout = 0;
            send_gimbal_moto_zero_current();
        }
        //发送反馈数据
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
        //云台任务周期控制 5ms
        osDelayUntil(&gimbal_wake_time, GIMBAL_PERIOD);
    }
}

/* 以下为云台任务调用的内部函数，请勿改动 */

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
    //获取 imu 数据
    //TODO
    get_imu_data(&imu);

    //云台相对角度获取
    yaw_relative_angle = get_relative_pos(YawMotor.RawAngle, yaw_center_offset)/22.75f;
    pit_relative_angle = get_relative_pos(PitMotor.RawAngle, pit_center_offset)/22.75f;

    //云台模式获取
    gim.ac_mode = remote_is_action();
    gim.last_mode = gim.ctrl_mode;

    //读取遥控器的控制命令
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

        //一起正常，唤醒底盘任务
        osThreadResume(task1_t);
    }
    else
    {
        //云台没有校准，进入校准处理
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
            //TODO：chassis.mode，是上板根据遥控解算出来还是下板传上来？chassis的模式是只看遥控器数据还是要结合云台状态一起看？
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
        case (RC_MI)://自瞄模式
        {
            //如果遥控无输入
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
//    //限制yaw轴的活动角度
//    if ((yaw_relative_angle >= YAW_ANGLE_MIN) && (yaw_relative_angle <= YAW_ANGLE_MAX))
//    {
//      VAL_LIMIT(yaw_angle_ref, chassis_angle_tmp + YAW_ANGLE_MIN, chassis_angle_tmp + YAW_ANGLE_MAX);
//    }
        //限制pitch轴的活动角度
        if ((pit_relative_angle >= PIT_ANGLE_MIN) && (pit_relative_angle <= PIT_ANGLE_MAX))
        {
            VAL_LIMIT(pit_angle_ref, PIT_ANGLE_MIN, PIT_ANGLE_MAX);
        }
    }

}

//云台初始化
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
            //以下两句代码只执行一次
            yaw_angle_ref = 0;
            gim.yaw_offset_angle = imu.angle_z;//重置初始角度
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
    //电调 bug
    osDelay(3000);

    /* 云台pitch轴电机PID参数初始化 */
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

    /* 云台yaw轴电机PID参数初始化 */
    PID_Init(&YawMotor.PID_Velocity, YAW_V_PID_MAXOUT_M, YAW_V_PID_MAXINTEGRAL_M, 0,
             YAW_V_PID_KP_M, YAW_V_PID_KI_M, YAW_V_PID_KD_M, 1000, 5000,
             YAW_V_PID_LPF_M, YAW_V_PID_D_LPF_M, 0,
             Integral_Limit | Trapezoid_Intergral);
    PID_Init(&YawMotor.PID_Angle, YAW_A_PID_MAXOUT_M, YAW_A_PID_MAXINTEGRAL_M, 0.0,
             YAW_A_PID_KP_M, YAW_A_PID_KI_M, YAW_A_PID_KD_M, 5, 2, 0, 0, 0,
             Integral_Limit | Trapezoid_Intergral);
    YawMotor.Max_Out = YAW_MOTOR_MAXOUT * 0.9f;
    /* 将云台的初始化状态设置为释放 */
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


extern char mmp_buf[];

/* 云台电机期望角度(degree) */
float yaw_angle_ref;
float pit_angle_ref;
/* 云台电机期望角速度(degree/s) */
float yaw_speed_ref;
float pit_speed_ref;
/* 云台电机电流 */
int16_t yaw_moto_current;
int16_t pit_moto_current;


/* 云台控制信号获取，将输入信号积分成云台的角度degree */
void gimbal_yaw_control(void)
{
    //yaw轴的角度累加，单位degree
    yaw_angle_ref += -rc.ch3 * RC_RATIO * GIMBAL_RC_MOVE_RATIO_YAW
                     -rc.mouse.x * KB_RATIO * GIMBAL_PC_MOVE_RATIO_YAW;

}
void gimbal_pitch_control(void)
{
    //pitch轴的角度累加，单位degree
    pit_angle_ref += rc.ch4 * RC_RATIO * GIMBAL_RC_MOVE_RATIO_PIT
                     - rc.mouse.y * KB_RATIO * GIMBAL_PC_MOVE_RATIO_PIT;
}


/* 云台电机转速的闭环控制 */
void gimbal_custom_control(void)
{
    static int16_t last_current = 0;
//	float a[5]={pid_yaw.p,pid_yaw.i,pid_yaw.d,yaw_angle_ref,pit_angle_ref};
//	int16_t b[2]={yaw_moto_current,pit_moto_current};
//	write_uart(BLUETOOTH_UART,(uint8_t*)b,4*sizeof(uint8_t));
//	write_uart(BLUETOOTH_UART,(uint8_t*)"\n",1);
    //闭环计算云台电机的输出电流
//  /* yaw轴预期速度计算，单位degree/s */
//  yaw_speed_ref  = pid_calc(&pid_yaw, yaw_angle_fdb, yaw_angle_ref);    //degree
//  /* yaw轴电机电流计算 */
//  yaw_moto_current = pid_calc(&pid_yaw_speed, imu.gyro_z, yaw_speed_ref+(pid_yaw.err[NOW] - pid_yaw.err[LAST])); //degree/s
    yaw_moto_current = Motor_Angle_Calculate(&YawMotor, yaw_angle_fdb,imu.gyro_z,yaw_angle_ref);

    /* pitch轴俯仰角度限制 */
    VAL_LIMIT(pit_angle_ref, PIT_ANGLE_MIN, PIT_ANGLE_MAX);
//  /* pitch轴预期速度计算，单位degree/s */
//  pit_speed_ref    = pid_calc(&pid_pit, pit_angle_fdb, pit_angle_ref);    //degree
//  /* pitch轴电机电压计算 */

//	pit_moto_current = pid_calc(&pid_pit_speed, imu.gyro_y, pit_speed_ref); //degree/s
//	//滤波
//	pit_moto_current = 0.5*last_current + 0.5*pit_moto_current;
//	last_current = pit_moto_current;
    pit_moto_current = Motor_Angle_Calculate(&PitMotor, pit_angle_fdb,imu.gyro_y,pit_angle_ref);

    //发送电流到云台电机电调
    send_gimbal_moto_current(yaw_moto_current, pit_moto_current);


}
//云台自瞄处理
void gimbal_auto_control()
{
//	float rate=0.3,a,b;
    static float last_p=0.0f,last_y=0.0f;
//	HAL_GPIO_TogglePin(GPIOG,GPIO_PIN_2);
    pit_angle_ref=data_recv.pitchAngleSet*0.7f+last_p*0.3f;
    yaw_angle_ref=data_recv.yawAngleSet;
    //遥控器微调
    gimbal_yaw_control();
    //gimbal_pitch_control();
    //pit_angle_ref=pit_relative_angle+b;

    //限制pit轴的活动角度
    if ((pit_angle_ref >= PIT_ANGLE_MAX) && (pit_angle_ref <= PIT_ANGLE_MIN))
    {
        VAL_LIMIT(yaw_angle_ref, PIT_ANGLE_MIN, PIT_ANGLE_MAX);
    }
    if ((yaw_angle_ref >= 170) && (yaw_angle_ref <= -170))
    {
        VAL_LIMIT(yaw_angle_ref, -170, 170);
    }
    last_p=data_recv.pitchAngleSet;
    last_y=data_recv.yawAngleSet;
    //计算pitch轴相对角度差
    pit_angle_fdb = pit_relative_angle;
    //计算yaw轴相对角度差
    yaw_angle_fdb = yaw_relative_angle;
}

