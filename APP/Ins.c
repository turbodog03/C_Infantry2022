//
// Created by HLiamso on 2022-01-16.ins_task
//
#include "Ins.h"
#include "QuaternionAHRS.h"
#include "BMI088driver.h"
#include "bsp_dwt.h"
#include "cmsis_os.h"

uint32_t time=0;
float dt=0;
float EulerAngle[3]={0};
void ins_task() {
    /* USER CODE BEGIN GetAttitudeTask */
    Quaternion_AHRS_InitIMU(BMI088.Accel[0],BMI088.Accel[1],BMI088.Accel[2],BMI088.gNorm);
    /* Infinite loop */
    uint32_t task_wake_time = osKernelSysTick();
    for (;;) {
        BMI088_Read(&BMI088);
        dt=DWT_GetDeltaT(&time);
        Quaternion_AHRS_UpdateIMU(BMI088.Gyro[0],BMI088.Gyro[1],BMI088.Gyro[2],BMI088.Accel[0],BMI088.Accel[1],BMI088.Accel[2],0,0,0,dt);
        Get_EulerAngle(AHRS.q);
        EulerAngle[0]=-AHRS.Pitch;
        EulerAngle[1]=-AHRS.Roll;
        EulerAngle[2]=-AHRS.Yaw;
        vTaskDelayUntil(&task_wake_time, 1);
    }
    /* USER CODE END GetAttitudeTask */
}