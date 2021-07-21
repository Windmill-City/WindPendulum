/**
 * @file Controller.h
 * @author Windmill_City
 * @brief 理想单摆在特定角度时的角速度与系统当前角速度构成PID参数
 * @version 0.1
 * @date 2021-07-14
 * 
 * @copyright Copyright (c) 2021
 */
#include "MotorController.h"
#include "MPU6050.h"
#include "PIDProvider.h"
#include "FireTools/PIDTuner.h"

#define LOG_TAG "WindPendulum"
#include <elog.h>

//Q16 定点数转浮点数
#define ToFloat(fixed, count) (((float)fixed) / pow(2, count))

/**
 * @brief 重力加速度
 */
#define g 9.7833
/**
 * @brief 单摆半径
 */
#define R 0.84

struct Attribute
{
    /**
     * @brief 欧拉角
     */
    struct Euler euler;
    /**
     * @brief 角速度
     */
    float omegaTheta;
    float omegaPhi;
    /**
     * @brief 角加速度
     */
    float accOmegaTheta;
    float accOmegaPhi;
};

struct Attribute fetchAttr()
{
    long data[9];
    int8_t accuracy;
    unsigned long timestamp;

    struct Attribute attr;

    //欧拉角
    inv_get_sensor_type_euler(data, &accuracy, (inv_time_t *)&timestamp);

    attr.euler.Pitch = ToFloat(data[0], 16);
    attr.euler.Roll = ToFloat(data[1], 16);
    attr.euler.Yaw = ToFloat(data[2], 16);

    log_i("Euler: Pitch:%f Roll:%f Yaw:%f", attr.euler.Pitch, attr.euler.Roll, attr.euler.Yaw);

    //角速度
    inv_get_sensor_type_gyro(data, &accuracy, (inv_time_t *)&timestamp);

    attr.omegaTheta = ToFloat(data[0], 16);
    attr.omegaPhi = ToFloat(data[1], 16);

    log_i("Omega: %f %f %f", attr.omegaTheta, attr.omegaPhi, ToFloat(data[2], 16));
    //角加速度
    inv_get_sensor_type_accel(data, &accuracy, (inv_time_t *)&timestamp);

    attr.accOmegaTheta = ToFloat(data[0], 16);
    attr.accOmegaPhi = ToFloat(data[1], 16);

    log_i("Acc: %f %f %f", attr.accOmegaTheta, attr.accOmegaPhi, ToFloat(data[2], 16));

    return attr;
}

/**
 * @brief XZ/YZ 方向上的电机控制器
 */
struct MotorController motorXZ;
struct MotorController motorYZ;

struct PIDProvider pidXZ;
struct PIDProvider pidYZ;

//目标角度
int targetAngleXZ;
int targetAngleYZ;
//相位差
int targetPhaseXZ;
int targetPhaseYZ;

#define PI acos(-1)

#define ToRad(x) (x / 57.3f)
#define ToAngle(x) (x * 57.3f)

void load_straight_line_pid()
{
    pid_reset_all(&pidXZ);
    pid_reset_all(&pidYZ);

    struct PIDParam pXZ = {0, 0, 0, 180};
    struct PIDParam pYZ = {0, 0, 0, 180};

    pidXZ.param = pXZ;
    pidYZ.param = pYZ;

    targetAngleXZ = 0; //atan( 0.25 / R ) * 180 / 3.14 ;//0;
    targetAngleYZ = ToAngle(atan(0.3 / R));

    targetPhaseXZ = 0;
    targetPhaseYZ = 0;

    sync_pid(CH1, pidXZ.param.P, pidXZ.param.I, pidXZ.param.D);
    sync_pid(CH4, pidYZ.param.P, pidYZ.param.I, pidYZ.param.D);
}

void PacketHandler(pPacketBase packet, uint8_t *data, size_t len)
{
    float *pid = (float *)data;
    switch (packet->cmd)
    {
    case CS_SET_PID:
        switch (packet->addr)
        {
        case CH1:
            pidXZ.param.P = pid[0];
            pidXZ.param.I = pid[1];
            pidXZ.param.D = pid[2];
            break;
        case CH4:
            pidYZ.param.P = pid[0];
            pidYZ.param.I = pid[1];
            pidYZ.param.D = pid[2];
            break;
        }
        pid_reset_all(&pidXZ);
        pid_reset_all(&pidYZ);
        motor_ctl_update_energy(&motorXZ, 0, 0);
        motor_ctl_update_energy(&motorYZ, 0, 0);
        while (true)
        {
            HAL_IWDG_Refresh(&hiwdg);
            struct Attribute attr = fetchAttr();
            if (abs(attr.omegaPhi) < 0.01 && abs(attr.omegaTheta) < 0.01)
                break;
        }
        break;
    case CS_SET_TARGET:
        switch (packet->addr)
        {
        case CH1:
            targetAngleXZ = *((int32_t *)data);
            break;
        case CH2:
            targetAngleXZ = *((int32_t *)data);
            break;
        case CH3:
            targetPhaseXZ = *((int32_t *)data);
            break;
        case CH4:
            targetPhaseYZ = *((int32_t *)data);
            break;
        }
        break;
    case CS_CM_START:
        switch (packet->addr)
        {
        case CH1:
            gAddressFilter = CH1;
            break;
        case CH2:
            gAddressFilter = CH2;
            break;
        case CH3:
            gAddressFilter = CH3;
            break;
        case CH4:
            gAddressFilter = CH4;
            break;
        case CH5:
            gAddressFilter = CH5;
            break;
        }
        break;
    case CS_CM_STOP:
        switch (packet->addr)
        {
        case CH1:
        case CH2:
        case CH3:
        case CH4:
        case CH5:
            gAddressFilter = 0;
            break;
        }
        break;
    case CS_CM_RESET:
        NVIC_SystemReset();
        break;
    case CS_SET_PEIRIOD:
        break;
    }
}

/**
 * @brief 初始化风力摆
 */
void wind_pendulum_init()
{
    mpu6050_init(20, true);
    mpu_return_zero();

    //XZ平面
    motorXZ.Id = 1;
    motorXZ.left.Id = 1;
    motorXZ.left.Port_IN1 = XL_IN1_GPIO_Port;
    motorXZ.left.Pin_IN1 = XL_IN1_Pin;
    motorXZ.left.Port_IN2 = XL_IN2_GPIO_Port;
    motorXZ.left.Pin_IN2 = XL_IN2_Pin;
    motorXZ.left.pwmGenerater.Id = 1;
    motorXZ.left.pwmGenerater.duty = &TIM1->CCR1;

    motorXZ.right.Id = 2;
    motorXZ.right.Port_IN1 = XR_IN1_GPIO_Port;
    motorXZ.right.Pin_IN1 = XR_IN1_Pin;
    motorXZ.right.Port_IN2 = XR_IN2_GPIO_Port;
    motorXZ.right.Pin_IN2 = XR_IN2_Pin;
    motorXZ.right.pwmGenerater.Id = 2;
    motorXZ.right.pwmGenerater.duty = &TIM1->CCR2;

    //YZ平面
    motorYZ.Id = 2;
    motorYZ.left.Id = 3;
    motorYZ.left.Port_IN1 = YL_IN1_GPIO_Port;
    motorYZ.left.Pin_IN1 = YL_IN1_Pin;
    motorYZ.left.Port_IN2 = YL_IN2_GPIO_Port;
    motorYZ.left.Pin_IN2 = YL_IN2_Pin;
    motorYZ.left.pwmGenerater.Id = 3;
    motorYZ.left.pwmGenerater.duty = &TIM1->CCR3;

    motorYZ.right.Id = 4;
    motorYZ.right.Port_IN1 = YR_IN1_GPIO_Port;
    motorYZ.right.Pin_IN1 = YR_IN1_Pin;
    motorYZ.right.Port_IN2 = YR_IN2_GPIO_Port;
    motorYZ.right.Pin_IN2 = YR_IN2_Pin;
    motorYZ.right.pwmGenerater.Id = 4;
    motorYZ.right.pwmGenerater.duty = &TIM1->CCR4;

    pidXZ.Id = 1;
    pidYZ.Id = 2;

    //初始化 PID调参器
    init_firetools_server(PacketHandler);

    load_straight_line_pid();
}

/**
 * @brief 单摆运动周期
 */
#define T (2 * PI * sqrt(R / g))

/**
 * @brief 单摆运动角频率
 */
#define Omega (2 * PI / T)

/**
 * @brief 根据时间计算理想状态下摆角
 * 
 * @param time 时间, 毫秒
 * @param maxAngle 最大摆角, 角度
 * @param phase 相位差, 角度
 * @return float 摆角, 角度
 */
float getAngleByTime(uint32_t time, float maxAngle, float phase)
{
    return ToAngle(ToRad(maxAngle) * cos(Omega * time / 1000.0f + ToRad(phase)));
}

/**
 * @brief 通过时间算理想角速度
 * 
 * @param time 时间, 毫秒
 * @param maxAngle 最大摆角
 * @param phase 相位差, 弧度
 * @return float 理想状态角速度
 */
float getOmegaByTime(uint32_t time, float maxAngle, float phase)
{
    return -ToRad(maxAngle) * Omega * sin(Omega * time / 1000.0f + phase);
}

void update_motor_state()
{
    struct Attribute attr = fetchAttr();

    float expectAngleXZ = getAngleByTime(HAL_GetTick(), targetAngleXZ, targetPhaseXZ);
    float expectOmegaXZ = getOmegaByTime(HAL_GetTick(), targetAngleXZ, targetPhaseXZ);

    float errAngleXZ = expectAngleXZ - attr.euler.Roll;
    float errOmegaXZ = attr.omegaTheta - expectOmegaXZ;

    log_i("Angle Current:%f Expect:%f, Err:%f", attr.euler.Roll, expectAngleXZ, errAngleXZ);
    log_i("Omega Current:%f Expect:%f, Err:%f", attr.omegaTheta, expectOmegaXZ, errOmegaXZ);

    send_actual(CH1, attr.euler.Roll);
    send_target(CH1, expectAngleXZ);

    float energyXZ = pid_push_new_err(&pidXZ, errAngleXZ);
    send_target(CH2, pidXZ.integral);
    send_actual(CH2, pidXZ.err);
    send_actual(CH3, energyXZ / 360);
    motor_ctl_update_energy(&motorXZ, energyXZ, attr.omegaTheta);

    float expectAngleYZ = getAngleByTime(HAL_GetTick(), targetAngleYZ, targetPhaseYZ);
    float expectOmegaYZ = getOmegaByTime(HAL_GetTick(), targetAngleYZ, targetPhaseYZ);

    float errAngleYZ = expectAngleYZ - attr.euler.Pitch;
    float errOmegaYZ = attr.omegaTheta - expectOmegaYZ;

    log_i("Angle Current:%f Expect:%f, Err:%f", attr.euler.Pitch, expectAngleYZ, errAngleYZ);
    log_i("Omega Current:%f Expect:%f, Err:%f", attr.omegaPhi, expectOmegaYZ, errOmegaYZ);

    send_actual(CH4, attr.euler.Pitch);
    send_target(CH4, expectAngleYZ);

    float energyYZ = pid_push_new_err(&pidYZ, errAngleYZ);
    //send_actual(CH3, pidYZ.integral);
    motor_ctl_update_energy(&motorYZ, energyYZ, attr.omegaPhi);

    HAL_Delay(5);
}