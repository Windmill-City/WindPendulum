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
#define R 0.60

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

struct Attribute fetchAttr(struct MPU6050 data_)
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

void load_straight_line_pid()
{
    pid_reset_all(&pidXZ);
    pid_reset_all(&pidYZ);

    struct PIDParam pXZ = {5000, 0, 0, 500};
    struct PIDParam pYZ = {5000, 0, 0, 500};

    pidXZ.param = pXZ;
    pidYZ.param = pYZ;
}

/**
 * @brief 初始化风力摆
 */
void wind_pendulum_init()
{
    log_i("Begin init");
    mpu6050_init(20, true);

    HAL_Delay(5000);

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
    motorYZ.Id = 3;
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

    log_i("Initialize finished");
    load_straight_line_pid();
}

#define ToRad(x) (x / 57.3f)
#define ToAngle(x) (x * 57.3f)

/**
 * @brief 根据时间计算理想状态下摆角
 * 
 * @param 最大摆角, 角度
 * @param time 时间, 毫秒
 * @param phase 相位差, 弧度
 * @return float 摆角, 角度
 */
float getAngleByTime(float maxAngle, uint32_t time, float phase)
{
    return ToAngle(ToRad(maxAngle) * cos(sqrt(g / R) * time / 1000.0f + phase));
}

/**
 * @brief 通过摆杆当前角度算理想单摆当前角度下的角速度
 * 
 * @param angle 摆杆当前角度
 * @param phi 最大摆角
 * @param gravity 重力加速度
 * @param radius 摆杆长度
 * @return double 理想状态角速度
 */
float getOmegaByAngle(float angle, float phi)
{
    return sqrt(2 * g * (cos(ToRad(phi)) - cos(ToRad(angle))) / R);
}

void update_loop()
{
    struct MPU6050 data;
    if (fifo_read(&data))
    {
        struct Attribute attr = fetchAttr(data);

        float expectAngle = getAngleByTime(30, HAL_GetTick(), ToAngle(90));
        float expectOmega = expectAngle > 0 ? getOmegaByAngle(30, expectAngle) : -getOmegaByAngle(30, expectAngle);

        float errAngle = expectAngle - attr.euler.Roll;
        float errOmega = expectOmega - attr.omegaTheta;

        log_i("Angle Current:%f Expect:%f, Err:%f", attr.euler.Roll, expectAngle, errAngle);
        log_i("Omega Current:%f Expect:%f, Err:%f", attr.omegaTheta, expectOmega, errOmega);

        float energy = pid_push_new_err(&pidXZ, errOmega);
        motor_ctl_update_energy(&motorXZ, energy, attr.omegaTheta);
    }
    HAL_Delay(10);
}