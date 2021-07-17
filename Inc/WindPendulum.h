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
#include <elog.h>

#define LOG_TAG "WindPendulum"

//Q16 定点数转浮点数
#define ToFloat(fixed, count) ((float)fixed / (float)pow(2, 30))

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

    auto q0 = ToFloat(data_.quat[0], 30);
    auto q1 = ToFloat(data_.quat[1], 30);
    auto q2 = ToFloat(data_.quat[2], 30);
    auto q3 = ToFloat(data_.quat[3], 30);

    //欧拉角
    inv_get_sensor_type_euler(data, &accuracy, (inv_time_t *)&timestamp);

    attr.euler.Pitch = asin(-2 * q1 * q3 + 2 * q0 * q2) * 57.3f;
    attr.euler.Roll = atan2(2 * q2 * q3 + 2 * q0 * q1, -2 * q1 * q1 - 2 * q2 * q2 + 1) * 57.3f;
    attr.euler.Yaw = atan2(2 * (q1 * q2 + q0 * q3), q0 * q0 + q1 * q1 - q2 * q2 - q3 * q3) * 57.3f;

    // attr.euler.Pitch = ToFloat(data[0], 16);
    // attr.euler.Roll = ToFloat(data[1], 16);
    // attr.euler.Yaw = ToFloat(data[2], 16);

    log_i("Euler: %d %d %d", (int)(attr.euler.Pitch * 100), (int)(attr.euler.Roll*100), (int)(attr.euler.Yaw*100));

    return attr;
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
double getOmegaByAngle(double angle, double phi)
{
    return sqrt(2 * g * (cos(angle) - cos(phi)) / R);
}

/**
 * @brief XZ/YZ 方向上的电机控制器
 */
struct MotorController motorXZ;
struct MotorController motorYZ;

/**
 * @brief 初始化风力摆
 */
void wind_pendulum_init()
{
    log_i("Begin init");
    mpu6050_init(20, true);

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

    log_i("Initialize finished");
}