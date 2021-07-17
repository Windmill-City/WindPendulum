#include "PWMGenerater.h"
#include <elog.h>

#define LOG_TAG "Motor"

#define min(a, b) ((a < b) ? a : b)
#define max(a, b) ((a > b) ? a : b)
#define clamp(vl, v, vh) max(vl, min(v, vh))

/**
 * @brief 电机模式
 */
typedef uint8_t MotorMode;

#define Free ((MotorMode)0x00)
#define Brake ((MotorMode)0x01)
#define Forward ((MotorMode)0x02)
#define Backward ((MotorMode)0x03)

struct Motor
{
    /**
     * @brief 唯一识别 Id
     * 在 log 中输出
     */
    uint8_t Id;
    /**
     * @brief 电机驱动控制引脚
     */
    GPIO_TypeDef *Port_IN1;
    uint16_t Pin_IN1;
    GPIO_TypeDef *Port_IN2;
    uint16_t Pin_IN2;

    MotorMode mode;
    struct PWMGenerater pwmGenerater;
};

/**
 * @brief 设置电机正反转、停机、自由转动
 * 
 * @param mode 正反转、停机、自由转动
 * @param port1 IN1_Port
 * @param pin1 IN1_Pin
 * @param port2 IN2_Port
 * @param pin2 IN2_Pin
 */
void set_motor_mode(struct Motor motor, MotorMode mode)
{
    log_i("[%d]Set Mode: Old Mode:%d, New Mode:%d", motor.Id, motor.mode, mode);
    auto port1 = motor.Port_IN1;
    auto pin1 = motor.Pin_IN1;
    auto port2 = motor.Port_IN2;
    auto pin2 = motor.Pin_IN2;

    switch (mode)
    {
    case Free:
        HAL_GPIO_WritePin(port1, pin1, GPIO_PIN_RESET);
        HAL_GPIO_WritePin(port2, pin2, GPIO_PIN_RESET);
        break;
    case Brake:
        HAL_GPIO_WritePin(port1, pin1, GPIO_PIN_SET);
        HAL_GPIO_WritePin(port2, pin2, GPIO_PIN_SET);
        break;
    case Forward:
        HAL_GPIO_WritePin(port1, pin1, GPIO_PIN_SET);
        HAL_GPIO_WritePin(port2, pin2, GPIO_PIN_RESET);
        break;
    case Backward:
        HAL_GPIO_WritePin(port1, pin1, GPIO_PIN_RESET);
        HAL_GPIO_WritePin(port2, pin2, GPIO_PIN_SET);
        break;
    }
}

/**
 * @brief 操作电机
 * 
 * @param motor 要操作的电机
 * @param mode 正反转\刹车\自由转动
 * @param duty 电机 PWM 占空比
 */
void motor_doOp(struct Motor motor, MotorMode mode, Duty duty)
{
    log_i("[%d]Do operation: Mode:%d Duty:%d", motor.Id, mode, duty);
    set_motor_mode(motor, mode);
    set_pwm_duty(motor.pwmGenerater, duty);
}
