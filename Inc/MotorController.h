#include <stdlib.h>
#include "Motor.h"

#define LOG_TAG "MotorController"

#include <elog.h>

#define min(a, b) ((a < b) ? a : b)
#define max(a, b) ((a > b) ? a : b)
#define clamp(vl, v, vh) max(vl, min(v, vh))

typedef float Energy;

/**
 * @brief 风力摆空心杯电机控制器
 * 根据传入的 Duty 值控制左右两个电机的  Duty 以及 正反转
 */
struct MotorController
{
    /**
     * @brief 唯一识别 Id
     * 在 log 中输出
     */
    uint8_t Id;
    struct Motor left;
    struct Motor right;
};

/**
 * @brief 根据 Energy 设定左右电机的 Duty 和 正反转
 * 当 Energy 为正时， 代表我们需要更多动能，所以应当在角速度方向加速， 反之减速
 * 
 * @param energy 动能差，+ 代表增加动能 - 代表减少动能
 * @param omega 当前速度
 */
void motor_ctl_update_energy(struct MotorController *motor_ctl, Energy energy, float omega)
{
    log_i("[%d]Update Energy:%f, Omega:%f", motor_ctl->Id, energy, omega);
    int duty = (Duty)min(abs(energy), 36000);

    if (abs(omega) < 0.01 || abs(energy) < 600)
    {
        motor_doOp(&motor_ctl->left, Free, 0);
        motor_doOp(&motor_ctl->right, Free, 0);
    }

    if (signbit(energy) ^ signbit(omega))
    {
        log_i("[%d]逆时针用力");
        motor_doOp(&motor_ctl->left, Forward, duty);
        motor_doOp(&motor_ctl->right, Brake, duty);
    }
    else
    {
        log_i("[%d]顺时针用力");
        motor_doOp(&motor_ctl->left, Brake, duty);
        motor_doOp(&motor_ctl->right, Forward, duty);
    }
}
