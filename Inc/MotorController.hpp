#include <math.h>
#include "PIDProvider.hpp"
#include "Motor.hpp"

/**
 * @brief 风力摆空心杯电机控制器
 * 根据传入的 Duty 值控制左右两个电机的  Duty 以及 正反转
 */
class MotorController
{
private:
    Motor *left;
    Motor *right;

public:
    MotorController(Motor *left, Motor *right)
    {
        this->left = left;
        this->right = right;
    }

    /**
     * @brief 根据 Duty 设定左右电机的 Duty 和 正反转
     * 当 Duty 为正时， 代表我们需要更多动能，所以应当在角速度方向加速， 反之减速
     * 
     * @param duty 动能差，+ 代表增加动能 - 代表减少动能
     * @param omega 当前速度
     */
    void updateMotorDuty(PWMGenerater::Duty duty, double omega)
    {
        if (signbit(omega) ^ signbit(duty))
        {
            duty = abs(duty);
            //顺时针加速
            left->setMode(Motor::Backward);
            left->setDuty(duty);
            right->setMode(Motor::Forward);
            right->setDuty(duty);
        }
        else
        {
            duty = abs(duty);
            //逆时针加速
            left->setMode(Motor::Forward);
            left->setDuty(duty);
            right->setMode(Motor::Backward);
            right->setDuty(duty);
        }
    }
};
