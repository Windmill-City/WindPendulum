#include "tim.h"

/**
 * @brief PWM 控制器
 */
class PWMGenerater
{
public:
    typedef int Duty;
    typedef volatile uint32_t CCRx;

private:
    TIM_HandleTypeDef *tim;
    /**
     * @brief 比较值
     * 决定占空比
     */
    CCRx &ccr;

public:
    PWMGenerater(TIM_HandleTypeDef *tim, CCRx &ccr) : ccr(ccr) {}

    /**
     * @brief 设置生成PWM波的占空比
     * 
     * @param duty 占空比
     */
    void setDuty(Duty duty)
    {
        ccr = duty;
    }

    /**
     * @brief 返回PWM波当前占空比
     * 
     * @return 占空比
     */
    Duty getDuty()
    {
        return ccr;
    }
};
