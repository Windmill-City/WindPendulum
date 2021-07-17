#include <elog.h>
#include "tim.h"

#define LOG_TAG "PWM Generater"

typedef volatile uint32_t Duty;

/**
 * @brief PWM生成器
 */
struct PWMGenerater
{
    /**
     * @brief 唯一识别 Id
     * 在 log 中输出
     */
    uint8_t Id;
    /**
     * @brief 用于设置 占空比 的地址
     */
    Duty *duty;
};

/**
 * @brief 设置指定 PWM 生成器 的 占空比
 * 
 * @param pwm PWM生成器
 * @param duty 占空比
 */
void set_pwm_duty(struct PWMGenerater pwm, Duty duty)
{
    log_i("[%d]Old Duty:%d, New Duty:%d", pwm.Id, pwm.duty, duty);
    *pwm.duty = duty;
}
