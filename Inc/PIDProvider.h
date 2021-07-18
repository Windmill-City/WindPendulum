#define LOG_TAG "PIDProvider"
#include <elog.h>

#define min(a, b) ((a < b) ? a : b)
#define max(a, b) ((a > b) ? a : b)
#define clamp(vl, v, vh) max(vl, min(v, vh))

/**
 * @brief PID 参数
 */
struct PIDParam
{
    float P;
    float I;
    float D;
    float maxIntegral;
};
struct PIDProvider
{
    /**
     * @brief 唯一识别 Id
     * 在 log 中输出
     */
    uint8_t Id;
    struct PIDParam param;
    /**
     * @brief 上一次误差值
     */
    float err;
    /**
     * @brief 误差积分值
     */
    float integral;
    /**
     * @brief 误差微分值
     */
    float diff;
};

float pid_push_new_err(struct PIDProvider* provider, float new_err)
{
    provider->integral += new_err;
    
    provider->integral = clamp(-provider->param.maxIntegral, provider->integral, provider->param.maxIntegral);

    provider->diff = new_err - provider->err;
    provider->err = new_err;
    log_i("[%d]Err:%f, Integral:%f, Diff:%f", provider->Id, provider->err, provider->integral, provider->diff);
    return provider->param.P * new_err + provider->param.I * provider->integral + provider->param.D * provider->diff;
}

void pid_reset_all(struct PIDProvider* provider)
{
    provider->integral = 0;
    provider->diff = 0;
    provider->err = 0;
    log_i("[%d]pid reset all", provider->Id);
}
