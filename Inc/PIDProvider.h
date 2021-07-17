/**
 * @brief PID 参数
 */
struct PIDParam
{
    double P;
    double I;
    double D;
};
struct PIDProvider
{
    PIDParam param;
    /**
     * @brief 上一次误差值
     */
    double err;
    /**
     * @brief 误差积分值
     */
    double integral;
    /**
     * @brief 误差微分值
     */
    double diff;
};

double pid_push_new_err(PIDProvider provider, double new_err)
{
    provider.integral += new_err;
    provider.diff = new_err - provider.err;
    provider.err = new_err;
    return provider.param.P * new_err + provider.param.I * provider.integral + provider.param.D * provider.diff;
}
