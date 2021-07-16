/**
 * @brief PID Provider
 */
template <typename T>
class PIDProvider
{
private:
    /**
     * @brief 上一次误差值
     */
    T err;
    /**
     * @brief 误差积分值
     */
    T integral;
    /**
     * @brief 误差微分值
     */
    T diff;

public:
    struct Param
    {
        double P;
        double I;
        double D;
    };
    Param param;

    /**
     * @brief Construct a new PIDProvider object
     * 
     * @param initialParam 初始PID参数
     */
    PIDProvider(Param initialParam)
    {
        this->param = initialParam;
    }

    /**
     * @brief 置入新的误差值并返回控制值
     * 
     * @param err 新误差
     * @return 控制值
     */
    T pushNewErr(T err)
    {
        this->integral += err;
        this->diff = err - this->err;
        this->err = err;
        return param.P * err + param.I * integral + param.D * diff;
    }
};
