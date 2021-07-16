#include <functional>
#include "PWMGenerater.hpp"

#define min(a, b) ((a < b) ? a : b)
#define max(a, b) ((a > b) ? a : b)
#define clamp(vl, v, vh) max(vl, min(v, vh))

class Motor
{
public:
    enum Mode
    {
        /**
         * @brief 自由转动
         */
        Free,
        /**
         * @brief 刹车
         */
        Brake,
        /**
         * @brief 正转
         */
        Forward,
        /**
         * @brief 反转
         */
        Backward
    };

    enum State
    {
        Stopped,
        Starting,
        Running,
        Stopping
    };

private:
    /**
     * @brief 起转\停转完毕的时间
     */
    uint32_t actionCompleteTime = -1;

    PWMGenerater *pwmGenerater;

    Mode mode;
    State state = Stopped;

public:
    struct Config
    {
        /**
         * @brief 电机最小启动时间, 单位 ms
         */
        uint32_t minStartupTime;
        /**
         * @brief 电机最小停转时间, 单位 ms
         */
        uint32_t minStopTime;
        /**
         * @brief 电机保持运动最小占空比
         */
        PWMGenerater::Duty minDuty;
        /**
         * @brief 电机从静止开始启动需要的最小占空比
         */
        PWMGenerater::Duty minStartupDuty;
        /**
         * @brief 电机从静止开始启动不堵转的最大占空比
         */
        PWMGenerater::Duty maxStartupDuty;
        /**
         * @brief 电机最大占空比
         */
        PWMGenerater::Duty maxDuty;
    };

    typedef std::function<void(Mode)> ModeSetter;
    typedef std::function<Mode(void)> ModeGetter;

    Config config;
    ModeSetter modeSetter;
    ModeGetter modeGetter;

    Motor(PWMGenerater *pwmGenerater, Config config, ModeSetter setter, ModeGetter getter)
    {
        this->pwmGenerater = pwmGenerater;
        this->config = config;
        this->modeSetter = setter;
        this->modeGetter = getter;
    }

    /**
     * @brief 设置电机的占空比
     * 控制占空比在电机的安全范围内
     * @param duty 占空比
     */
    void setDuty(PWMGenerater::Duty duty)
    {
        switch (state)
        {
        case Stopped:
        Stopped:
            //小于保持 Duty, 保持停机
            if (duty < config.minDuty)
                break;
            //大于等于保持 Duty, 启动电机
            state = Starting;
            actionCompleteTime = HAL_GetTick() + config.minStartupTime;
        case Starting:
            //启动完成
            if (HAL_GetTick() > actionCompleteTime)
            {
                state = Running;
                goto Running;
            }
            //起转 Duty 范围
            pwmGenerater->setDuty(clamp(config.minStartupDuty, duty, config.maxStartupDuty));
            break;
        case Running:
        Running:
            //小于最小保持 Duty, 停止电机
            if (duty < config.minDuty)
            {
                actionCompleteTime = HAL_GetTick() + config.minStopTime;
                state = Stopping;
                goto Stopping;
            }
            //正常运行 Duty 不超过最大值
            pwmGenerater->setDuty(min(duty, config.maxDuty));
            break;
        case Stopping:
            //停机完毕
            if (HAL_GetTick() > actionCompleteTime)
            {
                state = Stopped;
                goto Stopped;
            }
        Stopping:
            pwmGenerater->setDuty(0);
            break;
        }
    }

    PWMGenerater::Duty getDuty()
    {
        return pwmGenerater->getDuty();
    }

    /**
     * @brief 设置电机正反转、刹车、自由转动
     * 
     * @param mode 模式
     */
    void setMode(Mode mode)
    {
        modeSetter(mode);
    }

    Mode getMode()
    {
        return modeGetter();
    }

    State getState()
    {
        return state;
    }
};