/**
 * @file Controller.h
 * @author Windmill_City
 * @brief 理想单摆在特定角度时的角速度与系统当前角速度构成PID参数
 * @version 0.1
 * @date 2021-07-14
 * 
 * @copyright Copyright (c) 2021
 */
#include "MotorController.hpp"
#include "MPU6050.hpp"

/**
 * @brief 重力加速度
 */
#define g 9.7833
/**
 * @brief 单摆半径
 */
#define R 0.60

/**
 * @brief 旋转矩阵, 根据你的 MPU 在设备中方位来设定
 */
static struct MPU6050::platform_data_s gyro_pdata = {
    .orientation = {0, 0, 1,
                    0, 1, 0,
                    -1, 0, 0}};

/**
 * @brief 风力摆控制器
 */
class WindPendulum
{
private:
    MPU6050 mpu = MPU6050(20, gyro_pdata, true);

public:
    /**
     * @brief 风力摆姿态解算器
     */
    class AttributeProvider
    {
    public:
        struct Attribute
        {
            /**
             * @brief 欧拉角
             */
            Euler euler;
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

//Q16 定点数转浮点数
#define ToFloat(fixed) (float)(fixed * (1 / 1 << 16))

        Attribute fetchAttr()
        {
            long data[9];
            int8_t accuracy;
            unsigned long timestamp;

            Attribute attr;

            //欧拉角
            inv_get_sensor_type_euler(data, &accuracy, (inv_time_t *)&timestamp);
            attr.euler = {
                .Pitch = ToFloat(data[0]),
                .Roll = ToFloat(data[1]),
                .Yaw = ToFloat(data[2])};
            //角速度
            inv_get_sensor_type_gyro(data, &accuracy, (inv_time_t *)&timestamp);
            attr.omegaTheta = ToFloat(data[0]);
            attr.omegaPhi = ToFloat(data[1]);
            //角加速度
            inv_get_sensor_type_accel(data, &accuracy, (inv_time_t *)&timestamp);
            attr.accOmegaTheta = ToFloat(data[0]);
            attr.accOmegaPhi = ToFloat(data[1]);
            return attr;
        }
    };

    struct Config
    {
        /**
         * @brief 当地重力加速度
         */
        double gravity;

        /**
         * @brief 风力摆摆长
         */
        double radius;
    };

    AttributeProvider *provider;
    Config config;

    /**
     * @brief XZ、YZ平面电机控制器
     */
    MotorController *motorXZ;
    MotorController *motorYZ;

    /**
     * @brief Construct a new Wind Pendulum object
     * 
     * @param motorXZ XZ平面电机控制器
     * @param motorYZ YZ平面电机控制器
     * @param provider 姿态解算器
     */
    WindPendulum(
        MotorController *motorXZ,
        MotorController *motorYZ,
        Config config,
        AttributeProvider *provider)
    {
        this->config = config;
        this->provider = provider;
        this->motorXZ = motorXZ;
        this->motorYZ = motorYZ;
    }

    PIDProvider<double> pidYZ = PIDProvider<double>({1000,10,10});

    /**
     * @brief 风力摆控制器运作循环
     */
#pragma GCC push_options
#pragma GCC optimize("O0")
    void updateLoop()
    {
        //有新数据
        if (mpu.updateLoop())
        {
            auto cur = provider->fetchAttr();
            auto expect = getOmegaByAngle(cur.euler.Pitch, 60, config.gravity, config.radius);
            auto errOmega = cur.omegaTheta - expect;
            motorYZ->updateMotorDuty(pidYZ.pushNewErr(errOmega), cur.omegaTheta);
        }
    }
#pragma GCC pop_options

private:
    /**
     * @brief 通过摆杆当前角度算理想单摆当前角度下的角速度
     * 
     * @param angle 摆杆当前角度
     * @param phi 最大摆角
     * @param gravity 重力加速度
     * @param radius 摆杆长度
     * @return double 理想状态角速度
     */
    double getOmegaByAngle(double angle, double phi, double gravity, double radius)
    {
        return sqrt(2 * gravity * (cos(angle) - cos(phi)) / radius);
    }
};
