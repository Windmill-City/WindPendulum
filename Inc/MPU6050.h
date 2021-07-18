#include "inv_mpu.h"
#include "inv_mpu_dmp_motion_driver.h"
#include "invensense.h"
#include "invensense_adv.h"
#include "eMPL_outputs.h"

#define LOG_TAG "MPU6050"
#include <elog.h>

unsigned char *mpl_key = (unsigned char *)"eMPL 5.1";

struct rx_s
{
    unsigned char header[3];
    unsigned char cmd;
};

struct hal_s
{
    unsigned char lp_accel_mode;
    unsigned char sensors;
    unsigned char dmp_on;
    unsigned char wait_for_tap;
    volatile unsigned char new_gyro;
    unsigned char motion_int_mode;
    unsigned long no_dmp_hz;
    unsigned long next_pedo_ms;
    unsigned long next_temp_ms;
    unsigned long next_compass_ms;
    unsigned int report;
    unsigned short dmp_features;
    struct rx_s rx;
};

#define ACCEL_ON (0x01)
#define GYRO_ON (0x02)
#define COMPASS_ON (0x04)

struct Euler
{
    float Pitch, Roll, Yaw;
};

/**
 * @brief DMP 轻敲事件回调 Tap Callback
 * 
 * @param direction 敲击面
 * @param count 敲击次数
 */
static void tap_cb(unsigned char direction, unsigned char count)
{
    switch (direction)
    {
    case TAP_X_UP:
        MPL_LOGI("Tap X+ ");
        break;
    case TAP_X_DOWN:
        MPL_LOGI("Tap X- ");
        break;
    case TAP_Y_UP:
        MPL_LOGI("Tap Y+ ");
        break;
    case TAP_Y_DOWN:
        MPL_LOGI("Tap Y- ");
        break;
    case TAP_Z_UP:
        MPL_LOGI("Tap Z+ ");
        break;
    case TAP_Z_DOWN:
        MPL_LOGI("Tap Z- ");
        break;
    default:
        return;
    }
    MPL_LOGI("x%d\n", count);
    return;
}

/**
 * @brief DMP 设备方向回调
 * 
 * @param orientation 当前方向
 */
static void android_orient_cb(unsigned char orientation)
{
    switch (orientation)
    {
    case ANDROID_ORIENT_PORTRAIT:
        MPL_LOGI("Portrait\n");
        break;
    case ANDROID_ORIENT_LANDSCAPE:
        MPL_LOGI("Landscape\n");
        break;
    case ANDROID_ORIENT_REVERSE_PORTRAIT:
        MPL_LOGI("Reverse Portrait\n");
        break;
    case ANDROID_ORIENT_REVERSE_LANDSCAPE:
        MPL_LOGI("Reverse Landscape\n");
        break;
    default:
        return;
    }
}

static volatile unsigned char rx_new;
static struct hal_s hal = {0};

/**
 * @brief 初始化 MPL
 */
static inv_error_t mpl_init()
{
    inv_error_t result;

    /**
     * @brief 初始化 MPL 库
     */
    result = inv_init_mpl();
    if (result)
    {
        MPL_LOGE("Could not initialize MPL.\n");
    }

    /**
     * @brief 启用 MPL 库计算 6 轴四元数功能
     */
    result = inv_enable_quaternion();

    /**
     * @brief 启用 MPL 融合 6 + 3(磁场) 轴四元数功能
     * 
     * MPU6050 无磁场计
     */
    //result = inv_enable_9x_sensor_fusion();

    /**
     * @brief 启用 MPL 库磁场向量计算功能
     */
    //inv_enable_vector_compass_cal();

    /**
     * @brief 启用 MPL 磁场去干扰功能
     * 
     * 在磁场计检测到干扰时, 四元数计算自动切换至 6 轴模式
     * 然后每 5s 检测一次磁场, 如果磁场恢复正常, 则自动切换回 9 轴模式
     */
    //inv_enable_magnetic_disturbance();

    /**
     * @brief 启用 MPL 在陀螺仪静止时校准误差功能
     * inv_enable_fast_nomot() 设定为 0.5s 无运动进行校准
     * 
     * 如果需要自定义无运动时间, 删除 inv_enable_fast_nomot()
     * 添加:
     * inv_enable_motion_no_motion();
     * inv_set_no_motion_time(1000); 1000 <- 无运动时间
     */
    result = inv_enable_fast_nomot();

    /**
     * @brief 启用 MPL 在温度变化时校准陀螺仪功能
     */
    result = inv_enable_gyro_tc();

    /**
     * @brief 启用 eMPL-hal 读出 MPL 中欧拉角等数据功能
     */
    result = inv_enable_eMPL_outputs();

    /**
     * @brief 启动 MPL 计算
     */
    result = inv_start_mpl();
    if (result == INV_ERROR_NOT_AUTHORIZED)
    {
        MPL_LOGE("Not authorized.\n");
    }
    else if (result)
    {
        MPL_LOGE("Could not start the MPL.\n");
    }
    return result;
};

/**
 * @brief 平台相关参数
 */
struct platform_data_s
{
    /**
     * @brief 旋转矩阵，使芯片返回的 XYZ 方向与设备 XYZ 方向相同
     * 芯片返回的 XYZ 方向与设备本身定义的 XYZ 方向可能不同，需要用此矩阵进行纠正
     */
    signed char orientation[9];
};

/**
 * @brief 旋转矩阵, 根据你的 MPU 在设备中方位来设定
 */
static struct platform_data_s gyro_pdata = {
    .orientation = {1, 0, 0,
                    0, 1, 0,
                    0, 0, 1}};

/**
 * @brief 配置 MPL参数
 * 
 * @attention 需要先初始化 MPL
 */
static void mpl_config()
{
    unsigned char accel_fsr;
    unsigned short gyro_fsr, gyro_rate;

    mpu_get_sample_rate(&gyro_rate);
    mpu_get_gyro_fsr(&gyro_fsr);
    mpu_get_accel_fsr(&accel_fsr);

    /**
     * @brief 配置 MPL 陀螺仪 和 加速度计 采样率参数
     */
    inv_set_gyro_sample_rate(1000000L / gyro_rate);
    inv_set_accel_sample_rate(1000000L / gyro_rate);

    /**
     * @brief 配置 磁场计 采样率参数
     */
    //inv_set_compass_sample_rate(COMPASS_READ_MS * 1000L);

    /**
     * @brief 设定 Gyro, Accel 旋转矩阵, 以适配设备; 设定硬件 数据单位 为 dps/g's/degrees
     * 
     * @attention 旋转矩阵需要根据你的 MPU 在设备中方位来设定
     */
    inv_set_gyro_orientation_and_scale(
        inv_orientation_matrix_to_scalar(gyro_pdata.orientation),
        (long)gyro_fsr << 15);
    inv_set_accel_orientation_and_scale(
        inv_orientation_matrix_to_scalar(gyro_pdata.orientation),
        (long)accel_fsr << 15);

    /**
     * @brief 设定 Compass 矩阵 和 单位
     */
    // inv_set_compass_orientation_and_scale(
    //     inv_orientation_matrix_to_scalar(compass_pdata.orientation),
    //     (long)compass_fsr << 15);
}

/**
 * @brief 初始化 DMP
 * 
 * 1. 加载 DMP 固件到 MPU内存 dmp_load_motion_driver_firmware()
 * 2. 设定 DMP 渲染矩阵 
 * 3. 注册姿态变化(gesture)回调(callbacks), 无论是否用到都要注册
 * 不启用对应特性(feature)回调不会被调用
 * 4. 设定启用的特性(feature) dmp_enable_feature(mask)
 * 5. 设定 DMP 输出到 FIFO 队列的速度 dmp_set_fifo_rate(freq)
 * 6. 调用设定特性(feature)属性的函数
 * 
 * 7. 启动 DMP -> mpu_set_dmp_state(1)
 * 
 * inv_mpu_dmp_motion_driver.c 中特性(feature)列表:
 * 1. DMP_FEATURE_LP_QUAT:
 *      Gyro-Only(3轴) 四元数计算
 *      采样 Gyro的速率为 200Hz
 *      对比使用 MPL在低采样率下计算的情况, 此模式精度更高
 * 2. DMP_FEATURE_6X_LP_QUAT:
 *      Gyro & Accel(6轴) 四元数计算
 *      采样 Gyro & Accel的速率为 200Hz
 *      不能与 DMP_FEATURE_LP_QUAT 同时使用
 * 3. DMP_FEATURE_TAP: 检测在 XYZ方向上的轻敲动作(tap)
 * 4. DMP_FEATURE_ANDROID_ORIENT:
 *      Google 屏幕旋转算法, 在屏幕应当旋转时产生一个事件
 * 5. DMP_FEATURE_GYRO_CAL:
 *      8 秒内没有移动, 校准陀螺仪
 * 6. DMP_FEATURE_SEND_RAW_ACCEL: 发送 加速度计原始数据(Raw Accel)到 FIFO
 * 7. DMP_FEATURE_SEND_RAW_GYRO: 发送 陀螺仪原始数据(Raw Gyro)到 FIFO
 * 8. DMP_FEATURE_SEND_CAL_GYRO: 发送 校准后的陀螺仪数据到 FIFO
 *      不能与 DMP_FEATURE_SEND_RAW_GYRO 同时使用
 */
void dmp_init(int sampleRate)
{
    /**
     * @brief 加载 DMP 固件到 MPU内存
     */
    dmp_load_motion_driver_firmware();

    /**
     * @brief 设定 DMP 矩阵, 以适配设备
     * 
     * @attention 旋转矩阵需要根据你的 MPU 在设备中方位来设定
     */
    dmp_set_orientation(
        inv_orientation_matrix_to_scalar(gyro_pdata.orientation));

    /**
     * @brief 注册 轻敲(tap)事件回调
     */
    dmp_register_tap_cb(tap_cb);

    /**
     * @brief 注册 设备方向事件回调
     */
    dmp_register_android_orient_cb(android_orient_cb);

    /**
     * @brief 启用 DMP_FEATURE_TAP 特性以使 dmp_set_fifo_rate 参数生效
     * 启用 DMP时, DMP会以 200Hz的速率采集传感器的数据,
     * 然后以 dmp_set_fifo_rate 设定的速率输出到 FIFO中,
     * 每当一个新数据被放入 FIFO就产生一个中断
     * 
     * 已知问题:
     *      当你没有启用 DMP_FEATURE_TAP特性时,
     *      FIFO 产生中断的速率变为 200Hz, 即 dmp_set_fifo_rate 设定的值无效
     * 解决方案:
     *      启用 DMP_FEATURE_TAP特性
     * 
     * @attention DMP 仅能工作在 Gyro +-2000dps 和 Accel +-2G下
     */
    hal.dmp_features = DMP_FEATURE_6X_LP_QUAT |
                       DMP_FEATURE_TAP |
                       DMP_FEATURE_ANDROID_ORIENT |
                       DMP_FEATURE_SEND_RAW_ACCEL |
                       DMP_FEATURE_SEND_CAL_GYRO |
                       DMP_FEATURE_GYRO_CAL;
    dmp_enable_feature(hal.dmp_features);
    dmp_set_fifo_rate(sampleRate);
    mpu_set_dmp_state(1);
    hal.dmp_on = 1;
}

/**
 * @brief MPU 归零
 */
void mpu_return_zero()
{
    int result;
    long gyro[3], accel[3];

    result = mpu_run_self_test(gyro, accel);

    if (result == 0x5)
    {
        MPL_LOGI("Passed!\n");
        MPL_LOGI("accel: %7.4f %7.4f %7.4f\n",
                 accel[0] / 65536.f,
                 accel[1] / 65536.f,
                 accel[2] / 65536.f);
        MPL_LOGI("gyro: %7.4f %7.4f %7.4f\n",
                 gyro[0] / 65536.f,
                 gyro[1] / 65536.f,
                 gyro[2] / 65536.f);

        /* Push the calibrated data to the MPL library.
         *
         * MPL expects biases in hardware units << 16, but self test returns
		 * biases in g's << 16.
		 */
        unsigned short accel_sens;
        float gyro_sens;

        mpu_get_accel_sens(&accel_sens);
        accel[0] *= accel_sens;
        accel[1] *= accel_sens;
        accel[2] *= accel_sens;
        inv_set_accel_bias(accel, 3);
        dmp_set_accel_bias(accel);
        mpu_get_gyro_sens(&gyro_sens);
        gyro[0] = (long)(gyro[0] * gyro_sens);
        gyro[1] = (long)(gyro[1] * gyro_sens);
        gyro[2] = (long)(gyro[2] * gyro_sens);
        inv_set_gyro_bias(gyro, 3);
        dmp_set_gyro_bias(gyro);
    }
    else
    {
        if (!(result & 0x1))
            MPL_LOGE("Gyro failed.\n");
        if (!(result & 0x2))
            MPL_LOGE("Accel failed.\n");
        if (!(result & 0x4))
            MPL_LOGE("Compass failed.\n");
    }

    /* Let MPL know that contiguity was broken. */
    inv_accel_was_turned_off();
    inv_gyro_was_turned_off();
    inv_compass_was_turned_off();
}

/**
 * @brief 初始化 MPU6050
 * 
 * @param sampleRate 采样率
 * @param gyro_data 旋转矩阵
 * @param useDMP 是否使用 DMP
 */
inv_error_t mpu6050_init(int sampleRate, bool useDMP)
{
    log_i("Begin initialize MPU6050");
    inv_error_t result;

    /**
     * @brief 初始化 MPU
     */
    result = mpu_init(0);
    if (result)
    {
        MPL_LOGE("Could not initialize gyro.\n");
    }

    /**
     * @brief 设定 MPU 采集 陀螺仪 和 加速度 数据
     * 
     * INV_XYZ_GYRO - 陀螺仪
     * INV_XYZ_ACCEL - 加速度
     * INV_XYZ_COMPASS - 磁场(MPU6050 无磁场计)
     */
    mpu_set_sensors(INV_XYZ_GYRO | INV_XYZ_ACCEL);

    /**
     * @brief 设定 FIFO 队列中放入 陀螺仪 和 加速度 数据
     * 
     * INV_XYZ_GYRO - 陀螺仪
     * INV_XYZ_ACCEL - 加速度
     * INV_XYZ_COMPASS - 磁场(MPU6050 无磁场计)
     */
    mpu_configure_fifo(INV_XYZ_GYRO | INV_XYZ_ACCEL);

    /**
     * @brief 设定传感器(Gyro, Accel)采样率, 单位 Hz, 最大 200Hz
     * 
     * @attention 采样率受限于 MCU
     * TI 16-bit MSP430 在计算 9 轴四元数时应设定在 100Hz以下,
     * 超过此值开始丢失数据
     * 32 位处理器通常可以达到 200 Hz
     * 
     * 使用 DMP 可以达到最大采样速率
     */
    mpu_set_sample_rate(sampleRate);

    mpu_set_gyro_fsr(2000);
    mpu_set_accel_fsr(2);

    /**
     * @brief 设定磁场计采样率, 单位 Hz
     * 
     * 设定较低的值以减少功耗
     */
    //mpu_set_compass_sample_rate(1000 / COMPASS_READ_MS);

    mpl_init();

    mpl_config();

    hal.sensors = ACCEL_ON | GYRO_ON;
    hal.dmp_on = 0;
    hal.report = 0;
    hal.rx.cmd = 0;
    hal.next_pedo_ms = 0;
    hal.next_compass_ms = 0;
    hal.next_temp_ms = 0;

    mpu_return_zero();

    if (useDMP)
    {
        dmp_init(sampleRate);
    }

    return result;
}

#include <stdlib.h>
#include <string.h>

/**
 * @brief 读取 FIFO 中的数据， 并送入 MPL
 * 
 * @param newData 是否有新数据
 * @return FIFO中剩余数据量
 */
unsigned char fifo_read(bool *newData)
{
    unsigned char more;

    short gyro[3];
    long accel[3], quat[4];

    unsigned long sensor_timestamp;
    short accel_short[3], sensors;
    long temperature;

    if (hal.dmp_on)
    {
        dmp_read_fifo(gyro, accel_short, quat, &sensor_timestamp, &sensors, &more);
    }
    else
    {
        mpu_read_fifo(gyro, accel_short, &sensor_timestamp, (unsigned char *)&sensors, &more);
    }

    if (sensors & INV_XYZ_GYRO)
    {
        log_v("Receive Gyro:%d, %d, %d", gyro[0], gyro[1], gyro[2]);

        /* Push the new data to the MPL. */
        inv_build_gyro(gyro, sensor_timestamp);
        *newData = true;
        if (HAL_GetTick() > hal.next_temp_ms)
        {
            hal.next_temp_ms = 500 + HAL_GetTick();
            /* Temperature only used for gyro temp comp. */
            mpu_get_temperature(&temperature, &sensor_timestamp);
            inv_build_temp(temperature, sensor_timestamp);
        }
    }
    if (sensors & INV_XYZ_ACCEL)
    {
        accel[0] = (long)accel_short[0];
        accel[1] = (long)accel_short[1];
        accel[2] = (long)accel_short[2];
        log_v("Receive Accel:%d, %d, %d", accel[0], accel[1], accel[2]);
        inv_build_accel(accel, 0, sensor_timestamp);
        *newData = true;
    }
    if (sensors & INV_WXYZ_QUAT)
    {
        log_v("Receive Quat:%d, %d, %d, %d", quat[0], quat[1], quat[2], quat[3]);
        inv_build_quat(quat, 0, sensor_timestamp);
        *newData = true;
    }

    if (*newData)
    {
        inv_execute_on_data();
    }

    return more;
}
