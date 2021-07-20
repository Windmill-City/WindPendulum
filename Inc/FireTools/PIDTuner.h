/**
 * @file PIDTuner.h
 * @author Windmill_City
 * @brief 野火上位机 PID调试助手
 * @version 1.0.2.5
 * @date 2021-07-18
 * 
 * @copyright Copyright (c) 2021
 */

#ifndef __PIDTUNER_H__
#define __PIDTUNER_H__


#include "Server.h"

#define LOG_TAG "[FireTools]PIDTuner"
#include <elog.h>

/**
 * @brief PID 通道
 */
#define CH1 0x01
#define CH2 0x02
#define CH3 0x03
#define CH4 0x04
#define CH5 0x05

/**
 * 上位机为 Client -> C
 * 下位机为 Server -> S
 */
#pragma region SC->下位机到上位机
/**
 * 发送上位机目标值
 * 
 * @param int 目标值
 */
#define SC_SEND_TARGET 0x01
/**
 * 发送上位机实际值
 * 
 * @param int 实际值
 */
#define SC_SEND_ACTUAL 0x02
/**
 * 同步上位机 PID值
 * 
 * @param float P
 * @param float I
 * @param float D
 */
#define SC_SYNC_PID 0x03
/**
 * 同步上位机启动按钮状态
 */
#define SC_SYNC_START 0x04
/**
 * 同步上位机停止按钮状态
 */
#define SC_SYNC_STOP 0x05
/**
 * 同步上位机周期
 * 
 * @param unsigned int 周期值
 */
#define SC_SYNC_PERIOD 0x06
#pragma endregion

#pragma region 发送
/**
 * @brief 发送目标值到上位机
 * 
 * @param target 目标值
 */
void send_target(uint8_t channel, int target)
{
    send_packet(channel, SC_SEND_TARGET, (uint8_t *)&target, sizeof(target));
}

/**
 * @brief 发送实际值到上位机
 * 
 * @param target 实际值
 */
void send_actual(uint8_t channel, int actual)
{
    send_packet(channel, SC_SEND_ACTUAL, (uint8_t *)&actual, sizeof(actual));
}

/**
 * @brief 同步 PID到上位机
 */
void sync_pid(uint8_t channel, float p, float i, float d)
{
    float data[3] = {p, i, d};
    send_packet(channel, SC_SYNC_PID, (u_int8_t *)data, sizeof(data));
}

/**
 * @brief 同步启动按钮到上位机
 */
void sync_start(uint8_t channel)
{
    send_packet(channel, SC_SYNC_START, 0, 0);
}

/**
 * @brief 同步停止按钮到上位机
 */
void sync_stop(uint8_t channel)
{
    send_packet(channel, SC_SYNC_STOP, 0, 0);
}

/**
 * @brief 同步周期到上位机
 */
void sync_period(uint8_t channel, unsigned int period)
{
    send_packet(channel, SC_SYNC_PERIOD, (u_int8_t *)&period, sizeof(period));
}
#pragma endregion

#pragma region CS->上位机到下位机
/**
 * 设置下位机 PID值
 * 
 * @param float P
 * @param float I
 * @param float D
 */
#define CS_SET_PID 0x10
/**
 * 设置下位机目标值
 * 
 * @param int 目标值
 */
#define CS_SET_TARGET 0x11
/**
 * 下位机启动指令
 */
#define CS_CM_START 0x12
/**
 * 下位机停止指令
 */
#define CS_CM_STOP 0x13
/**
 * 下位机复位指令
 */
#define CS_CM_RESET 0x14
/**
 * 设置下位机周期
 * 
 * @param unsigned int 周期值
 */
#define CS_SET_PEIRIOD 0x15
#pragma endregion

#endif