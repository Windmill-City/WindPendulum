/**
 * @file Server.h
 * @author Windmill_City
 * @brief 野火上位机 通信接口
 * @version 1.0.2.5
 * @date 2021-07-18
 * 
 * @copyright Copyright (c) 2021
 */

#ifndef __SERVER_H__
#define __SERVER_H__

#include <stdio.h>
#include <string.h>
#include "usart.h"

#define LOG_TAG "FireTools"
#include <elog.h>

#define PACKET_HEAD 0x59485A53

/**
 * @brief 数据包基础结构
 * 一字节对齐
 */
#pragma pack(1)
typedef struct PacketBase
{
    /**
     * @brief 包头, 固定为 0x59485A53
     */
    uint32_t head;
    /**
     * @brief 通道/设备地址
     */
    uint8_t addr;
    /**
     * @brief 包长度
     * 从包头到校验所有数据长度
     */
    uint32_t len;
    /**
     * @brief 指令
     */
    uint8_t cmd;
} PacketBase_t, *pPacketBase;
#pragma pack()

/**
 * @brief 计算发送端数据校验和
 * 
 * @param data 数据
 * @param len 长度, 字节
 * @return uint8_t 校验和
 */
uint8_t checksum_tx(uint8_t *data, size_t len)
{
    uint8_t checksum = 0;
    while (len--)
    {
        checksum += data[len];
    }
    return ~checksum;
}

/**
 * @brief 计算接收端数据校验和是否合法
 * 
 * @param data 数据, 包含校验和
 * @param len 长度, 字节
 * @return true 校验通过
 * @return false 校验失败
 */
bool checksum_rx(uint8_t *data, size_t len)
{
    uint8_t checksum = 0;
    while (len--)
    {
        checksum += data[len];
    }
    return checksum + 1;
}

/**
 * @brief 过滤要发送的地址
 */
extern uint8_t gAddressFilter = 0;

/**
 * @brief 发送数据包
 * 
 * @param cmd 命令
 * @param data 参数
 * @param len 参数长度, 字节
 */
void send_packet(uint8_t addr, uint8_t cmd, uint8_t *data, size_t len)
{
    /**
     * @brief 数据包基础长度 + 参数长度 + 校验和长度
     */
    size_t packet_len_no_checksum = sizeof(PacketBase_t) + len;
    size_t packet_len = packet_len_no_checksum + sizeof(uint8_t);

    uint8_t buf[packet_len];
    uint8_t *checksum = buf + packet_len_no_checksum;

    pPacketBase packet = (pPacketBase)buf;
    packet->head = PACKET_HEAD;
    packet->addr = addr;
    packet->len = packet_len;
    packet->cmd = cmd;

    /**
     * @brief 复制参数到缓冲区
     */
    memcpy(buf + sizeof(PacketBase_t), data, len);

    *checksum = checksum_tx(buf, packet_len_no_checksum);

    HAL_UART_Transmit(&huart1, buf, packet_len, 100);
}

/**
 * @brief 服务器状态定义
 * 
 * 1. 等待数据包基础信息
 * 2. 等待包长度数据量接收完成
 */
#define WAIT_PACKET_BASE 0x01
#define WAIT_DATA 0x02

uint8_t gState = WAIT_PACKET_BASE;

/**
 * @brief 包处理函数
 * 
 * @param packet 包基础信息
 * @param data 数据区信息
 * @param len 数据区长度
 */
typedef void (*PacketHandlerCallback)(pPacketBase packet, uint8_t *data, size_t len);

PacketHandlerCallback gPkHandler;

/**
 * @brief 数据缓冲
 */
#define RX_MAX_BUF_SIZE 256
uint8_t gRxBuffer[RX_MAX_BUF_SIZE];

/**
 * @brief 初始化接收服务
 */
void init_firetools_server(PacketHandlerCallback handler)
{
    gPkHandler = handler;
    //接收数据包基础信息
    HAL_UART_Receive_IT(&huart1, (uint8_t *)&gRxBuffer, sizeof(PacketBase_t));

    log_d("Initialized!")
}

/**
 * @brief 处理收到的数据
 * 
 * @param huart 串口句柄
 */
void HAL_UART_RxCpltCallback(UART_HandleTypeDef *huart)
{
    struct PacketBase *pkBase = (pPacketBase)gRxBuffer;
    switch (gState)
    {
    case WAIT_PACKET_BASE:
        //错误的包头
        if (pkBase->head != PACKET_HEAD)
        {
            log_e("Wrong packet head!");
            //重新接收基础信息
            HAL_UART_Receive_IT(&huart1, (uint8_t *)&gRxBuffer, sizeof(PacketBase_t));
            break;
        }

        size_t data_left = pkBase->len - sizeof(PacketBase_t);
        if (data_left > 0)
        {
            //接收剩余数据
            HAL_UART_Receive_IT(&huart1, (uint8_t *)&gRxBuffer + sizeof(PacketBase_t), data_left);
            gState = WAIT_DATA;
        }
        else
        {
            goto HandleData;
        }
        break;
    case WAIT_DATA:
    HandleData:
        if (checksum_rx(gRxBuffer, pkBase->len))
        {
            log_d("Received data");
            gPkHandler(pkBase, gRxBuffer + sizeof(PacketBase_t),
                       pkBase->len - sizeof(PacketBase_t) - sizeof(uint8_t));
        }
        else
        {
            log_e("CheckSum failed!");
        }

        gState = WAIT_PACKET_BASE;
        HAL_UART_Receive_IT(&huart1, (uint8_t *)&gRxBuffer, sizeof(PacketBase_t));
        break;
    }
}

#endif