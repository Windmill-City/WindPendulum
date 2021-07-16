#include <string.h>
#include "usart.h"

/**
 * @brief 正点原子 ESP8266 Wifi模块
 */
class Wifi
{
public:
    enum Mode
    {
        STA,
        AP,
        STA_AP
    };

private:
    UART_HandleTypeDef *port;
public:
    Wifi(UART_HandleTypeDef *port)
    {
        this->port = port;
    }
};