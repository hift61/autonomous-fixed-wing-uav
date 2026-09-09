/*
 * usart.h  -  USART1 for the GPS (NEO-6M NMEA, PA9 TX / PA10 RX)
 *             (USART2 stays in main.c as the ST-Link debug port)
 */
#ifndef __USART_H__
#define __USART_H__
#ifdef __cplusplus
extern "C" {
#endif

#include "main.h"

extern UART_HandleTypeDef huart1;

void MX_USART1_UART_Init(void);

#ifdef __cplusplus
}
#endif
#endif /* __USART_H__ */
