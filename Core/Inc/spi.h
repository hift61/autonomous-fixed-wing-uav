/*
 * spi.h  -  SPI1 for MPU-9250 (PA5 SCK / PA6 MISO / PA7 MOSI, CS = PB6)
 */
#ifndef __SPI_H__
#define __SPI_H__
#ifdef __cplusplus
extern "C" {
#endif

#include "main.h"

extern SPI_HandleTypeDef hspi1;

void MX_SPI1_Init(void);

#ifdef __cplusplus
}
#endif
#endif /* __SPI_H__ */
