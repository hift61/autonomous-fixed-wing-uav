/*
 * spi.c  -  SPI1 master for the MPU-9250 IMU.
 *
 * The pins PA5/PA6/PA7 are already put in AF5 (SPI1) mode by MX_GPIO_Init()
 * in main.c, so here we only enable the SPI1 clock and configure the block.
 * Chip-select is driven manually on PB6 (see GY_CS_* in main.h).
 */
#include "spi.h"

SPI_HandleTypeDef hspi1;

void MX_SPI1_Init(void)
{
  __HAL_RCC_SPI1_CLK_ENABLE();

  hspi1.Instance               = SPI1;
  hspi1.Init.Mode              = SPI_MODE_MASTER;
  hspi1.Init.Direction         = SPI_DIRECTION_2LINES;
  hspi1.Init.DataSize          = SPI_DATASIZE_8BIT;
  hspi1.Init.CLKPolarity       = SPI_POLARITY_LOW;   /* SPI mode 0. If IMU misreads, try SPI_POLARITY_HIGH + SPI_PHASE_2EDGE (mode 3) */
  hspi1.Init.CLKPhase          = SPI_PHASE_1EDGE;
  hspi1.Init.NSS               = SPI_NSS_SOFT;
  hspi1.Init.BaudRatePrescaler = SPI_BAUDRATEPRESCALER_64; /* 84 MHz / 64 = 1.3 MHz (safe for MPU-9250 register access) */
  hspi1.Init.FirstBit          = SPI_FIRSTBIT_MSB;
  hspi1.Init.TIMode            = SPI_TIMODE_DISABLE;
  hspi1.Init.CRCCalculation    = SPI_CRCCALCULATION_DISABLE;
  hspi1.Init.CRCPolynomial     = 10;
  if (HAL_SPI_Init(&hspi1) != HAL_OK)
  {
    Error_Handler();
  }
}
