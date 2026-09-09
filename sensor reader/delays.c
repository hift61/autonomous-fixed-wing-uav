/*
 * delays.c  -  microsecond delay using the Cortex-M4 DWT cycle counter.
 *
 * Replaces the original TIM3-based delays.c from the BMPXX80 library so that
 * no extra timer needs to be configured. Used only by the BMP180/BME280 paths.
 */
#include "main.h"
#include "delays.h"

void Delays_Init(void)
{
  CoreDebug->DEMCR |= CoreDebug_DEMCR_TRCENA_Msk;
  DWT->CYCCNT = 0;
  DWT->CTRL |= DWT_CTRL_CYCCNTENA_Msk;
}

void Delay_us(uint16_t us)
{
  uint32_t start = DWT->CYCCNT;
  uint32_t ticks = (uint32_t)us * (SystemCoreClock / 1000000U);
  while ((DWT->CYCCNT - start) < ticks)
  {
    /* busy wait */
  }
}
