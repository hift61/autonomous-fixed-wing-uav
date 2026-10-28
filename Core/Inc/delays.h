/*
 * delays.h  -  microsecond busy-wait (DWT cycle counter based).
 *
 * Only needed by the BMP180 / BME280 code paths of the BMPXX80 driver.
 * Self-contained: no timer peripheral required. Call Delays_Init() once
 * before the first Delay_us() (Sensors_Init() does this automatically when
 * BMP180 is selected).
 */
#ifndef DELAYS_H_
#define DELAYS_H_

#include <stdint.h>

void Delays_Init(void);
void Delay_us(uint16_t us);

#endif /* DELAYS_H_ */
