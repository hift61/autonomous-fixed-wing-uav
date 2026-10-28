/*
 * sensors.h  -  one-call sensor hub for the fixed_wing board.
 *
 *   IMU  : MPU-9250        on SPI1 (CS = PB6, INT = PB0)   [desertkun/MPU9250]
 *   MAG  : AK8963          inside the MPU-9250 (read over SPI)
 *   BARO : BMP280 / BMP180 on I2C1                          [lamik/BMXX80_STM32_HAL]
 *   GPS  : u-blox NEO-6M   on USART1 (NMEA, 9600 8N1)       [MaJerle/lwgps]
 *
 * Usage (in main.c):
 *      SensorData_t data;
 *      Sensors_Init();               // once, after the CubeMX MX_*_Init calls
 *      while (1) {
 *          Sensor_read(&data);       // fills every field in one shot
 *          Sensor_print(&data);      // optional: dump to the USART2 debug port
 *          HAL_Delay(500);
 *      }
 */
#ifndef SENSORS_H_
#define SENSORS_H_
#ifdef __cplusplus
extern "C" {
#endif

#include "main.h"
#include <stdint.h>

/* ===================== user settings ===================== */
/* Sea-level reference pressure used to derive barometric altitude [Pa].    */
/* Standard atmosphere = 101325. Adjust to the local QNH for accurate alt.  */
#define SEA_LEVEL_PRESSURE_PA   101325.0f
/* BMP chip model is chosen in BMPXX80.h (default BMP280; enable #define     */
/* BMP180 there if your barometer is a BMP180). Sensors_Init() reads the     */
/* chip-ID and prints a warning over USART2 if it does not match.           */
/* ========================================================= */

typedef struct
{
    /* ---- IMU: MPU-9250 (SPI1) ---- */
    float   accel_g[3];       /* acceleration  x,y,z  [g]        */
    float   gyro_dps[3];      /* angular rate  x,y,z  [deg/s]    */
    float   mag_uT[3];        /* magnetic field x,y,z [uT]  (AK8963) */

    /* ---- Barometer: BMP280 / BMP180 (I2C1) ---- */
    float   baro_temp_c;      /* temperature          [degC]     */
    float   baro_press_pa;    /* pressure             [Pa]       */
    float   baro_alt_m;       /* barometric altitude  [m]        */

    /* ---- GPS: NEO-6M (USART1, NMEA) ---- */
    double  gps_lat;          /* latitude             [deg]      */
    double  gps_lon;          /* longitude            [deg]      */
    double  gps_alt_m;        /* GPS altitude         [m]        */
    double  gps_speed_kmh;    /* ground speed         [km/h]     */
    uint8_t gps_fix;          /* 0 = none, 1 = GPS, 2 = DGPS     */
    uint8_t gps_sats;         /* satellites in use               */
    uint8_t gps_valid;        /* 1 = position valid (RMC)        */

    /* ---- status flags ---- */
    uint8_t imu_ok;           /* 1 = MPU-9250 answered at init   */
    uint8_t baro_ok;          /* 1 = barometer detected at init  */
} SensorData_t;

/* Initialise SPI1/I2C1/USART1 and all sensors. Returns 0 if everything is  */
/* OK, otherwise a bitmask: bit0 = IMU fail, bit1 = barometer fail.         */
uint8_t Sensors_Init(void);

/* Read every sensor once and fill *out (GPS values come from the latest    */
/* NMEA data parsed in the background over USART1).                         */
void    Sensor_read(SensorData_t *out);

/* Print a human-readable snapshot to the USART2 debug port (115200 8N1).   */
void    Sensor_print(const SensorData_t *d);

#ifdef __cplusplus
}
#endif
#endif /* SENSORS_H_ */
