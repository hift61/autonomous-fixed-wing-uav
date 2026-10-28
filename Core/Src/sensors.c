/*
 * sensors.c  -  one-call sensor hub. See sensors.h for the wiring / library map.
 */
#include "sensors.h"
#include "spi.h"
#include "i2c.h"
#include "usart.h"

#include "MPU9250.h"
#include "BMPXX80.h"
#include "lwgps/lwgps.h"
#ifdef BMP180
#include "delays.h"
#endif

#include <string.h>
#include <stdio.h>

/* USART2 (ST-Link virtual COM) is created in main.c and used for debug text */
extern UART_HandleTypeDef huart2;

/* ---- MPU-9250 scale factors for the library's default ranges ----
 *   Accel default = +/-16 g   -> 32768/16   = 2048 LSB per g
 *   Gyro  default = +/-250 dps -> 32768/250  = 131.072 LSB per (deg/s)
 *   Mag   AK8963 16-bit        -> 4912/32760 = ~0.15 uT per LSB (ASA already applied) */
#define ACC_LSB_PER_G     2048.0f
#define GYR_LSB_PER_DPS   131.072f
#define MAG_UT_PER_LSB    0.15f

/* Barometer I2C address the BMPXX80 library talks to (0x76 << 1). */
#define BMP_ADDR_PRIMARY   0xEC   /* 0x76 */
#define BMP_ADDR_SECONDARY 0xEE   /* 0x77 */
#define BMP_REG_CHIPID     0xD0

/* ---- GPS parser state (fed byte-by-byte from the USART1 interrupt) ---- */
static lwgps_t          s_gps;
static uint8_t          s_gps_rx_byte;

/* ---- init results ---- */
static uint8_t          s_imu_ok  = 0;
static uint8_t          s_baro_ok = 0;

/* -------- small debug helper (USART2) -------- */
static void dbg(const char *s)
{
    HAL_UART_Transmit(&huart2, (uint8_t *)s, (uint16_t)strlen(s), 100);
}

/* format a floating value without needing printf %f / long-long support */
static int f2s(char *buf, double v, int dec)
{
    char *p = buf;
    if (v < 0.0) { *p++ = '-'; v = -v; }
    if (dec > 9) dec = 9;
    long scale = 1;
    for (int i = 0; i < dec; i++) scale *= 10;
    long scaled = (long)(v * (double)scale + 0.5);
    long ip = scaled / scale;
    long fp = scaled % scale;
    p += sprintf(p, "%ld", ip);
    if (dec > 0)
    {
        *p++ = '.';
        for (long div = scale / 10; div > 0; div /= 10)
            *p++ = (char)('0' + (fp / div) % 10);
    }
    *p = '\0';
    return (int)(p - buf);
}

/* -------------------------------------------------------------------------- */
uint8_t Sensors_Init(void)
{
    uint8_t err = 0;
    char line[64];

    /* 1) Bring up the buses (pin AF muxing was done in MX_GPIO_Init) */
    MX_SPI1_Init();
    MX_I2C1_Init();
    MX_USART1_UART_Init();

    /* Idle the IMU chip-select high before any transfer */
    HAL_GPIO_WritePin(GY_CS_GPIO_Port, GY_CS_Pin, GPIO_PIN_SET);

    dbg("\r\n=== Sensors_Init ===\r\n");

    /* 2) IMU + magnetometer (MPU-9250 / AK8963) */
    if (MPU9250_Init() == 0)
    {
        s_imu_ok = 1;
        dbg("IMU  MPU-9250 : OK (accel/gyro/mag)\r\n");
    }
    else
    {
        s_imu_ok = 0;
        err |= 0x01;
        dbg("IMU  MPU-9250 : FAIL (check SPI wiring / CS=PB6)\r\n");
    }

    /* 3) Barometer: detect the chip by its ID register before init */
    {
        uint8_t id = 0;
        if (HAL_I2C_Mem_Read(&hi2c1, BMP_ADDR_PRIMARY, BMP_REG_CHIPID, 1, &id, 1, 100) != HAL_OK)
        {
            /* try the alternate 0x77 address */
            HAL_I2C_Mem_Read(&hi2c1, BMP_ADDR_SECONDARY, BMP_REG_CHIPID, 1, &id, 1, 100);
        }
        sprintf(line, "BARO chip-ID  : 0x%02X ", id);
        dbg(line);
        if      (id == 0x58) dbg("(BMP280)\r\n");
        else if (id == 0x55) dbg("(BMP180 -> enable #define BMP180 in BMPXX80.h)\r\n");
        else if (id == 0x60) dbg("(BME280 -> enable #define BME280 in BMPXX80.h)\r\n");
        else                 dbg("(unknown / not found)\r\n");

#ifdef BMP180
        if (id == 0x55) s_baro_ok = 1;
        Delays_Init();
        BMP180_Init(&hi2c1, BMP180_ULTRAHIGHRES);
#else
        if (id == 0x58) s_baro_ok = 1;                 /* BMP280 present */
        BMP280_Init(&hi2c1, BMP280_TEMPERATURE_16BIT, BMP280_ULTRAHIGHRES, BMP280_NORMALMODE);
        BMP280_SetConfig(BME280_STANDBY_MS_0_5, BME280_FILTER_X16);
#endif
        if (!s_baro_ok) err |= 0x02;
    }

    /* 4) GPS: init the NMEA parser and start byte-by-byte UART1 reception */
    lwgps_init(&s_gps);
    HAL_UART_Receive_IT(&huart1, &s_gps_rx_byte, 1);
    dbg("GPS  NEO-6M   : USART1 RX started (9600 8N1)\r\n\r\n");

    return err;
}

/* -------------------------------------------------------------------------- */
void Sensor_read(SensorData_t *out)
{
    memset(out, 0, sizeof(*out));

    /* ---- IMU / magnetometer ---- */
    if (s_imu_ok)
    {
        int16_t acc[3], mag[3], gyr[3];
        MPU9250_GetData(acc, mag, gyr);          /* order: Acc, Mag, Gyro */
        for (int i = 0; i < 3; i++)
        {
            out->accel_g[i]  = (float)acc[i] / ACC_LSB_PER_G;
            out->gyro_dps[i] = (float)gyr[i] / GYR_LSB_PER_DPS;
            out->mag_uT[i]   = (float)mag[i] * MAG_UT_PER_LSB;
        }
    }
    out->imu_ok = s_imu_ok;

    /* ---- Barometer ---- */
    if (s_baro_ok)
    {
#ifdef BMP180
        out->baro_temp_c   = BMP180_ReadTemperature();
        out->baro_press_pa = (float)BMP180_ReadPressure();
        out->baro_alt_m    = BMP180_PressureToAltitude(SEA_LEVEL_PRESSURE_PA, out->baro_press_pa);
#else
        float   t = 0.0f;
        int32_t p = 0;
        BMP280_ReadTemperatureAndPressure(&t, &p);
        out->baro_temp_c   = t;
        out->baro_press_pa = (float)p;
        out->baro_alt_m    = BMP280_ReadAltitude(SEA_LEVEL_PRESSURE_PA);
#endif
    }
    out->baro_ok = s_baro_ok;

    /* ---- GPS (latest values parsed in the background) ---- */
    out->gps_lat       = s_gps.latitude;
    out->gps_lon       = s_gps.longitude;
    out->gps_alt_m     = s_gps.altitude;
    out->gps_speed_kmh = lwgps_to_speed(s_gps.speed, LWGPS_SPEED_KPH);
    out->gps_fix       = s_gps.fix;
    out->gps_sats      = s_gps.sats_in_use;
    out->gps_valid     = s_gps.is_valid;
}

/* -------------------------------------------------------------------------- */
void Sensor_print(const SensorData_t *d)
{
    char line[128];
    char a[16], b[16], c[16];

    dbg("---- SENSORS ----\r\n");

    f2s(a, d->accel_g[0], 3); f2s(b, d->accel_g[1], 3); f2s(c, d->accel_g[2], 3);
    sprintf(line, "ACC [g]    x=%s y=%s z=%s\r\n", a, b, c); dbg(line);

    f2s(a, d->gyro_dps[0], 2); f2s(b, d->gyro_dps[1], 2); f2s(c, d->gyro_dps[2], 2);
    sprintf(line, "GYR [dps]  x=%s y=%s z=%s\r\n", a, b, c); dbg(line);

    f2s(a, d->mag_uT[0], 1); f2s(b, d->mag_uT[1], 1); f2s(c, d->mag_uT[2], 1);
    sprintf(line, "MAG [uT]   x=%s y=%s z=%s\r\n", a, b, c); dbg(line);

    f2s(a, d->baro_temp_c, 2); f2s(b, d->baro_press_pa, 0); f2s(c, d->baro_alt_m, 2);
    sprintf(line, "BARO       T=%s C  P=%s Pa  Alt=%s m  (ok=%u)\r\n",
            a, b, c, d->baro_ok); dbg(line);

    f2s(a, d->gps_lat, 6); f2s(b, d->gps_lon, 6);
    sprintf(line, "GPS  lat=%s lon=%s\r\n", a, b); dbg(line);
    f2s(a, d->gps_alt_m, 1); f2s(b, d->gps_speed_kmh, 2);
    sprintf(line, "     alt=%s m  spd=%s km/h  fix=%u sats=%u valid=%u\r\n",
            a, b, d->gps_fix, d->gps_sats, d->gps_valid); dbg(line);
}

/* -------------------------------------------------------------------------- */
/* USART1 RX complete: feed the byte to lwgps and re-arm reception.           */
void HAL_UART_RxCpltCallback(UART_HandleTypeDef *huart)
{
    if (huart->Instance == USART1)
    {
        lwgps_process(&s_gps, &s_gps_rx_byte, 1);
        HAL_UART_Receive_IT(&huart1, &s_gps_rx_byte, 1);
    }
}

/* If the GPS UART reports an error (e.g. overrun), re-arm reception.         */
void HAL_UART_ErrorCallback(UART_HandleTypeDef *huart)
{
    if (huart->Instance == USART1)
    {
        HAL_UART_Receive_IT(&huart1, &s_gps_rx_byte, 1);
    }
}
