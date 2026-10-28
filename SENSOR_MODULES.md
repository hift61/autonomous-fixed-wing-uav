# fixed_wing 센서 모듈 레퍼런스

STM32F410RB 보드의 IMU · 지자기 · GPS · 기압 센서를 **한 번의 호출로 읽는** 모듈 구성과
각 모듈의 기능 · 실행 방법을 정리한 문서입니다.

---

## 0. 한눈에 보기

- **당신이 직접 쓰는 것**: `Sensors_Init()` 1회 + 루프에서 `Sensor_read()` (+ 원하면 `Sensor_print()`)
- **나머지 6개 모듈**은 그 아래에서 자동으로 동작합니다.
- 데이터는 `SensorData_t` 구조체 하나에 **사람이 읽을 수 있는 단위**로 담깁니다.

```
[직접 사용]   sensors  (Sensors_Init / Sensor_read / Sensor_print)
                 │  자동 호출
   ┌─────────────┼───────────────┬───────────────┐
[버스]        [IMU 드라이버]   [기압 드라이버]   [GPS 파서]
spi/i2c/usart  MPU9250          BMXX80           lwgps  (+ delays)
```

---

## 1. 실행 흐름 (main.c 에 이미 연결됨)

```c
int main(void)
{
    HAL_Init();
    SystemClock_Config();
    MX_GPIO_Init();            // 핀 AF (SPI/I2C/USART/CS) 설정  ← CubeMX 기본
    MX_USART2_UART_Init();     // 디버그 포트                    ← CubeMX 기본

    /* USER CODE BEGIN 2 */
    Sensors_Init();            // 버스 + 모든 센서 1회 초기화
    /* USER CODE END 2 */

    while (1)
    {
        /* USER CODE BEGIN 3 */
        Sensor_read(&g_sensors);   // 4종 센서 값을 구조체에 한 번에
        Sensor_print(&g_sensors);  // USART2(115200)로 사람이 읽을 형식 출력
        HAL_Delay(500);
        /* USER CODE END 3 */
    }
}
```

> **GPS는 백그라운드로 동작**합니다. USART1로 NMEA 문자가 들어올 때마다 인터럽트가
> 발생해 자동 파싱되고, `Sensor_read()`는 그때그때 최신 위치를 복사만 합니다.

---

## 2. sensors 모듈 ⭐ (Core/Src/sensors.c, Core/Inc/sensors.h)

실제로 당신이 다루는 메인 API.

| 함수 | 기능 | 반환값 |
|------|------|--------|
| `uint8_t Sensors_Init(void)` | SPI1·I2C1·USART1 + IMU·기압·GPS 전부 초기화 | `0`=정상 / 비트마스크(bit0=IMU실패, bit1=기압실패) |
| `void Sensor_read(SensorData_t *out)` | IMU+지자기+기압 즉시 읽기 + 최신 GPS 값 채우기 | — |
| `void Sensor_print(const SensorData_t *d)` | USART2(115200)로 사람이 읽을 형식 출력 | — |

### SensorData_t (읽어온 값이 담기는 구조체)

| 필드 | 의미 | 단위 |
|------|------|------|
| `accel_g[3]` | 가속도 x,y,z | g |
| `gyro_dps[3]` | 각속도 x,y,z | °/s |
| `mag_uT[3]` | 지자기 x,y,z | µT |
| `baro_temp_c` | 온도 | ℃ |
| `baro_press_pa` | 기압 | Pa |
| `baro_alt_m` | 기압 고도 | m |
| `gps_lat`, `gps_lon` | 위도, 경도 | ° |
| `gps_alt_m` | GPS 고도 | m |
| `gps_speed_kmh` | 지상 속도 | km/h |
| `gps_fix` | 픽스 종류 (0=없음,1=GPS,2=DGPS) | — |
| `gps_sats` | 사용 위성 수 | 개 |
| `gps_valid` | 위치 유효(RMC) | 0/1 |
| `imu_ok`, `baro_ok` | 각 센서 응답 여부 | 0/1 |

### 사용 예

```c
SensorData_t d;
Sensors_Init();

Sensor_read(&d);
float az = d.accel_g[2];            // Z축 가속도
if (d.gps_valid) {                  // 위성 잡힌 뒤에만
    double lat = d.gps_lat;
    double lon = d.gps_lon;
}
```

---

## 3. 버스 모듈 (spi / i2c / usart) — 자동 호출

`Sensors_Init()`이 내부에서 부르므로 **직접 부를 필요 없음.** 핀 AF는 `MX_GPIO_Init()`에
이미 설정돼 있고, 이 모듈들은 클럭 활성화 + 설정만 담당합니다.

| 파일 | 핸들 | 초기화 함수 | 대상 / 설정 |
|------|------|-----------|-------------|
| `spi.c/.h` | `hspi1` | `MX_SPI1_Init()` | MPU-9250, SPI mode0, ~1.3 MHz |
| `i2c.c/.h` | `hi2c1` | `MX_I2C1_Init()` | 기압계, 100 kHz |
| `usart.c/.h` | `huart1` | `MX_USART1_UART_Init()` | GPS, 9600 8N1, RX 인터럽트 |

`usart.c`에는 GPS 수신 인터럽트 처리를 위한 `USART1_IRQHandler()`도 포함.

---

## 4. MPU9250 — IMU + 지자기 드라이버
`Core/Src/MPU9250.c` · 출처 [desertkun/MPU9250](https://github.com/desertkun/MPU9250) (WTFPL)

`Sensor_read()`가 자동 호출. 세밀 제어가 필요할 때만 직접 사용.

| 함수 | 기능 |
|------|------|
| `uint8_t MPU9250_Init(void)` | IMU + 내장 AK8963 지자기 초기화 (0=성공) |
| `void MPU9250_GetData(int16_t* Acc, int16_t* Mag, int16_t* Gyro)` | raw int16 3축 (**순서: Acc, Mag, Gyro**) |
| `void MPU9250_SetAccelRange(AccelRange)` | 가속도 범위 (기본 ±16G) |
| `void MPU9250_SetGyroRange(GyroRange)` | 자이로 범위 (기본 ±250°/s) |
| `void MPU9250_SetDLPFBandwidth(DLPFBandwidth)` | 저역통과 필터 |
| `void MPU9250_SetSampleRateDivider(SampleRateDivider)` | 샘플레이트 분주 |

> **주의**: raw→단위 변환 스케일은 `sensors.c` 상단(`ACC_LSB_PER_G`, `GYR_LSB_PER_DPS`,
> `MAG_UT_PER_LSB`)에 있음. 범위를 바꾸면 이 상수도 함께 수정해야 값이 맞습니다.

---

## 5. BMXX80 — 기압계 드라이버
`Core/Src/BMXX80.c` · 출처 [lamik/BMXX80_STM32_HAL](https://github.com/lamik/BMXX80_STM32_HAL) (MIT)

`Sensor_read()`가 자동 호출. 기본 **BMP280**.

| 함수 | 기능 |
|------|------|
| `void BMP280_Init(&hi2c1, 온도해상도, 오버샘플, 모드)` | 초기화 |
| `uint8_t BMP280_ReadTemperatureAndPressure(float *t, int32_t *p)` | 온도[℃]·기압[Pa] 동시 |
| `float BMP280_ReadAltitude(float 해수면기압Pa)` | 기압 고도[m] |

> BMP180을 쓰면 `Core/Inc/BMPXX80.h` 상단의 `//#define BMP180` 주석을 해제하세요.
> `sensors.c`가 자동으로 BMP180 함수 경로로 전환됩니다.
> 모듈 I2C 주소가 0x77이면 `BMPXX80.h`의 `BMP280_I2CADDR`를 `0xEE`로 변경.

---

## 6. lwgps — GPS NMEA 파서
`Core/Src/lwgps.c` · 출처 [MaJerle/lwgps](https://github.com/MaJerle/lwgps) (MIT)

폴링이 아니라 **백그라운드 파싱**:

```
GPS → USART1 → 바이트마다 RX 인터럽트
  → HAL_UART_RxCpltCallback() [sensors.c] → lwgps_process() 누적 파싱
  → 내부 구조체 s_gps 에 최신 위치/속도 저장
Sensor_read() 가 s_gps 값을 d.gps_* 로 복사
```

| 함수 | 기능 |
|------|------|
| `uint8_t lwgps_init(lwgps_t*)` | 파서 초기화 |
| `uint8_t lwgps_process(lwgps_t*, data, len)` | 수신 바이트 파싱 (콜백에서 호출) |
| `lwgps_float_t lwgps_to_speed(속도, LWGPS_SPEED_KPH)` | 노트→km/h 등 단위 변환 |

> 실외에서 위성이 잡히기 전에는 `gps_fix=0`, `gps_valid=0` 이고 좌표는 0입니다.

---

## 7. delays — 마이크로초 지연
`Core/Src/delays.c` (DWT 기반, 타이머 불필요)

BMP**180/BME280** 경로에서만 쓰이는 µs 지연. 기본 BMP280에서는 사용되지 않으므로 신경 쓸
필요 없음. (`Delays_Init()`, `Delay_us()`)

---

## 8. 빌드 & 실행 순서

1. STM32CubeIDE에서 프로젝트 **새로고침 (F5)** — 추가된 파일 인식
2. **Project → Build Project** (`Ctrl+B`)
3. 보드에 **플래시 (Run/Debug)**
4. **시리얼 모니터 115200 8N1** 을 ST-Link 가상 COM 포트에 연결

### 출력 예시

```
=== Sensors_Init ===
IMU  MPU-9250 : OK (accel/gyro/mag)
BARO chip-ID  : 0x58 (BMP280)
GPS  NEO-6M   : USART1 RX started (9600 8N1)

---- SENSORS ----
ACC [g]    x=0.012 y=-0.004 z=1.001
GYR [dps]  x=0.31 y=-0.12 z=0.05
MAG [uT]   x=12.3 y=-30.1 z=41.8
BARO       T=24.53 C  P=101210 Pa  Alt=9.42 m  (ok=1)
GPS  lat=0.000000 lon=0.000000
     alt=0.0 m  spd=0.00 km/h  fix=0 sats=0 valid=0
```

부팅 직후 `=== Sensors_Init ===` 블록에서 각 센서 감지 결과(특히 기압 칩 ID)를 확인하면
배선이 맞는지 바로 알 수 있습니다.

---

## 9. 바꿔야 할 수도 있는 설정 요약

| 항목 | 위치 | 기본값 |
|------|------|--------|
| 기압 칩(280/180) | `BMPXX80.h` 상단 `#define` | BMP280 |
| 기압 I2C 주소 | `BMPXX80.h` `BMP280_I2CADDR` | 0xEC(0x76) |
| SPI 모드 (IMU 이상 시) | `spi.c` CLKPolarity/CLKPhase | mode0 |
| GPS 보드레이트 | `usart.c` `BaudRate` | 9600 |
| 고도 기준 해수면 기압 | `sensors.h` `SEA_LEVEL_PRESSURE_PA` | 101325 |
| IMU 범위 + 스케일 | `Sensors_Init()` + `sensors.c` 스케일 상수 | ±16G / ±250°/s |

> ⚠️ CubeMX(.ioc)에서 코드를 **재생성하면** `stm32f4xx_hal_conf.h`가 덮어써져 SPI/I2C 모듈이
> 다시 꺼집니다. 재생성했다면 `HAL_SPI_MODULE_ENABLED`, `HAL_I2C_MODULE_ENABLED`를 다시 켜세요.

---

## 10. 파일 구성

```
Core/Inc/
  sensors.h              ← 메인 API + SensorData_t
  spi.h  i2c.h  usart.h  ← 버스 핸들/초기화
  gpio.h                 ← 서드파티 호환용 스텁
  MPU9250.h  MPU9250_Config.h
  BMPXX80.h
  delays.h
  lwgps/  (lwgps.h, lwgps_opt.h, lwgps_opts.h)
Core/Src/
  sensors.c              ← 통합 모듈 (여기서 각 라이브러리 호출)
  spi.c  i2c.c  usart.c
  MPU9250.c  BMXX80.c  delays.c  lwgps.c
Drivers/STM32F4xx_HAL_Driver/  (stm32f4xx_hal_spi/i2c*.c/.h 추가됨)
```
