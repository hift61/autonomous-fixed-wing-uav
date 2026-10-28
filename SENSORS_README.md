# fixed_wing 센서 통합 모듈 (Sensor_read)

핀 배치를 토대로 IMU · 지자기 · GPS · 기압 센서를 **한 번의 호출**로 읽는 모듈을
추가했습니다. 데이터는 `SensorData_t` 구조체에 사람이 읽을 수 있는 단위로 담깁니다.

## 사용법 (main.c 에 이미 연결됨)

```c
#include "sensors.h"

SensorData_t g_sensors;

Sensors_Init();                 // SPI1/I2C1/USART1 + 모든 센서 초기화 (1회)
while (1) {
    Sensor_read(&g_sensors);    // IMU+MAG+BARO + 최신 GPS 값을 한 번에 채움
    Sensor_print(&g_sensors);   // USART2(ST-Link 가상COM, 115200)로 보기 좋게 출력
    HAL_Delay(500);
}
```

`SensorData_t` 필드: `accel_g[3]`(g), `gyro_dps[3]`(°/s), `mag_uT[3]`(µT),
`baro_temp_c`(℃), `baro_press_pa`(Pa), `baro_alt_m`(m),
`gps_lat/gps_lon`(°), `gps_alt_m`(m), `gps_speed_kmh`(km/h), `gps_fix`, `gps_sats`, `gps_valid`,
그리고 상태 플래그 `imu_ok`, `baro_ok`.

## 배선 (핀 배치 기준)

| 센서 | 버스 | 핀 |
|------|------|-----|
| MPU-9250 (IMU + AK8963 지자기) | SPI1 | SCK=PA5, MISO=PA6, MOSI=PA7, **CS=PB6**, INT=PB0 |
| BMP280 / BMP180 (기압) | I2C1 | SCL=PB8, SDA=PB9 |
| NEO-6M (GPS) | USART1 | GPS_TX→PA10(RX), GPS_RX←PA9(TX), 9600 8N1 |
| 디버그 출력 | USART2 | ST-Link 가상 COM, 115200 |

## 사용한 오픈소스 라이브러리 (GitHub)

| 센서 | 라이브러리 | 라이선스 | 파일 |
|------|-----------|---------|------|
| MPU-9250 | [desertkun/MPU9250](https://github.com/desertkun/MPU9250) | WTFPL | `Core/Src/MPU9250.c`, `Core/Inc/MPU9250.h`, `Core/Inc/MPU9250_Config.h` |
| BMP280/180 | [lamik/BMXX80_STM32_HAL](https://github.com/lamik/BMXX80_STM32_HAL) | MIT | `Core/Src/BMXX80.c`, `Core/Inc/BMPXX80.h` |
| GPS(NMEA) | [MaJerle/lwgps](https://github.com/MaJerle/lwgps) | MIT | `Core/Src/lwgps.c`, `Core/Inc/lwgps/*` |

합성 모듈(`Core/Src/sensors.c`, `Core/Inc/sensors.h`)이 위 라이브러리를 호출합니다.
버스 초기화는 `Core/Src/{spi,i2c,usart}.c` 에 손으로 작성했습니다(핀 AF는 이미
`MX_GPIO_Init()`에서 설정됨). `delays.c`는 BMP180/BME280 경로 전용 µs 지연으로,
타이머 없이 동작하도록 DWT 기반으로 교체했습니다.

## 바꿔야 할 수도 있는 설정

- **기압 칩**: 기본 **BMP280**. 부팅 시 USART2로 감지된 칩 ID를 출력합니다.
  - `0x58`=BMP280(정상), `0x55`=BMP180, `0x60`=BME280.
  - BMP180이면 `Core/Inc/BMPXX80.h` 상단의 `//#define BMP180` 주석을 해제하세요
    (그러면 `sensors.c`가 자동으로 BMP180 경로를 씁니다).
  - 모듈 I2C 주소가 0x77이면 `BMPXX80.h`의 `BMP280_I2CADDR`를 `0xEE`로 바꾸세요.
- **SPI 모드**: `Core/Src/spi.c`에서 mode 0(CPOL_LOW/CPHA_1EDGE)로 설정. IMU 값이
  이상하면 mode 3(POLARITY_HIGH + PHASE_2EDGE)로 시도.
- **GPS 속도**: `Core/Src/usart.c`의 `BaudRate = 9600`. M8N 등은 38400.
- **고도 기준 기압**: `Core/Inc/sensors.h`의 `SEA_LEVEL_PRESSURE_PA`(기본 101325).
- **IMU 레인지**: 기본 가속도 ±16g, 자이로 ±250°/s. 바꾸려면 `Sensors_Init()`에서
  `MPU9250_SetAccelRange()/SetGyroRange()` 호출 후 `sensors.c`의 스케일 상수도 함께 수정.

## 프로젝트에 가한 변경

- `stm32f4xx_hal_conf.h`: `HAL_SPI_MODULE_ENABLED`, `HAL_I2C_MODULE_ENABLED` 활성화.
- `Drivers/STM32F4xx_HAL_Driver`에 `stm32f4xx_hal_spi.*`, `stm32f4xx_hal_i2c*.*` 추가
  (FW_F4 V1.28.3에서 복사, HAL 버전 동일).
- `main.h`: `GY_CS_Pin/GY_CS_GPIO_Port`(PB6) 정의.
- `main.c`: USER CODE 영역에만 `Sensors_Init()`/`Sensor_read()`/`Sensor_print()` 연결.

## ⚠️ CubeMX(.ioc) 재생성 시 주의

SPI1/I2C1/USART1은 .ioc에 "핀 신호"로만 있고 페리페럴로는 안 켜져 있어서, 초기화를
직접 작성했습니다. **CubeMX에서 코드를 다시 생성하면 `stm32f4xx_hal_conf.h`가
덮어써져 SPI/I2C 모듈이 다시 꺼집니다.** 재생성했다면 위 두 `#define`을 다시 켜세요.
(USER CODE 영역의 호출과 `sensors.c` 등 별도 파일은 보존됩니다.)

## 빌드 확인

arm-none-eabi-gcc 14.3(프로젝트 툴체인)으로 전체 컴파일·링크 검증 완료
(경고 0 / 에러 0, ELF: text≈26KB, RAM≈2.6KB — F410RB 여유 충분).
STM32CubeIDE에서 프로젝트 새로고침(F5) 후 빌드하세요.
