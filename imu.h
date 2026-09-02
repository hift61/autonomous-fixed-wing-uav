#ifndef IMU_H
#define IMU_H

#include "stm32f4xx_hal.h"
#include <stdbool.h>
#include <stdint.h>

typedef struct
{
	float accel_mps2[3]; //가속도
	float gyro_rps[3]; // 자이로 세 방향의 회전속도를 저장, XYZ축 3개
	float mag_uT[3]; // 세 방향의 자기장의 세기를 저장한다.
	float temperature_c; // 칩 내부 온도를 저장
	uint32_t time_us; // IMU 측정이 이루어진 시간을 마이크로초 단위로 저장함
	bool magnetometer_valid; // 자기계 측정값을 사용할 수 있는지 저장함


} ImuSample;

bool IMU_Init(
	// SPI_HandleTypeDef, GPIO_typeDef 둘 다 STM32 HAL에서 제공하는 자료형
    SPI_HandleTypeDef *hspi,
    GPIO_TypeDef *cs_port, // NCS핀이 어느 GPIO 포트에 속하는지 전달받겠다 라는 뜻
	uint16_t cs_pin // HAL이 GPIO 핀 정보를 16bit로 표현함
	//GPIO_TypeDef는 GPIOB, cs_pin은 GPIO_PIN_6 이런걸 전달함
);

bool IMU_CalibrateGyro(uint16_t sample_count); // 자이로스코프의 영점 오차를 측정하는 함수 선언
// sample_count : 값을 보정할 떄 몇번 측정할지 전달
bool IMU_Read(ImuSample *sample); // imusampl이 있는 주소를 sample이라는 이름으로 받겟다
// ImuSample은 그냥 자료형 이름임
// IMU의 최신 측정값을 읽는 함수를 전언, 측정 결과를 저장할 구조체의 주소

uint8_t IMU_GetDeviceId(void);// IMU내부의 식별 번호를 반환받는다


bool IMU_AuxWriteRegister(
    uint8_t device_address,          // 내부 I²C 장치 주소
    uint8_t reg,                     // 값을 쓸 레지스터 주소
    uint8_t value                    // 기록할 값
);

bool IMU_AuxReadRegisters(
    uint8_t device_address,          // 내부 I²C 장치 주소
    uint8_t start_reg,               // 읽기 시작할 레지스터
    uint8_t *data,                   // 읽은 값을 저장할 배열
    uint8_t length                   // 읽을 바이트 수
);

#endif
