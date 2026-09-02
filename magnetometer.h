#ifndef MAGNETOMETER_H
#define MAGNETOMETER_H

#include <stdbool.h>
#include <stdint.h>


typedef struct
{
    float field_uT[3];                   // X, Y, Z축 자기장 [µT]
    uint32_t time_us;                    // 측정 시각 [µs]
    bool valid;                          // 측정값 사용 가능 여부
} MagnetometerSample;


bool Magnetometer_Init(void);            // AK8963 초기화

bool Magnetometer_Read(
    MagnetometerSample *sample           // 측정값을 저장할 구조체 주소
);

bool Magnetometer_IsAvailable(void);     // 자기계 존재 여부

uint8_t Magnetometer_GetDeviceId(void);  // AK8963 식별번호


#endif
