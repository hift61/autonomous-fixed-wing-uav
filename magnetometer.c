#include "magnetometer.h"
#include "imu.h"
#include "stm32f4xx_hal.h"
#include <stddef.h>


#define AK8963_I2C_ADDRESS       0x0CU   // AK8963 내부 I²C 주소
#define AK8963_REG_WIA           0x00U   // 장치 식별번호 레지스터
#define AK8963_REG_ST1           0x02U   // 새 데이터 준비 상태
#define AK8963_REG_CNTL1         0x0AU   // 동작 모드 설정
#define AK8963_REG_ASAX          0x10U   // X축 공장 보정값 시작 주소

#define AK8963_DEVICE_ID         0x48U   // AK8963 식별번호
#define AK8963_MODE_POWER_DOWN   0x00U   // 절전 모드
#define AK8963_MODE_FUSE_ROM     0x0FU   // 공장 보정값 읽기 모드
#define AK8963_MODE_CONT_100HZ   0x16U   // 16비트, 연속 측정 100Hz

#define AK8963_DATA_READY        0x01U   // 새 측정값 준비 표시
#define AK8963_OVERFLOW          0x08U   // 자기장 측정 범위 초과
#define AK8963_DATA_LENGTH       8U      // ST1부터 ST2까지 8바이트
#define AK8963_SCALE_UT          0.15f   // 16비트 모드의 µT 변환값


static bool magnetometer_available = false;
static uint8_t magnetometer_device_id = 0U;

static float magnetometer_adjustment[3] = {
    1.0f,
    1.0f,
    1.0f
};


static int16_t Magnetometer_CombineBytes(
    uint8_t low_byte,
    uint8_t high_byte
)
{
    return (int16_t)(
        ((uint16_t)high_byte << 8U) |
        (uint16_t)low_byte
    );
}


bool Magnetometer_Init(void)
{
    magnetometer_available = false;

    if (IMU_GetDeviceId() !=
        MPU_DEVICE_ID_9250)
    {
        return false;                    // MPU-6500에는 자기계가 없음
    }

    if (!IMU_AuxReadRegisters(
            AK8963_I2C_ADDRESS,
            AK8963_REG_WIA,
            &magnetometer_device_id,
            1U))
    {
        return false;
    }

    if (magnetometer_device_id !=
        AK8963_DEVICE_ID)
    {
        return false;                    // AK8963 식별번호 불일치
    }

    if (!IMU_AuxWriteRegister(
            AK8963_I2C_ADDRESS,
            AK8963_REG_CNTL1,
            AK8963_MODE_POWER_DOWN))
    {
        return false;
    }

    HAL_Delay(10U);

    if (!IMU_AuxWriteRegister(
            AK8963_I2C_ADDRESS,
            AK8963_REG_CNTL1,
            AK8963_MODE_FUSE_ROM))
    {
        return false;
    }

    HAL_Delay(10U);

    uint8_t adjustment_raw[3];

    if (!IMU_AuxReadRegisters(
            AK8963_I2C_ADDRESS,
            AK8963_REG_ASAX,
            adjustment_raw,
            3U))
    {
        return false;
    }

    for (uint8_t i = 0U; i < 3U; i++)
    {
        magnetometer_adjustment[i] =
            (((float)adjustment_raw[i] -
              128.0f) /
             256.0f) +
            1.0f;
    }

    if (!IMU_AuxWriteRegister(
            AK8963_I2C_ADDRESS,
            AK8963_REG_CNTL1,
            AK8963_MODE_POWER_DOWN))
    {
        return false;
    }

    HAL_Delay(10U);

    if (!IMU_AuxWriteRegister(
            AK8963_I2C_ADDRESS,
            AK8963_REG_CNTL1,
            AK8963_MODE_CONT_100HZ))
    {
        return false;
    }

    HAL_Delay(10U);

    magnetometer_available = true;

    return true;
}


bool Magnetometer_Read(
    MagnetometerSample *sample
)
{
    if ((!magnetometer_available) ||
        (sample == NULL))
    {
        return false;
    }

    uint8_t raw[AK8963_DATA_LENGTH];

    if (!IMU_AuxReadRegisters(
            AK8963_I2C_ADDRESS,
            AK8963_REG_ST1,
            raw,
            AK8963_DATA_LENGTH))
    {
        sample->valid = false;
        return false;
    }

    if ((raw[0] & AK8963_DATA_READY) == 0U)
    {
        sample->valid = false;           // 아직 새 측정값이 없음
        return false;
    }

    if ((raw[7] & AK8963_OVERFLOW) != 0U)
    {
        sample->valid = false;           // 자기장 측정 범위 초과
        return false;
    }

    int16_t raw_x =
        Magnetometer_CombineBytes(
            raw[1],
            raw[2]
        );

    int16_t raw_y =
        Magnetometer_CombineBytes(
            raw[3],
            raw[4]
        );

    int16_t raw_z =
        Magnetometer_CombineBytes(
            raw[5],
            raw[6]
        );

    sample->field_uT[0] =
        (float)raw_x *
        AK8963_SCALE_UT *
        magnetometer_adjustment[0];

    sample->field_uT[1] =
        (float)raw_y *
        AK8963_SCALE_UT *
        magnetometer_adjustment[1];

    sample->field_uT[2] =
        (float)raw_z *
        AK8963_SCALE_UT *
        magnetometer_adjustment[2];

    sample->time_us =
        HAL_GetTick() * 1000U;           // 임시 측정 시각

    sample->valid = true;

    return true;
}


bool Magnetometer_IsAvailable(void)
{
    return magnetometer_available;
}


uint8_t Magnetometer_GetDeviceId(void)
{
    return magnetometer_device_id;
}
