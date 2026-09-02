#include "imu.h"
#include <stddef.h>                     // NULL 사용


static SPI_HandleTypeDef *imu_hspi;     // IMU에 사용할 SPI 정보
static GPIO_TypeDef *imu_cs_port;       // NCS가 속한 GPIO 포트
static uint16_t imu_cs_pin;             // NCS의 GPIO 핀
static uint8_t imu_device_id;           // 읽어온 IMU 식별번호

static float gyro_bias_rps[3] = {       // X, Y, Z축 자이로 영점 오차
    0.0f,
    0.0f,
    0.0f
};

static bool imu_initialized = false;    // IMU 초기화 성공 여부

#define MPU_REG_I2C_MST_CTRL     0x24U  // 내부 I²C 속도 설정
#define MPU_REG_I2C_SLV4_ADDR    0x31U  // 보조 센서 I²C 주소
#define MPU_REG_I2C_SLV4_REG     0x32U  // 보조 센서 레지스터 주소
#define MPU_REG_I2C_SLV4_DO      0x33U  // 보조 센서에 쓸 값
#define MPU_REG_I2C_SLV4_CTRL    0x34U  // 보조 통신 시작 설정
#define MPU_REG_I2C_SLV4_DI      0x35U  // 보조 센서에서 읽은 값
#define MPU_REG_I2C_MST_STATUS   0x36U  // 내부 I²C 통신 상태

#define MPU_I2C_READ_FLAG        0x80U  // 내부 I²C 읽기 표시
#define MPU_I2C_SLV4_ENABLE      0x80U  // SLV4 통신 시작
#define MPU_I2C_SLV4_DONE        0x40U  // SLV4 통신 완료
#define MPU_I2C_LOST_ARB         0x20U  // 내부 I²C 충돌 오류
#define MPU_I2C_SLV4_NACK        0x10U  // 보조 센서 응답 없음

#define MPU_ENABLE_I2C_MASTER    0x20U  // 내부 I²C Master 활성화
#define MPU_USER_CTRL_SPI_AUX    0x30U  // SPI + 내부 I²C 활성화
#define MPU_I2C_MST_CLOCK_400KHZ 0x0DU  // 내부 I²C 약 400kHz
#define MPU_AUX_TIMEOUT_MS       10U    // 내부 I²C 최대 대기시간

#define MPU_REG_SMPLRT_DIV       0x19U  // 측정 주기 분할 설정
#define MPU_REG_CONFIG           0x1AU  // 자이로 내부 필터 설정
#define MPU_REG_GYRO_CONFIG      0x1BU  // 자이로 측정 범위 설정
#define MPU_REG_ACCEL_CONFIG     0x1CU  // 가속도계 측정 범위 설정
#define MPU_REG_ACCEL_CONFIG2    0x1DU  // 가속도계 내부 필터 설정
#define MPU_REG_INT_ENABLE       0x38U  // 인터럽트 활성화 설정
#define MPU_REG_ACCEL_XOUT_H     0x3BU  // 가속도 X축 상위 바이트 주소
#define MPU_REG_USER_CTRL        0x6AU  // SPI와 내부 기능 설정
#define MPU_REG_PWR_MGMT_1       0x6BU  // 전원 및 클록 설정
#define MPU_REG_PWR_MGMT_2       0x6CU  // 센서 축 활성화 설정
#define MPU_REG_WHO_AM_I         0x75U  // 장치 식별번호 주소

#define MPU_SPI_READ_FLAG        0x80U  // SPI 읽기 표시
#define MPU_SPI_WRITE_MASK       0x7FU  // SPI 쓰기용 최상위 비트 제거

#define MPU_DEVICE_ID_9250       0x71U  // MPU-9250 식별번호
#define MPU_DEVICE_ID_6500       0x70U  // MPU-6500 식별번호

#define MPU_PWR_RESET            0x80U  // IMU 초기화 명령
#define MPU_CLOCK_PLL_XGYRO      0x01U  // 자이로 X축 PLL 클록 사용
#define MPU_DISABLE_PRIMARY_I2C  0x10U  // 주 통신을 SPI로 사용

#define MPU_GYRO_FS_500DPS       0x08U  // 자이로 범위 ±500°/s
#define MPU_ACCEL_FS_8G          0x10U  // 가속도 범위 ±8g
#define MPU_DLPF_CONFIG          0x03U  // 내부 저역통과 필터 설정값
#define MPU_SAMPLE_DIVIDER       0x01U  // 약 500Hz 측정 설정
#define MPU_DATA_READY_ENABLE    0x01U  // 새 데이터 준비 인터럽트 활성화

#define MPU_BURST_DATA_LENGTH    14U    // 가속도·온도·자이로 총 14바이트
#define MPU_MAX_BURST_LENGTH     14U    // 한 번에 읽을 최대 바이트 수
#define IMU_SPI_TIMEOUT_MS       10U    // SPI 최대 대기시간

#define MPU_ACCEL_LSB_PER_G      4096.0f  // ±8g 설정의 변환값
#define MPU_GYRO_LSB_PER_DPS     65.5f    // ±500°/s 설정의 변환값

#define STANDARD_GRAVITY_MPS2    9.80665f     // 중력가속도
#define DEG_TO_RAD               0.01745329252f // degree를 radian으로 변환


static void IMU_Select(void)
{
    HAL_GPIO_WritePin(
        imu_cs_port,                    // NCS가 속한 GPIO 포트
        imu_cs_pin,                     // NCS 핀
        GPIO_PIN_RESET                  // Low로 내려 통신 시작
    );
}


static void IMU_Deselect(void)
{
    HAL_GPIO_WritePin(
        imu_cs_port,                    // NCS가 속한 GPIO 포트
        imu_cs_pin,                     // NCS 핀
        GPIO_PIN_SET                    // High로 올려 통신 종료
    );
}


static bool IMU_WriteRegister(
    uint8_t reg,                        // 값을 쓸 레지스터 주소(어떤 기능을 조정할 것인지)
    uint8_t value                       // 레지스터에 저장할 값
)
{
    uint8_t tx_data[2] = {              // 주소와 설정값을 담을 배열
        reg & MPU_SPI_WRITE_MASK,       // 최상위 비트를 0으로 만들어 쓰기 설정
		/*  절전 상태에서 깨우기
			가속도계와 자이로스코프 활성화
			측정 범위 설정
			측정 주기 설정
			노이즈 필터 설정
			인터럽트 설정
			내부 자기계 통신 활성화 -> 이런 명령을 imu에 넣어야 해서 writeregister 함수가 필요함
		 */

        value                           // 레지스터에 기록할 값
    };

    IMU_Select();                       // SPI 통신 시작

    HAL_StatusTypeDef status =          // SPI 실행 결과 저장
        HAL_SPI_Transmit(
            imu_hspi,                   // 사용할 SPI1 정보
            tx_data,                    // 보낼 데이터
            2U,                         // 보낼 바이트 수
            IMU_SPI_TIMEOUT_MS          // 최대 대기시간
        );

    IMU_Deselect();                     // SPI 통신 종료

    if (status != HAL_OK)               // SPI 통신 실패 확인
    {
        return false;
    }

    return true;
}



static bool IMU_ReadRegisters(
    uint8_t start_reg,                  // 읽기 시작할 레지스터 주소
    uint8_t *data,                      // 읽은 값을 저장할 배열
    uint16_t length                     // 읽을 바이트 수
)
{
    if ((data == NULL) ||               // 저장할 배열이 없는지 확인
        (length == 0U) ||               // 읽을 길이가 0인지 확인
        (length > MPU_MAX_BURST_LENGTHMPU_MAX_BURST_LENGTH))// 최대 길이를 넘었는지 확인
    {
        return false;
    }



    uint8_t tx_data[MPU_MAX_BURST_LENGTH + 1U] = {0U};
    // 첫 바이트는 주소, 나머지는 값을 받기 위한 빈 데이터
    // 최대 15바이트 크기의 송신 배열을 만들고, 모든 칸을 0으로 초기화

    uint8_t rx_data[MPU_MAX_BURST_LENGTH + 1U] = {0U};
    // 첫 바이트는 불필요한 값, 나머지는 실제 센서값

    tx_data[0] = start_reg | MPU_SPI_READ_FLAG;
    // 시작 주소에 SPI 읽기 표시를 추가, 어디서부터 읽을지

    IMU_Select();                       // SPI 통신 시작

    HAL_StatusTypeDef status =          // SPI 실행 결과 저장
        HAL_SPI_TransmitReceive(
            imu_hspi,                   // 사용할 SPI1 정보
            tx_data,                    // 보낼 주소와 빈 데이터
            rx_data,                    // 받은 데이터를 저장할 배열
            length + 1U,                // 주소 1바이트를 포함한 전체 길이
            IMU_SPI_TIMEOUT_MS          // 최대 대기시간
        );

    IMU_Deselect();                     // SPI 통신 종료

    if (status != HAL_OK)               // SPI 통신 실패 확인
    {
        return false;
    }

    for (uint16_t i = 0U; i < length; i++)
    {
        data[i] = rx_data[i + 1U];      // 첫 번째 불필요한 바이트를 제외하고 복사
    }

    return true;
}

static bool IMU_AuxWaitForCompletion(void) // MPU가 자기계와 수행하는 내부 I²C 통신이 끝날 때까지 기다리는 함수
// 이번 자기계 통신 하나가 제대로 끝났는지 확인하고, 성공했을 때만 다음 작업으로 넘어가게 하는 검사 함수
{
    uint32_t start_time = HAL_GetTick();  // 대기 시작 시각

    while (true)
    {
        uint8_t status = 0U;              // 내부 I²C 상태 저장

        if (!IMU_ReadRegisters(
                MPU_REG_I2C_MST_STATUS,
                &status,
                1U))
        {
            return false;
        }

        if ((status & MPU_I2C_SLV4_NACK) != 0U)
        {
            return false;                 // 보조 센서가 응답하지 않음
        }

        if ((status & MPU_I2C_LOST_ARB) != 0U)
        {
            return false;                 // 내부 I²C 통신 충돌
        }

        if ((status & MPU_I2C_SLV4_DONE) != 0U)
        {
            return true;                  // 내부 I²C 통신 완료
        }

        if ((HAL_GetTick() - start_time) >=
            MPU_AUX_TIMEOUT_MS)
        {
            return false;                 // 제한시간 초과
        }
    }
}

bool IMU_AuxWriteRegister( // STM32가 MPU를 거쳐 자기계에 설정 명령을 전달하는 함수
    uint8_t device_address,
    uint8_t reg,
    uint8_t value
)
{
    if (!imu_initialized)
    {
        return false;
    }

    if (!IMU_WriteRegister(
            MPU_REG_I2C_SLV4_ADDR,
            device_address & 0x7FU))      // 쓰기이므로 읽기 비트를 0으로 설정
    {
        return false;
    }

    if (!IMU_WriteRegister(
            MPU_REG_I2C_SLV4_REG,
            reg))                         // 값을 기록할 AK8963 레지스터
    {
        return false;
    }

    if (!IMU_WriteRegister(
            MPU_REG_I2C_SLV4_DO,
            value))                       // AK8963에 기록할 값
    {
        return false;
    }

    if (!IMU_WriteRegister(
            MPU_REG_I2C_SLV4_CTRL,
            MPU_I2C_SLV4_ENABLE))         // 내부 I²C 쓰기 시작
    {
        return false;
    }

    return IMU_AuxWaitForCompletion();    // 통신 완료까지 대기
}

bool IMU_AuxReadRegisters(
    uint8_t device_address,
    uint8_t start_reg,
    uint8_t *data,
    uint8_t length
)
{
    if ((!imu_initialized) ||
        (data == NULL) ||
        (length == 0U))
    {
        return false;
    }

    for (uint8_t i = 0U; i < length; i++)
    {
        if (!IMU_WriteRegister(
                MPU_REG_I2C_SLV4_ADDR,
                device_address |
                MPU_I2C_READ_FLAG))       // 내부 I²C 읽기 설정
        {
            return false;
        }

        if (!IMU_WriteRegister(
                MPU_REG_I2C_SLV4_REG,
                start_reg + i))           // 이번에 읽을 레지스터 주소
        {
            return false;
        }

        if (!IMU_WriteRegister(
                MPU_REG_I2C_SLV4_CTRL,
                MPU_I2C_SLV4_ENABLE))     // 내부 I²C 읽기 시작
        {
            return false;
        }

        if (!IMU_AuxWaitForCompletion())
        {
            return false;
        }

        if (!IMU_ReadRegisters(
                MPU_REG_I2C_SLV4_DI,
                &data[i],
                1U))                      // 읽은 값을 배열에 저장
        {
            return false;
        }
    }

    return true;
}
//센서가 나눠 보낸 2바이트
//→ 원래의 부호 있는 16비트 측정값으로 복원
static int16_t IMU_CombineBytes(
    uint8_t high_byte,                  // 측정값의 상위 1바이트
    uint8_t low_byte                    // 측정값의 하위 1바이트
)
{
    return (int16_t)(
        ((uint16_t)high_byte << 8U) |   // 상위 바이트를 왼쪽으로 8비트 이동
        (uint16_t)low_byte              // 하위 바이트와 결합
    );
}


bool IMU_Init(
    SPI_HandleTypeDef *hspi,            // main.c에서 전달한 SPI1 주소
    GPIO_TypeDef *cs_port,              // NCS가 속한 GPIO 포트
    uint16_t cs_pin                     // NCS 핀
)
{
    if ((hspi == NULL) ||               // SPI 주소가 없는지 확인
        (cs_port == NULL))              // GPIO 포트가 없는지 확인
    {
        return false;
    }

    imu_initialized = false;            // 초기화 진행 중 상태

    imu_hspi = hspi;                    // SPI1 주소 저장
    imu_cs_port = cs_port;              // NCS GPIO 포트 저장
    imu_cs_pin = cs_pin;                // NCS GPIO 핀 저장

    IMU_Deselect();                     // NCS를 기본 High로 설정
    HAL_Delay(10U);                     // 전원 안정화를 위해 10ms 대기

    if (!IMU_WriteRegister(
            MPU_REG_USER_CTRL,
            MPU_USER_CTRL_SPI_AUX))       // SPI와 내부 I²C를 함께 활성화
    {
        return false;
    }

    if (!IMU_WriteRegister(
            MPU_REG_I2C_MST_CTRL,
            MPU_I2C_MST_CLOCK_400KHZ))    // 내부 I²C 속도 설정
    {
        return false;
    }

    if (!IMU_ReadRegisters(
            MPU_REG_WHO_AM_I,
            &imu_device_id,
            1U))                          // IMU 식별번호 1바이트 읽기
    {
        return false;
    }

    if ((imu_device_id != MPU_DEVICE_ID_9250) &&
        (imu_device_id != MPU_DEVICE_ID_6500))
    {
        return false;                   // 지원하지 않는 식별번호
    }

    if (!IMU_WriteRegister(
            MPU_REG_PWR_MGMT_1,         // 전원 관리 레지스터
            MPU_PWR_RESET))             // 전체 초기화 값
    {
        return false;
    }

    HAL_Delay(100U);                    // 초기화 완료까지 100ms 대기

    if (!IMU_WriteRegister(
            MPU_REG_PWR_MGMT_1,         // 전원과 클록 설정 레지스터
            MPU_CLOCK_PLL_XGYRO))       // 자이로 PLL 클록 선택
    {
        return false;
    }

    HAL_Delay(100U);                    // 센서가 깨어날 때까지 대기

    if (!IMU_WriteRegister(
            MPU_REG_PWR_MGMT_2,         // 센서 축 활성화 레지스터
            0x00U))                     // 가속도·자이로 모든 축 활성화
    {
        return false;
    }

    if (!IMU_WriteRegister(
            MPU_REG_USER_CTRL,          // 통신 방식 설정 레지스터
            MPU_DISABLE_PRIMARY_I2C))   // 주 통신을 SPI로 사용
    {
        return false;
    }

    if (!IMU_WriteRegister(
            MPU_REG_CONFIG,             // 자이로 내부 필터 레지스터
            MPU_DLPF_CONFIG))           // 저역통과 필터 설정값
    {
        return false;
    }

    if (!IMU_WriteRegister(
            MPU_REG_ACCEL_CONFIG2,      // 가속도계 내부 필터 레지스터
            MPU_DLPF_CONFIG))           // 저역통과 필터 설정값
    {
        return false;
    }

    if (!IMU_WriteRegister(
            MPU_REG_SMPLRT_DIV,         // 측정 주기 설정 레지스터
            MPU_SAMPLE_DIVIDER))        // 약 500Hz 출력 설정
    {
        return false;
    }

    if (!IMU_WriteRegister(
            MPU_REG_GYRO_CONFIG,        // 자이로 범위 설정 레지스터
            MPU_GYRO_FS_500DPS))        // ±500°/s 설정
    {
        return false;
    }

    if (!IMU_WriteRegister(
            MPU_REG_ACCEL_CONFIG,       // 가속도계 범위 설정 레지스터
            MPU_ACCEL_FS_8G))           // ±8g 설정
    {
        return false;
    }

    if (!IMU_WriteRegister(
            MPU_REG_INT_ENABLE,         // 인터럽트 활성화 레지스터
            MPU_DATA_READY_ENABLE))     // 새 데이터 준비 인터럽트 활성화
    {
        return false;
    }

    gyro_bias_rps[0] = 0.0f;            // X축 영점 오차 초기화
    gyro_bias_rps[1] = 0.0f;            // Y축 영점 오차 초기화
    gyro_bias_rps[2] = 0.0f;            // Z축 영점 오차 초기화

    imu_initialized = true;             // 초기화 성공 상태 저장

    return true;
}

// 받아온 데이터 우리가 알아들을 수 있게 바꿔주는 함수, 어캐 하는지는 알아서 읽으셈
bool IMU_Read(ImuSample *sample)        // 측정값을 sample 구조체에 저장
{
    if ((!imu_initialized) ||           // 초기화하지 않았는지 확인
        (sample == NULL))               // 저장할 구조체가 없는지 확인
    {
        return false;
    }

    uint8_t raw[MPU_BURST_DATA_LENGTH]; // 센서 원시값 14바이트 저장

    if (!IMU_ReadRegisters(
            MPU_REG_ACCEL_XOUT_H,       // 가속도 X축 상위 바이트부터 시작
            raw,                        // 읽은 값을 저장할 배열
            MPU_BURST_DATA_LENGTH))     // 총 14바이트 읽기
    {
        return false;
    }

    int16_t accel_raw[3];               // X, Y, Z 가속도 원시값
    int16_t gyro_raw[3];                // X, Y, Z 자이로 원시값
    int16_t temperature_raw;            // 온도 원시값

    accel_raw[0] = IMU_CombineBytes(raw[0], raw[1]);
    accel_raw[1] = IMU_CombineBytes(raw[2], raw[3]);
    accel_raw[2] = IMU_CombineBytes(raw[4], raw[5]);

    temperature_raw = IMU_CombineBytes(raw[6], raw[7]);

    gyro_raw[0] = IMU_CombineBytes(raw[8], raw[9]);
    gyro_raw[1] = IMU_CombineBytes(raw[10], raw[11]);
    gyro_raw[2] = IMU_CombineBytes(raw[12], raw[13]);

    for (uint8_t i = 0U; i < 3U; i++)   // X, Y, Z 세 축 반복
    {
        sample->accel_mps2[i] =
            ((float)accel_raw[i] /      // 원시값을 실수로 변환
             MPU_ACCEL_LSB_PER_G) *     // 원시값을 g 단위로 변환
            STANDARD_GRAVITY_MPS2;      // g를 m/s² 단위로 변환

        sample->gyro_rps[i] =
            (((float)gyro_raw[i] /      // 원시값을 실수로 변환
              MPU_GYRO_LSB_PER_DPS) *   // 초당 degree 값으로 변환
             DEG_TO_RAD) -              // 초당 radian 값으로 변환
            gyro_bias_rps[i];           // 자이로 영점 오차 제거

        sample->mag_uT[i] = 0.0f;       // 자기계는 아직 구현하지 않음
    }

    sample->temperature_c =
        ((float)temperature_raw /       // 온도 원시값을 실수로 변환
         333.87f) +
        21.0f;                          // MPU 온도 변환식 적용

    sample->time_us =
        HAL_GetTick() * 1000U;          // 임시로 ms 시간을 µs 형태로 변환

    sample->magnetometer_valid = false; // 자기계 데이터는 아직 사용할 수 없음

    return true;
}


bool IMU_CalibrateGyro(uint16_t sample_count)
{
    if ((!imu_initialized) ||           // IMU 초기화 여부 확인
        (sample_count == 0U))           // 측정 횟수가 0인지 확인
    {
        return false;
    }

    gyro_bias_rps[0] = 0.0f;            // 기존 X축 보정값 제거
    gyro_bias_rps[1] = 0.0f;            // 기존 Y축 보정값 제거
    gyro_bias_rps[2] = 0.0f;            // 기존 Z축 보정값 제거

    float gyro_sum[3] = {               // 각 축의 측정값 합계
        0.0f,
        0.0f,
        0.0f
    };

    ImuSample sample;                   // 한 번의 IMU 측정값을 저장

    for (uint16_t i = 0U; i < sample_count; i++)
    {
        if (!IMU_Read(&sample))         // IMU 측정 실패 확인
        {
            return false;
        }

        gyro_sum[0] += sample.gyro_rps[0]; // X축 측정값 누적
        gyro_sum[1] += sample.gyro_rps[1]; // Y축 측정값 누적
        gyro_sum[2] += sample.gyro_rps[2]; // Z축 측정값 누적

        HAL_Delay(2U);                  // 다음 측정까지 2ms 대기
    }

    gyro_bias_rps[0] =
        gyro_sum[0] / (float)sample_count; // X축 평균값 저장

    gyro_bias_rps[1] =
        gyro_sum[1] / (float)sample_count; // Y축 평균값 저장

    gyro_bias_rps[2] =
        gyro_sum[2] / (float)sample_count; // Z축 평균값 저장

    return true;
}


uint8_t IMU_GetDeviceId(void)
{
    return imu_device_id;               // 초기화할 때 읽어둔 식별번호 반환
}
