/*
 * eskf.h  -  Attitude Error-State Kalman Filter (6-state) for fixed_wing.
 *
 *   공칭 상태:  q (쿼터니언, body->world),  bg (자이로 바이어스)
 *   오차 상태:  [ dtheta(3), dbias(3) ]  -> P 는 6x6
 *   예측:  자이로
 *   보정:  가속도(중력 -> roll/pitch),  지자기(-> yaw)
 *
 *   world 좌표계 = ENU 성격 (Z = 위쪽), 중력 기준 up = [0,0,1].
 *   yaw 는 시동 시점의 (기울기 보정된) 자기 방위에 정렬됩니다.
 *   -> 안정적인 상대 방위 기준이며, 편각(declination)은 적용하지 않습니다.
 *
 *   이것은 15상태 INS/GPS 필터의 자세 부분입니다. 검증 후 위치/속도로 확장.
 *
 *   사용:
 *      ESKF_t ekf;
 *      Sensor_read(&data);
 *      ESKF_Init(&ekf, data.accel_g, data.mag_uT);   // 정지 상태에서 1회
 *      while (1) {
 *          Sensor_read(&data);
 *          ESKF_Step(&ekf, &data, dt);                // dt [s]
 *          ESKF_GetEuler(&ekf, &roll, &pitch, &yaw);  // [deg]
 *      }
 */
#ifndef ESKF_H_
#define ESKF_H_
#ifdef __cplusplus
extern "C" {
#endif

#include "main.h"
#include "sensors.h"
#include <stdint.h>

typedef struct
{
    /* ---- nominal state ---- */
    float q[4];       /* attitude quaternion [w,x,y,z], body -> world */
    float bg[3];      /* gyro bias [rad/s] */

    /* ---- error covariance (6x6, row-major) ---- */
    float P[36];

    /* ---- references (world frame, unit vectors) ---- */
    float g_ref[3];   /* gravity "up" reference = {0,0,1} */
    float m_ref[3];   /* magnetic reference, set at init */

    /* ---- tuning (variances; dt is applied inside Predict) ---- */
    float q_gyro;     /* process noise on angle  (per second) */
    float q_bias;     /* process noise on bias   (per second) */
    float r_acc;      /* accelerometer meas. noise (normalized) */
    float r_mag;      /* magnetometer meas. noise  (normalized) */
    float acc_gate;   /* skip accel update if |accel|-1g exceeds this (in g) */
    uint8_t use_mag;  /* 1 = run magnetometer (yaw) update */
    uint8_t ready;    /* 1 = ESKF_Init has run */
} ESKF_t;

/* Initialise attitude from a stationary accel (+ mag) sample and set defaults. */
void ESKF_Init(ESKF_t *e, const float acc_g[3], const float mag_uT[3]);

/* Prediction with a gyro sample [rad/s] over dt [s]. */
void ESKF_Predict(ESKF_t *e, const float gyro_rads[3], float dt);

/* Correction from accelerometer [g] (gravity direction -> roll/pitch). */
void ESKF_UpdateAccel(ESKF_t *e, const float acc_g[3]);

/* Correction from magnetometer [uT] (-> yaw). */
void ESKF_UpdateMag(ESKF_t *e, const float mag_uT[3]);

/* Convenience: unit-convert a SensorData_t snapshot and run predict + updates. */
void ESKF_Step(ESKF_t *e, const SensorData_t *s, float dt);

/* Read current attitude as Euler angles [deg]. */
void ESKF_GetEuler(const ESKF_t *e, float *roll_deg, float *pitch_deg, float *yaw_deg);

#ifdef __cplusplus
}
#endif
#endif /* ESKF_H_ */
