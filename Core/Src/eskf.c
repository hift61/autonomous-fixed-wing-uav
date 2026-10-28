/*
 * eskf.c  -  6-state attitude Error-State Kalman Filter. See eskf.h.
 *
 * Convention:  q = [w,x,y,z] rotates body -> world (world Z = up).
 *              error state dx = [dtheta(3), dbias(3)], body-frame (local) error
 *              so that  q_true = q (x) dq,  dq ~= [1, 0.5*dtheta].
 */
#include "eskf.h"
#include <math.h>
#include <string.h>

#define DEG2RAD   0.017453292519943295f
#define RAD2DEG   57.29577951308232f

/* ===================== small linear algebra ===================== */

/* C(ar x bc) = A(ar x ac) * B(ac x bc)  (row-major) */
static void matmul(const float *A, int ar, int ac,
                   const float *B, int bc, float *C)
{
    for (int i = 0; i < ar; i++)
        for (int j = 0; j < bc; j++)
        {
            float s = 0.0f;
            for (int k = 0; k < ac; k++)
                s += A[i * ac + k] * B[k * bc + j];
            C[i * bc + j] = s;
        }
}

/* T(c x r) = A(r x c)^T */
static void mattrans(const float *A, int r, int c, float *T)
{
    for (int i = 0; i < r; i++)
        for (int j = 0; j < c; j++)
            T[j * r + i] = A[i * c + j];
}

/* 3x3 inverse (row-major). returns 0 on success. */
static int mat3_inv(const float *M, float *Inv)
{
    float a = M[0], b = M[1], c = M[2];
    float d = M[3], e = M[4], f = M[5];
    float g = M[6], h = M[7], i = M[8];
    float A =  (e * i - f * h);
    float B = -(d * i - f * g);
    float C =  (d * h - e * g);
    float det = a * A + b * B + c * C;
    if (fabsf(det) < 1e-12f) return 1;
    float id = 1.0f / det;
    Inv[0] = A * id;
    Inv[1] = -(b * i - c * h) * id;
    Inv[2] =  (b * f - c * e) * id;
    Inv[3] = B * id;
    Inv[4] =  (a * i - c * g) * id;
    Inv[5] = -(a * f - c * d) * id;
    Inv[6] = C * id;
    Inv[7] = -(a * h - b * g) * id;
    Inv[8] =  (a * e - b * d) * id;
    return 0;
}

static void skew(const float v[3], float M[9])
{
    M[0] = 0;     M[1] = -v[2]; M[2] =  v[1];
    M[3] =  v[2]; M[4] = 0;     M[5] = -v[0];
    M[6] = -v[1]; M[7] =  v[0]; M[8] = 0;
}

/* ===================== quaternion helpers ===================== */

static void q_normalize(float q[4])
{
    float n = sqrtf(q[0]*q[0] + q[1]*q[1] + q[2]*q[2] + q[3]*q[3]);
    if (n < 1e-9f) { q[0] = 1; q[1] = q[2] = q[3] = 0; return; }
    float inv = 1.0f / n;
    q[0]*=inv; q[1]*=inv; q[2]*=inv; q[3]*=inv;
}

/* out = a (x) b   (Hamilton product) */
static void q_mul(const float a[4], const float b[4], float out[4])
{
    out[0] = a[0]*b[0] - a[1]*b[1] - a[2]*b[2] - a[3]*b[3];
    out[1] = a[0]*b[1] + a[1]*b[0] + a[2]*b[3] - a[3]*b[2];
    out[2] = a[0]*b[2] - a[1]*b[3] + a[2]*b[0] + a[3]*b[1];
    out[3] = a[0]*b[3] + a[1]*b[2] - a[2]*b[1] + a[3]*b[0];
}

/* rotation matrix R (body -> world), row-major 3x3 */
static void q_to_R(const float q[4], float R[9])
{
    float w=q[0], x=q[1], y=q[2], z=q[3];
    R[0] = 1-2*(y*y+z*z); R[1] = 2*(x*y-w*z);   R[2] = 2*(x*z+w*y);
    R[3] = 2*(x*y+w*z);   R[4] = 1-2*(x*x+z*z); R[5] = 2*(y*z-w*x);
    R[6] = 2*(x*z-w*y);   R[7] = 2*(y*z+w*x);   R[8] = 1-2*(x*x+y*y);
}

/* v_world = R * v_body */
static void R_mul(const float R[9], const float v[3], float o[3])
{
    o[0] = R[0]*v[0]+R[1]*v[1]+R[2]*v[2];
    o[1] = R[3]*v[0]+R[4]*v[1]+R[5]*v[2];
    o[2] = R[6]*v[0]+R[7]*v[1]+R[8]*v[2];
}

/* v_body = R^T * v_world */
static void Rt_mul(const float R[9], const float v[3], float o[3])
{
    o[0] = R[0]*v[0]+R[3]*v[1]+R[6]*v[2];
    o[1] = R[1]*v[0]+R[4]*v[1]+R[7]*v[2];
    o[2] = R[2]*v[0]+R[5]*v[1]+R[8]*v[2];
}

static void euler_to_q(float roll, float pitch, float yaw, float q[4])
{
    float cr=cosf(roll*0.5f),  sr=sinf(roll*0.5f);
    float cp=cosf(pitch*0.5f), sp=sinf(pitch*0.5f);
    float cy=cosf(yaw*0.5f),   sy=sinf(yaw*0.5f);
    q[0] = cr*cp*cy + sr*sp*sy;
    q[1] = sr*cp*cy - cr*sp*sy;
    q[2] = cr*sp*cy + sr*cp*sy;
    q[3] = cr*cp*sy - sr*sp*cy;
    q_normalize(q);
}

static void vec_normalize(float v[3])
{
    float n = sqrtf(v[0]*v[0]+v[1]*v[1]+v[2]*v[2]);
    if (n < 1e-9f) return;
    float inv = 1.0f/n;
    v[0]*=inv; v[1]*=inv; v[2]*=inv;
}

/* ===================== init ===================== */

void ESKF_Init(ESKF_t *e, const float acc_g[3], const float mag_uT[3])
{
    memset(e, 0, sizeof(*e));

    /* default tuning (starting points - retune with real data) */
    e->q_gyro  = 1e-4f;
    e->q_bias  = 1e-7f;
    e->r_acc   = 5e-2f;
    e->r_mag   = 8e-2f;
    e->acc_gate = 0.15f;
    e->use_mag  = 1;
    e->g_ref[0] = 0; e->g_ref[1] = 0; e->g_ref[2] = 1;

    /* roll/pitch from gravity direction */
    float a[3] = { acc_g[0], acc_g[1], acc_g[2] };
    vec_normalize(a);
    float roll  = atan2f(a[1], a[2]);
    float pitch = atan2f(-a[0], sqrtf(a[1]*a[1] + a[2]*a[2]));

    /* tilt-only quaternion, rotate mag into world, derive heading */
    float q_tilt[4];
    euler_to_q(roll, pitch, 0.0f, q_tilt);
    float Rt[9]; q_to_R(q_tilt, Rt);
    float m[3] = { mag_uT[0], mag_uT[1], mag_uT[2] };
    vec_normalize(m);
    float m_w[3]; R_mul(Rt, m, m_w);

    float yaw = -atan2f(m_w[0], m_w[1]);     /* align horizontal mag to +Y (north) */
    euler_to_q(roll, pitch, yaw, e->q);

    /* fixed world magnetic reference (east component zeroed) */
    e->m_ref[0] = 0.0f;
    e->m_ref[1] = sqrtf(m_w[0]*m_w[0] + m_w[1]*m_w[1]);
    e->m_ref[2] = m_w[2];
    vec_normalize(e->m_ref);

    /* initial covariance: modest tilt/heading + bias uncertainty */
    for (int i = 0; i < 36; i++) e->P[i] = 0.0f;
    e->P[0]  = e->P[7]  = 0.02f;   /* dtheta x,y (rad^2) */
    e->P[14] = 0.2f;               /* dtheta z (yaw) less certain */
    e->P[21] = e->P[28] = e->P[35] = 1e-3f; /* bias */

    e->ready = 1;
}

/* ===================== predict ===================== */

void ESKF_Predict(ESKF_t *e, const float gyro_rads[3], float dt)
{
    if (!e->ready || dt <= 0.0f) return;

    /* bias-corrected rate */
    float w[3] = { gyro_rads[0]-e->bg[0], gyro_rads[1]-e->bg[1], gyro_rads[2]-e->bg[2] };

    /* ---- nominal: integrate quaternion ---- */
    float ang = sqrtf(w[0]*w[0]+w[1]*w[1]+w[2]*w[2]) * dt;
    float dq[4];
    if (ang > 1e-8f)
    {
        float s = sinf(ang*0.5f) / (ang/dt);   /* = sin(ang/2)/|w| */
        dq[0] = cosf(ang*0.5f);
        dq[1] = w[0]*s; dq[2] = w[1]*s; dq[3] = w[2]*s;
    }
    else { dq[0]=1; dq[1]=0.5f*w[0]*dt; dq[2]=0.5f*w[1]*dt; dq[3]=0.5f*w[2]*dt; }
    float qn[4]; q_mul(e->q, dq, qn);
    q_normalize(qn);
    memcpy(e->q, qn, sizeof(qn));

    /* ---- error covariance: P = Phi P Phi^T + Q ----
     * F = [ -[w]x   -I ;  0   0 ],   Phi = I + F*dt            */
    float Wx[9]; skew(w, Wx);
    float Phi[36];
    for (int i=0;i<36;i++) Phi[i]=0.0f;
    for (int i=0;i<6;i++) Phi[i*6+i]=1.0f;           /* I6 */
    for (int i=0;i<3;i++)                             /* top-left: -[w]x*dt */
        for (int j=0;j<3;j++)
            Phi[i*6+j] += -Wx[i*3+j]*dt;
    for (int i=0;i<3;i++) Phi[i*6+(3+i)] += -dt;      /* top-right: -I*dt */

    float Pt[36], tmp[36], PhiT[36];
    matmul(Phi, 6,6, e->P, 6, tmp);                  /* tmp = Phi*P */
    mattrans(Phi, 6,6, PhiT);
    matmul(tmp, 6,6, PhiT, 6, Pt);                   /* Pt = Phi*P*Phi^T */

    /* + Q (diagonal, scaled by dt) */
    Pt[0]  += e->q_gyro*dt; Pt[7]  += e->q_gyro*dt; Pt[14] += e->q_gyro*dt;
    Pt[21] += e->q_bias*dt; Pt[28] += e->q_bias*dt; Pt[35] += e->q_bias*dt;

    memcpy(e->P, Pt, sizeof(Pt));
}

/* ===================== generic 3-dim vector update ===================== */
/* H = [ skew(h) | 0_3x3 ] (3x6).  z, h are 3-vectors.  r = scalar meas var. */
static void update3(ESKF_t *e, const float h[3], const float z[3], float r)
{
    float H[18];                 /* 3x6 */
    float Hsk[9]; skew(h, Hsk);
    for (int i=0;i<3;i++)
        for (int j=0;j<6;j++)
            H[i*6+j] = (j<3) ? Hsk[i*3+j] : 0.0f;

    float HT[18];  mattrans(H, 3,6, HT);             /* 6x3 */
    float PHt[18]; matmul(e->P, 6,6, HT, 3, PHt);    /* 6x3 */
    float S[9];    matmul(H, 3,6, PHt, 3, S);        /* 3x3 */
    S[0]+=r; S[4]+=r; S[8]+=r;
    float Sinv[9];
    if (mat3_inv(S, Sinv)) return;                   /* skip if singular */

    float K[18];   matmul(PHt, 6,3, Sinv, 3, K);     /* 6x3 */

    /* innovation y = z - h */
    float y[3] = { z[0]-h[0], z[1]-h[1], z[2]-h[2] };

    /* dx = K*y  (6) */
    float dx[6];
    for (int i=0;i<6;i++)
        dx[i] = K[i*3+0]*y[0] + K[i*3+1]*y[1] + K[i*3+2]*y[2];

    /* ---- inject ---- */
    float dq[4] = { 1.0f, 0.5f*dx[0], 0.5f*dx[1], 0.5f*dx[2] };
    float qn[4]; q_mul(e->q, dq, qn); q_normalize(qn);
    memcpy(e->q, qn, sizeof(qn));
    e->bg[0]+=dx[3]; e->bg[1]+=dx[4]; e->bg[2]+=dx[5];

    /* ---- covariance (Joseph form): P = (I-KH)P(I-KH)^T + K R K^T ---- */
    float KH[36]; matmul(K, 6,3, H, 6, KH);          /* 6x6 */
    float IKH[36];
    for (int i=0;i<36;i++) IKH[i] = -KH[i];
    for (int i=0;i<6;i++) IKH[i*6+i] += 1.0f;        /* I - KH */

    float tmp[36], IKHt[36], P1[36];
    matmul(IKH, 6,6, e->P, 6, tmp);
    mattrans(IKH, 6,6, IKHt);
    matmul(tmp, 6,6, IKHt, 6, P1);                   /* (I-KH)P(I-KH)^T */

    /* + K R K^T  (R = r*I3  ->  r*K*K^T) */
    float KT[18]; mattrans(K, 6,3, KT);
    float KKt[36]; matmul(K, 6,3, KT, 6, KKt);
    for (int i=0;i<36;i++) P1[i] += r*KKt[i];

    /* symmetrize for numerical stability */
    for (int i=0;i<6;i++)
        for (int j=i;j<6;j++)
        {
            float s = 0.5f*(P1[i*6+j]+P1[j*6+i]);
            e->P[i*6+j] = s; e->P[j*6+i] = s;
        }
}

/* ===================== accel / mag updates ===================== */

void ESKF_UpdateAccel(ESKF_t *e, const float acc_g[3])
{
    if (!e->ready) return;
    float norm = sqrtf(acc_g[0]*acc_g[0]+acc_g[1]*acc_g[1]+acc_g[2]*acc_g[2]);
    if (norm < 1e-3f) return;
    if (fabsf(norm - 1.0f) > e->acc_gate) return;    /* reject during acceleration */

    float z[3] = { acc_g[0]/norm, acc_g[1]/norm, acc_g[2]/norm };
    float R[9]; q_to_R(e->q, R);
    float h[3]; Rt_mul(R, e->g_ref, h);              /* predicted up-dir in body */
    update3(e, h, z, e->r_acc);
}

void ESKF_UpdateMag(ESKF_t *e, const float mag_uT[3])
{
    if (!e->ready || !e->use_mag) return;
    float z[3] = { mag_uT[0], mag_uT[1], mag_uT[2] };
    float n = sqrtf(z[0]*z[0]+z[1]*z[1]+z[2]*z[2]);
    if (n < 1e-6f) return;
    z[0]/=n; z[1]/=n; z[2]/=n;

    float R[9]; q_to_R(e->q, R);
    float h[3]; Rt_mul(R, e->m_ref, h);              /* predicted mag in body */
    update3(e, h, z, e->r_mag);
}

/* ===================== convenience step ===================== */

void ESKF_Step(ESKF_t *e, const SensorData_t *s, float dt)
{
    float gyro[3] = { s->gyro_dps[0]*DEG2RAD,
                      s->gyro_dps[1]*DEG2RAD,
                      s->gyro_dps[2]*DEG2RAD };
    ESKF_Predict(e, gyro, dt);
    ESKF_UpdateAccel(e, s->accel_g);
    ESKF_UpdateMag(e, s->mag_uT);
}

/* ===================== output ===================== */

void ESKF_GetEuler(const ESKF_t *e, float *roll_deg, float *pitch_deg, float *yaw_deg)
{
    float w=e->q[0], x=e->q[1], y=e->q[2], z=e->q[3];
    float roll  = atan2f(2.0f*(w*x + y*z), 1.0f - 2.0f*(x*x + y*y));
    float sp = 2.0f*(w*y - z*x);
    if (sp >  1.0f) sp =  1.0f;
    if (sp < -1.0f) sp = -1.0f;
    float pitch = asinf(sp);
    float yaw   = atan2f(2.0f*(w*z + x*y), 1.0f - 2.0f*(y*y + z*z));
    if (roll_deg)  *roll_deg  = roll  * RAD2DEG;
    if (pitch_deg) *pitch_deg = pitch * RAD2DEG;
    if (yaw_deg)   *yaw_deg   = yaw   * RAD2DEG;
}
