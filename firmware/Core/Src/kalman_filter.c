/*
 * kalman_filter.c
 *
 *  Created on: 18 พ.ค. 2569
 *      Author: POND
 *
 * =======================================================================
 *  Full-order Discrete Kalman Filter -- 1-DOF Robot
 * =======================================================================
 *
 *  CONTINUOUS-TIME STATE SPACE
 *  ───────────────────────────
 *  x = [q, q_dot, i, tau_d]^T       u = V (motor voltage)
 *
 *  Ac = [ 0,       1,              0,      0  ]
 *       [ 0,    -b/J,   N*Kt*e/J, -1/J       ]
 *       [ 0,  -Ke*N/L,    -R/L,      0       ]
 *       [ 0,       0,          0,    0       ]
 *
 *  Bc = [0,  0,  1/L,  0]^T
 *  C  = [1,  0,    0,  0]      (measure position only)
 *
 *  DISCRETISATION (Euler forward, DT = 1 ms)
 *  ─────────────────────────────────────────
 *  Ad = I + Ac*DT
 *  Bd = Bc*DT
 *
 *  Ad (with N=2, numerical values):
 *    Row 0: [1,      0.001,         0,           0     ]
 *    Row 1: [0,  1-6.75e-3,   11.02e-3,   -277.8e-3   ]
 *    Row 2: [0,  -78.44e-3,   1-793.1e-3,      0      ]
 *    Row 3: [0,       0,           0,           1     ]
 *
 *  KF CYCLE (every 1 ms)
 *  ─────────────────────
 *  --- PREDICT ---
 *    x_p = Ad*x + Bd*u
 *    P_p = Ad*P*Ad^T + Q
 *
 *  --- UPDATE  (C=[1,0,0,0] simplifies everything to scalars) ---
 *    S   = P_p[0][0] + R_meas          (scalar)
 *    K   = P_p[:,0]  / S               (4x1 vector, = P_p*C^T/S)
 *    nu  = z - x_p[0]                  (innovation)
 *    x   = x_p + K*nu
 *    P   = (I - K*C) * P_p
 * =======================================================================
 */

#include "kalman_filter.h"
#include <string.h>
#include <math.h>

/* ================================================================== */
/*  Internal matrix helpers (4x4, optimised for no heap allocation)   */
/* ================================================================== */

/* out = A * B  (4x4) */
static void mat4_mul(float out[4][4],
                     const float A[4][4],
                     const float B[4][4])
{
    float tmp[4][4];
    memset(tmp, 0, sizeof(tmp));
    for (int i = 0; i < 4; i++)
        for (int k = 0; k < 4; k++) {
            if (A[i][k] == 0.0f) continue;
            for (int j = 0; j < 4; j++)
                tmp[i][j] += A[i][k] * B[k][j];
        }
    memcpy(out, tmp, sizeof(tmp));
}

/* out = A * B^T  (4x4) -- used for Ad*P*Ad^T in two steps */
static void mat4_mul_BT(float out[4][4],
                        const float A[4][4],
                        const float B[4][4])
{
    float tmp[4][4];
    memset(tmp, 0, sizeof(tmp));
    /* out[i][j] = sum_k A[i][k] * B[j][k]  (B transposed) */
    for (int i = 0; i < 4; i++)
        for (int j = 0; j < 4; j++)
            for (int k = 0; k < 4; k++)
                tmp[i][j] += A[i][k] * B[j][k];
    memcpy(out, tmp, sizeof(tmp));
}

/* ================================================================== */
/*  KF_Init                                                            */
/* ================================================================== */
void KF_Init(KalmanFilter_t *kf, float q0)
{
    memset(kf, 0, sizeof(KalmanFilter_t));

    /* ---------- Initial state ---------- */
    kf->x[0] = q0;    /* position       */
    kf->x[1] = 0.0f;  /* velocity       */
    kf->x[2] = 0.0f;  /* current        */
    kf->x[3] = 0.0f;  /* disturbance    */

    /* ---------- Derive continuous-time A entries ---------- */
    /*
     *  Motor 2 รอบ : Joint 1 รอบ  ->  N = 2
     *  All quantities in joint-side coordinates.
     */
    const float a22 = -KF_B  / KF_J;                        /* -6.750  */
    const float a23 =  KF_N  * KF_KT * KF_EEFF / KF_J;     /* +11.02  */
    const float a24 = -1.0f  / KF_J;                        /* -277.8  */
    const float a32 = -KF_KE * KF_N  / KF_L;               /* -78.44  */
    const float a33 = -KF_R  / KF_L;                        /* -793.1  */
    const float b3  =  1.0f  / KF_L;                        /* +375.1  */

    /* ---------- Build Ad = I + Ac*DT (Euler forward) ---------- */
    /* Row 0: d(q)/dt = q_dot */
    kf->Ad[0][0] = 1.0f;
    kf->Ad[0][1] = KF_DT;        /* 0.001  */
    kf->Ad[0][2] = 0.0f;
    kf->Ad[0][3] = 0.0f;

    /* Row 1: d(q_dot)/dt = a22*q_dot + a23*i + a24*tau_d */
    kf->Ad[1][0] = 0.0f;
    kf->Ad[1][1] = 1.0f + a22 * KF_DT;   /*  1 - 6.750e-3 =  0.99325 */
    kf->Ad[1][2] = a23  * KF_DT;          /* +11.02e-3      = +0.01102 */
    kf->Ad[1][3] = a24  * KF_DT;          /* -277.8e-3      = -0.2778  */

    /* Row 2: d(i)/dt = a32*q_dot + a33*i + b3*V */
    kf->Ad[2][0] = 0.0f;
    kf->Ad[2][1] = a32 * KF_DT;           /* -78.44e-3      = -0.07844 */
    kf->Ad[2][2] = 1.0f + a33 * KF_DT;   /*  1 - 793.1e-3  =  0.2069  */
    kf->Ad[2][3] = 0.0f;

    /* Row 3: d(tau_d)/dt = 0  (random walk) */
    kf->Ad[3][0] = 0.0f;
    kf->Ad[3][1] = 0.0f;
    kf->Ad[3][2] = 0.0f;
    kf->Ad[3][3] = 1.0f;

    /* ---------- Build Bd = Bc*DT ---------- */
    kf->Bd[0] = 0.0f;
    kf->Bd[1] = 0.0f;
    kf->Bd[2] = b3 * KF_DT;    /* 375.1e-3 = 0.3751 */
    kf->Bd[3] = 0.0f;

    /* ---------- Process noise Q (diagonal) ---------- */
    kf->Q[0][0] = KF_Q_POS;    /* 1e-8  */
    kf->Q[1][1] = KF_Q_VEL;    /* 1e-4  */
    kf->Q[2][2] = KF_Q_CUR;    /* 1e-2  */
    kf->Q[3][3] = KF_Q_TAUD;   /* 1e-5  */

    /* ---------- Measurement noise ---------- */
    kf->R_meas = KF_R_MEAS;    /* 1.471e-7 rad^2 */

    /* ---------- Initial covariance P0 = diag([1, 10, 1, 1]) ----------
     *  ตั้งสูงเผื่อความไม่แน่ใจเริ่มต้น โดยเฉพาะ velocity           */
    kf->P[0][0] = 1.0f;
    kf->P[1][1] = 10.0f;
    kf->P[2][2] = 1.0f;
    kf->P[3][3] = 1.0f;

    /* ---------- Output convenience fields ---------- */
    kf->est_position    = q0;
    kf->est_velocity    = 0.0f;
    kf->est_current     = 0.0f;
    kf->est_disturbance = 0.0f;
}

/* ================================================================== */
/*  KF_Reset                                                           */
/* ================================================================== */
void KF_Reset(KalmanFilter_t *kf, float q0)
{
    /* Keep Ad, Bd, Q, R_meas intact -- only reset state & covariance */
    kf->x[0] = q0;
    kf->x[1] = 0.0f;
    kf->x[2] = 0.0f;
    kf->x[3] = 0.0f;

    memset(kf->P, 0, sizeof(kf->P));
    kf->P[0][0] = 1.0f;
    kf->P[1][1] = 10.0f;
    kf->P[2][2] = 1.0f;
    kf->P[3][3] = 1.0f;

    kf->est_position    = q0;
    kf->est_velocity    = 0.0f;
    kf->est_current     = 0.0f;
    kf->est_disturbance = 0.0f;
}

/* ================================================================== */
/*  KF_Update  -- call every 1 ms                                      */
/* ================================================================== */
void KF_Update(KalmanFilter_t *kf, float V_in, float z_pos)
{
    int i, j;

    /* ==============================================================
     *  STEP 1: PREDICT STATE
     *  x_p = Ad * x  +  Bd * u
     * ============================================================== */
    float xp[4];
    for (i = 0; i < 4; i++) {
        xp[i] = kf->Bd[i] * V_in;
        for (j = 0; j < 4; j++)
            xp[i] += kf->Ad[i][j] * kf->x[j];
    }

    /* Clamp current estimate (physical sanity: |i| < V_max/R) */
    const float i_max = KF_VMAX / KF_R;   /* ~11.35 A */
    if      (xp[2] >  i_max) xp[2] =  i_max;
    else if (xp[2] < -i_max) xp[2] = -i_max;

    /* ==============================================================
     *  STEP 2: PREDICT COVARIANCE
     *  P_p = Ad * P * Ad^T  +  Q
     *
     *  Compute in two steps to avoid a 3rd temporary:
     *    tmp  = Ad * P
     *    P_p  = tmp * Ad^T   (= mat4_mul_BT(tmp, Ad))
     * ============================================================== */
    float tmp[4][4];
    float Pp[4][4];

    mat4_mul(tmp, kf->Ad, kf->P);    /* tmp = Ad * P    */
    mat4_mul_BT(Pp, tmp, kf->Ad);    /* Pp  = tmp * Ad^T */

    /* Add Q */
    for (i = 0; i < 4; i++)
        for (j = 0; j < 4; j++)
            Pp[i][j] += kf->Q[i][j];

    /* ==============================================================
     *  STEP 3: KALMAN GAIN
     *  C = [1, 0, 0, 0]
     *  S   = C * P_p * C^T + R  =  P_p[0][0] + R_meas
     *  K   = P_p * C^T / S      =  P_p[:, 0] / S
     * ============================================================== */
    float S    = Pp[0][0] + kf->R_meas;
    float Sinv = 1.0f / S;

    for (i = 0; i < 4; i++)
        kf->K[i] = Pp[i][0] * Sinv;

    /* ==============================================================
     *  STEP 4: UPDATE STATE
     *  innovation nu = z - C * x_p  =  z - x_p[0]
     *  x = x_p + K * nu
     * ============================================================== */
    float nu = z_pos - xp[0];

    for (i = 0; i < 4; i++)
        kf->x[i] = xp[i] + kf->K[i] * nu;

    /* ==============================================================
     *  STEP 5: UPDATE COVARIANCE  (Joseph form for numerical stability)
     *  P = (I - K*C) * P_p * (I - K*C)^T  +  K * R * K^T
     *
     *  For scalar R and C=[1,0,0,0]:
     *    IKC[i][j] = delta(i,j) - K[i] * C[j]
     *             = delta(i,j) - K[i] * (j==0)
     *
     *  Joseph form:
     *    P = IKC * Pp * IKC^T  +  R * K * K^T
     * ============================================================== */
    float IKC[4][4];
    for (i = 0; i < 4; i++)
        for (j = 0; j < 4; j++)
            IKC[i][j] = (float)(i == j) - kf->K[i] * (float)(j == 0);

    float IKC_Pp[4][4];
    mat4_mul(IKC_Pp, IKC, Pp);             /* IKC_Pp = IKC * Pp         */
    mat4_mul_BT(kf->P, IKC_Pp, IKC);      /* P = IKC_Pp * IKC^T        */

    /* Add  R * K * K^T  (rank-1 update) */
    for (i = 0; i < 4; i++)
        for (j = 0; j < 4; j++)
            kf->P[i][j] += kf->R_meas * kf->K[i] * kf->K[j];

    /* Force P symmetric (fix floating-point drift) */
    for (i = 0; i < 4; i++)
        for (j = i + 1; j < 4; j++) {
            float avg = 0.5f * (kf->P[i][j] + kf->P[j][i]);
            kf->P[i][j] = avg;
            kf->P[j][i] = avg;
        }

    /* ==============================================================
     *  Mirror to convenience output fields
     * ============================================================== */
    kf->est_position    = kf->x[0];
    kf->est_velocity    = kf->x[1];
    kf->est_current     = kf->x[2];
    kf->est_disturbance = kf->x[3];
}
