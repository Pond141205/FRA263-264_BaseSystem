/*
 * kalman_filter.h
 *
 *  Created on: 18 พ.ค. 2569
 *      Author: POND
 *
 * =======================================================================
 *  Full-order Kalman Filter -- 1-DOF Robot (DC Motor + Pulley)
 *  State vector: x = [q, q_dot, i, tau_d]^T
 *
 *  JOINT-SIDE EQUATIONS OF MOTION
 *  ───────────────────────────────
 *    J * q_ddot = N * Kt * e_eff * i  -  b * q_dot  -  tau_d
 *    L * i_dot  = V  -  R * i  -  Ke * N * q_dot
 *    tau_d_dot  = 0     (slow random-walk disturbance)
 *
 *  GEAR (Pulley)
 *  ─────────────
 *    Motor 2 รอบ : Joint 1 รอบ
 *      => N = omega_motor / omega_joint = 2.0
 *      => omega_motor = N * q_dot_joint
 *      => tau_joint   = tau_motor * N * e_eff   (torque ขยาย 2x)
 *    Ke, Kt อ้างอิงที่ motor shaft:
 *      back-EMF = Ke * omega_motor = Ke * N * q_dot_joint
 *
 *  ENCODER (on JOINT shaft)
 *  ────────────────────────
 *    CPR        = 8192 counts/rev
 *    resolution = 2*PI / 8192 = 7.669e-4 rad/count
 *    sigma      = 0.5 count  => 3.835e-4 rad
 *    R_meas     = sigma^2    = 1.47e-7 rad^2
 *
 *  OPERATING LIMITS
 *  ────────────────
 *    V_max = 24 V,  DT = 1 ms
 *
 *  MEASUREMENT OUTPUT
 *  ──────────────────
 *    y = q   (position only, C = [1, 0, 0, 0])
 * =======================================================================
 */

#ifndef KALMAN_FILTER_H_
#define KALMAN_FILTER_H_

#include <stdint.h>

/* ------------------------------------------------------------------ */
/*  Motor / Plant Parameters                                           */
/* ------------------------------------------------------------------ */
#define KF_R        2.1142f       /* Winding resistance              [Ohm]        */
#define KF_L        0.0026657f    /* Winding inductance              [H]          */
#define KF_KE       0.1045f       /* Back-EMF constant (motor shaft) [V.s/rad_m]  */
#define KF_KT       0.1045f       /* Torque constant   (motor shaft) [N.m/A]      */
#define KF_J        0.0036f       /* Reflected inertia (joint side)  [kg.m^2]     */
#define KF_B        0.0243f       /* Viscous damping   (joint side)  [N.m.s/rad]  */
#define KF_EEFF     0.1897f       /* Drivetrain efficiency           [-]           */
#define KF_N        2.0f          /* Gear ratio: omega_motor/omega_joint           */
#define KF_VMAX     24.0f         /* Max motor voltage               [V]           */
#define KF_DT       0.001f        /* Sample period                   [s]           */

/*
 *  Derived A-matrix entries (computed in KF_Init for clarity):
 *
 *    a22 = -KF_B  / KF_J                     = -0.0243/0.0036    = -6.750
 *    a23 =  KF_N * KF_KT * KF_EEFF / KF_J   =  2*0.1045*0.1897/0.0036 = +11.02
 *    a24 = -1.0   / KF_J                     = -1/0.0036          = -277.8
 *    a32 = -KF_KE * KF_N / KF_L             = -0.1045*2/0.002666 = -78.44
 *    a33 = -KF_R  / KF_L                    = -2.1142/0.002666   = -793.1
 *    b3  =  1.0   / KF_L                    =  1/0.002666         = +375.1
 */

/* ------------------------------------------------------------------ */
/*  Encoder                                                            */
/* ------------------------------------------------------------------ */
#define KF_ENC_CPR      8192
/* resolution = 2*PI / CPR = 7.669e-4 rad,  sigma = 0.5 count = 3.835e-4 rad */

/* ------------------------------------------------------------------ */
/*  Noise Tuning                                                       */
/*                                                                     */
/*  Q[i][i]  = process noise  (ยิ่งมาก = เชื่อ model น้อยลง)         */
/*  R_meas   = measurement noise variance (จาก encoder resolution)   */
/*                                                                     */
/*  แนวทางปรับ Q:                                                      */
/*   - velocity ยัง oscillate มาก  -> เพิ่ม KF_Q_VEL                 */
/*   - filter ตาม encoder เร็วเกิน -> ลด KF_Q_VEL หรือเพิ่ม R_MEAS  */
/*   - disturbance ตอบสนองช้า      -> เพิ่ม KF_Q_TAUD                */
/* ------------------------------------------------------------------ */
#define KF_Q_POS    1e-8f    /* q     : position process noise         */
#define KF_Q_VEL    1e-4f    /* q_dot : velocity process noise         */
#define KF_Q_CUR    1e-2f    /* i     : current process noise          */
#define KF_Q_TAUD   1e-5f    /* tau_d : disturbance process noise      */

/* R_meas = (0.5 * 2*PI/8192)^2 = (3.835e-4)^2 = 1.471e-7 rad^2     */
#define KF_R_MEAS   1.471e-7f

/* ------------------------------------------------------------------ */
/*  Dimensions                                                         */
/* ------------------------------------------------------------------ */
#define KF_N_STATES  4       /* [q, q_dot, i, tau_d]  */
#define KF_N_OBS     1       /* [q]                   */

/* ------------------------------------------------------------------ */
/*  Kalman Filter Structure                                            */
/* ------------------------------------------------------------------ */
typedef struct {

    /* State estimate: x = [q, q_dot, i, tau_d] */
    float x[KF_N_STATES];

    /* Error covariance matrix 4x4 (row-major) */
    float P[KF_N_STATES][KF_N_STATES];

    /* Discrete-time system matrices (built once in KF_Init) */
    float Ad[KF_N_STATES][KF_N_STATES]; /* Ad = I + Ac*dt  (Euler forward) */
    float Bd[KF_N_STATES];              /* Bd = Bc*dt                       */

    /* Noise covariance */
    float Q[KF_N_STATES][KF_N_STATES];  /* Process noise (diagonal)         */
    float R_meas;                        /* Measurement noise variance       */

    /* Kalman gain vector (4x1, exploits C = [1,0,0,0]) */
    float K[KF_N_STATES];

    /* ---- Convenience output fields (always mirror x[]) ---- */
    float est_position;     /* q       [rad]   */
    float est_velocity;     /* q_dot   [rad/s] */
    float est_current;      /* i       [A]     */
    float est_disturbance;  /* tau_d   [N.m]   */

} KalmanFilter_t;

/* ------------------------------------------------------------------ */
/*  Public API                                                         */
/* ------------------------------------------------------------------ */

/**
 * @brief  Initialise: build Ad/Bd from motor params, set Q, R, P0.
 * @param  kf   Pointer to filter instance.
 * @param  q0   Initial joint position [rad]  (read from encoder after homing).
 */
void KF_Init(KalmanFilter_t *kf, float q0);

/**
 * @brief  One filter cycle (predict + update). Call every KF_DT = 1 ms.
 *
 *         Recommended call order inside TIM ISR:
 *           1. Encoder_Update(&henc2);
 *           2. z = Encoder_GetPositionRad(&henc2);
 *           3. KF_Update(&hkf, monitor_V_in, z);      // <-- here
 *           4. Cascade_Control_Update(ref_q, ref_qd);
 *
 * @param  kf     Filter instance.
 * @param  V_in   Voltage commanded in the PREVIOUS cycle [V].
 *                Use monitor_V_in (set by Motor_Drive in last cycle).
 *                One-step lag is negligible at 1 ms.
 * @param  z_pos  Encoder measurement this cycle [rad].
 */
void KF_Update(KalmanFilter_t *kf, float V_in, float z_pos);

/**
 * @brief  Reset state and covariance (call after e-stop or re-homing).
 * @param  kf   Filter instance.
 * @param  q0   Current joint position [rad].
 */
void KF_Reset(KalmanFilter_t *kf, float q0);

#endif /* KALMAN_FILTER_H_ */
