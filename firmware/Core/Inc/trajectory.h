/*
 * trajectory.h
 *
 *  Created on: 29 เม.ย. 2569
 *      Author: POND
 */

#ifndef TRAJECTORY_H_
#define TRAJECTORY_H_

#include "main.h"
#include "encoder.h"
#include <stdint.h>

// ---------------- กำหนดค่าพารามิเตอร์ ----------------
#define TRAJ_V_MAX 3.8f    // ความเร็วสูงสุดเป้าหมาย (rad/s)
#define TRAJ_A_MAX 23.3f   // ความเร่งสูงสุด (Rad/s^2)
#define TRAJ_J_MAX 10.0f  // ความกระชากสูงสุด (Rad/s^3)

// กำหนด "ระยะเวลา" ที่ใช้เร่งและเบรก (วินาที)
#define TRAJ_ACCEL_TIME     0.613f    // เวลาที่ใช้เร่งจาก 0 ถึง V_MAX (เช่น 0.5 วินาที)

// ความสมูทของ S-Curve (วินาที) - ยิ่งมากยิ่งโค้งมน (แนะนำ 0.05 - 0.15)
#define TRAJ_S_SMOOTHNESS   10.1f


// =================================================================
// 1. Trapezoidal Profile
// =================================================================
typedef struct {
    float q_start;
    float q_end;
    float v_cruise;
    float a_calc;       // อัตราเร่งที่คำนวณจากเวลา
    float t_a;
    float t_c;
    float t_f;
    float sign;

    float current_t;
    uint8_t is_running;
} Trapz_Profile_t;

void Trapz_Init(Trapz_Profile_t *prof);
void Trapz_MoveTo(Trapz_Profile_t *prof, float start_pos, float target_pos);
void Trapz_Update(Trapz_Profile_t *prof, float *out_q, float *out_qd, float *out_qdd, float *out_j);


// =================================================================
// 2. S-Curve Profile
// =================================================================
typedef struct {
    float q_start, q_end, sign;
    float current_t;
    int is_running;

    // ช่วงเวลาทั้ง 7 (Time Segments)
    float t1, t2, t3, t4, t5, t6, t7;

    // ตัวแปรสะสมค่าแบบ Real-time
    float q_cmd, v_cmd, a_cmd;
    float prev_a_cmd; // ไว้ใช้หาค่า Jerk จากการ Diff
} SCurve_Profile_t;

void SCurve_Init(SCurve_Profile_t *prof);
void SCurve_MoveTo(SCurve_Profile_t *prof, float start_pos, float target_pos);
void SCurve_Update(SCurve_Profile_t *prof, float *out_q, float *out_qd, float *out_qdd, float *out_j);
#endif /* INC_TRAJECTORY_H_ */
