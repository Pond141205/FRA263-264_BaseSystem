/*
 * trajectory.c
 *
 *  Created on: 29 เม.ย. 2569
 *      Author: POND
 */


#include "trajectory.h"
#include "cascade_control.h" // จำเป็นต้อง Include เพื่อดึงค่า PI และ q_out มาใช้งาน
#include <math.h>

// =================================================================
// 1. Trapezoidal Profile
// =================================================================

void Trapz_Init(Trapz_Profile_t *prof) {
    prof->is_running = 0;
}

void Trapz_MoveTo(Trapz_Profile_t *prof, float start_pos, float target_pos) {
    prof->q_start = start_pos;
    prof->q_end = target_pos;
    prof->current_t = 0.0f;

    float distance = fabs(target_pos - start_pos);
    prof->sign = (target_pos > start_pos) ? 1.0f : -1.0f;

    if (distance == 0.0f) {
        prof->is_running = 0;
        return;
    }

    // แปลง "เวลา (ACCEL_TIME)" เป็น "อัตราเร่ง"
    prof->a_calc = TRAJ_V_MAX / TRAJ_ACCEL_TIME;

    float a_max = prof->a_calc;
    float d_a = 0.5f * (TRAJ_V_MAX * TRAJ_V_MAX) / a_max;

    if (2.0f * d_a >= distance) { // Triangle (วิ่งไม่ถึง V_max)
        float calc_t_a = sqrtf(distance / a_max);
        prof->v_cruise = a_max * calc_t_a;
        prof->t_a = calc_t_a;
        prof->t_c = 0.0f;
    } else { // Trapezoid (วิ่งถึง V_max)
        prof->v_cruise = TRAJ_V_MAX;
        prof->t_a = TRAJ_ACCEL_TIME; // ใช้เวลาที่กำหนดได้เลย
        prof->t_c = (distance - (2.0f * d_a)) / TRAJ_V_MAX;
    }

    prof->t_f = (2.0f * prof->t_a) + prof->t_c;
    prof->is_running = 1;
}

void Trapz_Update(Trapz_Profile_t *prof, float *out_q, float *out_qd, float *out_qdd, float *out_j) {
    if (!prof->is_running) {
        *out_q = prof->q_end;
        *out_qd = 0.0f;
        *out_qdd = 0.0f;
        return;
    }

    prof->current_t += 0.001f;
    float t = prof->current_t;
    float a_max = prof->a_calc;

    if (t <= prof->t_a) {
        *out_q = prof->q_start + prof->sign * (0.5f * a_max * t * t);
        *out_qd = prof->sign * (a_max * t);
        *out_qdd = prof->sign * a_max;
    }
    else if (t <= (prof->t_a + prof->t_c)) {
        float t_cruise = t - prof->t_a;
        float d_a = 0.5f * prof->v_cruise * prof->t_a;
        *out_q = prof->q_start + prof->sign * (d_a + (prof->v_cruise * t_cruise));
        *out_qd = prof->sign * prof->v_cruise;
        *out_qdd = 0.0f;
    }
    else if (t < prof->t_f) {
        float t_dec = t - (prof->t_a + prof->t_c);
        float d_a = 0.5f * prof->v_cruise * prof->t_a;
        float d_c = prof->v_cruise * prof->t_c;
        *out_q = prof->q_start + prof->sign * (d_a + d_c + (prof->v_cruise * t_dec) - (0.5f * a_max * t_dec * t_dec));
        *out_qd = prof->sign * (prof->v_cruise - (a_max * t_dec));
        *out_qdd = prof->sign * -a_max;
    }
    else {
        *out_q = prof->q_end;
        *out_qd = 0.0f;
        *out_qdd = 0.0f;
        prof->is_running = 0;
    }
}


// =================================================================
// 2. S-Curve Profile
// =================================================================

void SCurve_Init(SCurve_Profile_t *prof) {
    prof->is_running = 0;
    prof->current_t = 0.0f;
    prof->q_start = 0.0f;
    prof->q_end = 0.0f;
    prof->q_cmd = 0.0f;
    prof->v_cmd = 0.0f;
    prof->a_cmd = 0.0f;
    prof->prev_a_cmd = 0.0f;
}

void SCurve_MoveTo(SCurve_Profile_t *prof, float start_pos, float target_pos) {
    prof->q_start = start_pos;
    prof->q_end = target_pos;
    prof->current_t = 0.0f;
    prof->is_running = 1;

    prof->q_cmd = start_pos;
    prof->v_cmd = 0.0f;
    prof->a_cmd = 0.0f;
    prof->prev_a_cmd = 0.0f;

    float distance = fabs(target_pos - start_pos);
    prof->sign = (target_pos >= start_pos) ? 1.0f : -1.0f;

    if (distance == 0.0f) {
        prof->is_running = 0;
        return;
    }

    // คำนวณเวลาแต่ละช่วง (สมมติฐานว่าระยะทางไกลพอที่จะทำความเร็วสูงสุดได้)
    float Tj = TRAJ_A_MAX / TRAJ_J_MAX;
	float Ta = (TRAJ_V_MAX / TRAJ_A_MAX) - Tj;
	float Tv = (distance / TRAJ_V_MAX) - (Ta + 2.0f * Tj);

	// --- ส่วนที่แก้บั๊กใหม่: ถ้าระยะทางสั้นเกินไปจนทำความเร็วสูงสุดไม่ได้ ---
	if (Tv < 0.0f) {
		Tv = 0.0f;

		// แก้สมการกำลังสองหา V_MAX ใหม่ที่วิ่งได้จริงในระยะทางแค่นี้
		float a_quad = 1.0f / TRAJ_A_MAX;
		float b_quad = Tj;
		float c_quad = -distance;
		float V_new = (-b_quad + sqrtf(b_quad*b_quad - 4.0f*a_quad*c_quad)) / (2.0f*a_quad);

		// คำนวณเวลา Ta ใหม่ตามความเร็วใหม่
		Ta = (V_new / TRAJ_A_MAX) - Tj;

		// ถ้าระยะทางสั้นมากๆ จนความเร่งยังไม่ทันถึง A_MAX ด้วยซ้ำ
		if (Ta < 0.0f) {
			Ta = 0.0f;
			Tj = cbrtf(distance / (2.0f * TRAJ_J_MAX));
		}
	}

    // กำหนดจุดเวลาเปลี่ยนผ่านทั้ง 7 จุด
    prof->t1 = Tj;
    prof->t2 = Tj + Ta;
    prof->t3 = Tj + Ta + Tj;
    prof->t4 = prof->t3 + Tv;
    prof->t5 = prof->t4 + Tj;
    prof->t6 = prof->t5 + Ta;
    prof->t7 = prof->t6 + Tj;
}

void SCurve_Update(SCurve_Profile_t *prof, float *out_q, float *out_qd, float *out_qdd, float *out_j) {
    // 1. ถ้าระบบไม่ได้รัน ให้ล็อกค่าเป้าหมายสุดท้ายไว้นิ่งๆ (ป้องกัน Stick-slip)
	if (!prof->is_running) {
	        // ให้พิกัดเป้าหมายค่อยๆ ไหลเข้าหาจุดจบแบบล่องหน (ป้องกันการวาร์ปข้ามทศนิยม)
	        float q_err = prof->q_end - prof->q_cmd;
	        if (fabs(q_err) > 0.0001f) {
	            prof->q_cmd += 0.01f * q_err; // ค่อยๆ ซึมเข้าหาเป้า
	        } else {
	            prof->q_cmd = prof->q_end; // ล็อกเมื่อสนิทแล้ว
	        }

	        *out_q = prof->q_cmd;
	        *out_qd = 0.0f;
	        *out_qdd = 0.0f;
	        *out_j = 0.0f;
	        return;
	    }

    prof->current_t += 0.001f; // DT = 1ms
    float t = prof->current_t;
    float J = 0.0f;

    // 2. กำหนดค่า Jerk ตามช่วงเวลา
    if      (t <= prof->t1) J =  TRAJ_J_MAX;
    else if (t <= prof->t2) J =  0.0f;
    else if (t <= prof->t3) J = -TRAJ_J_MAX;
    else if (t <= prof->t4) J =  0.0f;
    else if (t <= prof->t5) J = -TRAJ_J_MAX;
    else if (t <= prof->t6) J =  0.0f;
    else if (t <= prof->t7) J =  TRAJ_J_MAX;

    // 3. กระบวนการ Integrate และระบบ ป้องกันอาการกระตุกตอนจบ (Anti-Snap)
    if (t <= prof->t7) {
        float signed_J = J * prof->sign;
        prof->a_cmd += signed_J * 0.001f;
        prof->v_cmd += prof->a_cmd * 0.001f;

        // ✨ THE MAGIC FIX: Zero-Crossing Detection ✨
        // ในช่วงเบรกสุดท้าย (t > t6) ความเร็วจะวิ่งลงมาหา 0
        // ถ้าทศนิยมพาความเร็วทะลุ 0 ไปอีกฝั่ง ให้ตัดจบโปรไฟล์อย่างนิ่มนวลทันที!
        if (t > prof->t6) {
            if ((prof->sign > 0.0f && prof->v_cmd <= 0.0f) ||
                (prof->sign < 0.0f && prof->v_cmd >= 0.0f)) {

                prof->is_running = 0;
                prof->q_cmd = prof->q_end; // วางเป้าลงจุดจบเป๊ะๆ
                prof->v_cmd = 0.0f;        // บังคับความเร็วเป็น 0
                prof->a_cmd = 0.0f;
            }
        }

        // ถ้ายังไม่โดนตัดจบ ก็สะสมตำแหน่งต่อไป
        if (prof->is_running) {
            prof->q_cmd += prof->v_cmd * 0.001f;
        }

    } else {
        // กรณีเผื่อเหลือเผื่อขาด ถ้าเวลาเกิน t7 ไปแล้ว
        prof->is_running = 0;
        prof->q_cmd = prof->q_end;
        prof->v_cmd = 0.0f;
        prof->a_cmd = 0.0f;
    }

    // 4. บันทึกค่า a เพื่อคำนวณ Jerk ออกไปพล็อตกราฟ
    float derived_jerk = (prof->a_cmd - prof->prev_a_cmd) / 0.001f;
    prof->prev_a_cmd = prof->a_cmd;

    // 5. ส่งค่าออก
    *out_q = prof->q_cmd;
    *out_qd = prof->v_cmd;
    *out_qdd = prof->a_cmd;
    *out_j = derived_jerk;
}
