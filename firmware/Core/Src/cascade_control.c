/*
 * cascade_control.c
 *
 *  Created on: 29 เม.ย. 2569
 *      Author: POND
 */


#include "cascade_control.h"
#include "encoder.h"
#include "trajectory.h"
#include "kalman_filter.h"
#include "base_system.h"   /* modbus_registers[] */
#include <math.h>

// ---------------- ดึงค่าฮาร์ดแวร์จาก main.c ----------------
extern TIM_HandleTypeDef htim1; // PWM PA8
extern TIM_HandleTypeDef htim2; // QEI
extern Encoder_t henc2;

// ลบคำว่า extern ออก แล้วกำหนดค่าเริ่มต้นได้ตามปกติเลยครับ
volatile float monitor_V_in = 0.0f;

// ---------------- Private Variables (ตัวแปรภายใน) ----------------
volatile float q_out = 0.0f;       // ตำแหน่งจริง
static float prev_q_out = 0.0f;
volatile float qd_out = 0.0f;      // ความเร็วจริง


// ---------------- Public Variables (กำหนดค่าเริ่มต้น) ----------------
float target_q = 0.0f;
float target_qd = 0.0f;
float qdd_out = 0.0f;
float j_out = 0.0f;

KalmanFilter_t hkf;

// ---------------- Setup PID Controllers ----------------
PID_Controller pos_ctrl = { .Kp = 20.0f, .Ki = 0.0f, .Kd = 0.0f, .integral = 0.0f, .prev_error = 0.0f, .integral_limit = 50.0f };
PID_Controller vel_ctrl = { .Kp = 7.0f,  .Ki = 1.0f, .Kd = 0.0f, .integral = 0.0f, .prev_error = 0.0f, .integral_limit = MAX_VOLTAGE };
static float K_ff = 0.1045f; //ke

// ---------------- Private Functions (ฟังก์ชันซ่อนภายใน) ----------------
// ใส่ static เพื่อไม่ให้ไฟล์อื่นเรียกใช้ได้โดยตรง
static float calculate_pid(PID_Controller *pid, float error, float dt) {
    pid->integral += error * dt;
    if (pid->integral > pid->integral_limit) pid->integral = pid->integral_limit;
    else if (pid->integral < -pid->integral_limit) pid->integral = -pid->integral_limit;

    float derivative = (error - pid->prev_error) / dt;
    pid->prev_error = error;
    return (pid->Kp * error) + (pid->Ki * pid->integral) + (pid->Kd * derivative);
}

static void Motor_Drive(float V_in) {



    if (V_in >= 0.0f) {
        HAL_GPIO_WritePin(Motor_Dir_GPIO_Port, Motor_Dir_Pin, GPIO_PIN_RESET);
    } else {
        HAL_GPIO_WritePin(Motor_Dir_GPIO_Port, Motor_Dir_Pin, GPIO_PIN_SET);
        V_in = -V_in;
    }

    if (V_in > MAX_VOLTAGE) V_in = MAX_VOLTAGE;
    monitor_V_in = V_in;
    uint32_t duty = (uint32_t)((V_in / MAX_VOLTAGE) * PWM_ARR_MAX);
    __HAL_TIM_SET_COMPARE(&htim1, TIM_CHANNEL_1, duty);
}

// ---------------- Public Functions ----------------
// ฟังก์ชันล้างค่า เผื่อใช้สำหรับรีเซ็ตระบบ
void Cascade_Control_Init(void) {
    q_out = 0.0f;
    prev_q_out = 0.0f;
    qd_out = 0.0f;
    target_q = 0.0f;
    target_qd = 0.0f;

    KF_Init(&hkf, 0.0f);
}

// ลูปคำนวณหลัก ที่จะถูกเรียกใน Timer Interrupt
void Cascade_Control_Update(float ref_q, float ref_qd)
{
    static uint8_t pos_loop_counter    = 0;
     float   current_q_dot_ref   = 0.0f;
    static float   prev_qd_kf          = 0.0f;
    static float   prev_qdd_out        = 0.0f;

    /* ── [เดิม] อ่าน encoder ── */
    Encoder_Update(&henc2);
    float z_enc = Encoder_GetPositionRad(&henc2);

    /* ── [ใหม่] รัน Kalman Filter ──
     *   monitor_V_in ถูก set ใน Motor_Drive() รอบก่อนหน้า (1-step lag ยอมรับได้)
     */
    KF_Update(&hkf, monitor_V_in, z_enc);

    /* ── รับค่า state จาก KF แทนการคำนวณแบบเดิม ── */
    q_out  = hkf.est_position;   /* แทน Encoder_GetPositionRad()  */
    qd_out = hkf.est_velocity;   /* แทน numerical diff + LPF เดิม */

    /* ── qdd, jerk: คำนวณจาก qd_out ที่ smooth แล้วของ KF ── */
    float raw_qdd = (qd_out - prev_qd_kf) / KF_DT;
    float alpha_a = 0.02f;
    qdd_out = (alpha_a * raw_qdd) + ((1.0f - alpha_a) * qdd_out);
    prev_qd_kf = qd_out;

    float raw_j   = (qdd_out - prev_qdd_out) / KF_DT;
    float alpha_j = 0.005f;
    j_out = (alpha_j * raw_j) + ((1.0f - alpha_j) * j_out);
    prev_qdd_out = qdd_out;

    /* ════════════════════════════════════════════════════════════
     *  OUTER LOOP: Position (ทุก 10 ms)
     * ════════════════════════════════════════════════════════════ */
    pos_loop_counter++;
    if (pos_loop_counter >= 10) {
      //  float q_error = ref_q - q_out;
      //  current_q_dot_ref = calculate_pid(&pos_ctrl, q_error, DT_POS) + ref_qd;
        pos_loop_counter = 0;
    }

    current_q_dot_ref = ref_qd; // clear

    /* ════════════════════════════════════════════════════════════
     *  [TUNE] Live-update Kp/Ki/Kd จาก Modbus (reg 0x10-0x12)
     *  PC เขียนค่า Kp*100, Ki*100, Kd*100 ผ่าน FC06
     *  ถ้า reg == 0 แสดงว่ายังไม่ได้เซ็ต ให้คงค่าเดิมไว้
     * ════════════════════════════════════════════════════════════ */
    if (modbus_registers[0x10] != 0)
        vel_ctrl.Kp = (float)(int16_t)modbus_registers[0x10] / 100.0f;
    if (modbus_registers[0x11] != 0)
        vel_ctrl.Ki = (float)(int16_t)modbus_registers[0x11] / 100.0f;
    if (modbus_registers[0x12] != 0)
        vel_ctrl.Kd = (float)(int16_t)modbus_registers[0x12] / 100.0f;

    /* ════════════════════════════════════════════════════════════
     *  INNER LOOP: Velocity (ทุก 1 ms)
     * ════════════════════════════════════════════════════════════ */
    float qd_error = current_q_dot_ref - qd_out;
    float V_VEL    = calculate_pid(&vel_ctrl, qd_error, DT_VEL);

    /* Feed Forward (velocity) */
    float V_FF = K_ff * (ref_qd * GEAR_RATIO);

    /*
     *  (ตัวเลือก) Disturbance Feed-Forward จาก KF
     *  ────────────────────────────────────────────
     *  tau_d ถูก estimate โดย KF -> แปลงกลับเป็น Voltage ชดเชยได้เลย
     *
     *    tau_d [N.m joint] -> tau_motor = tau_d / (N * e_eff)
     *    V_dist = tau_motor * R / Kt  (static inversion)
     *           = tau_d * R / (N * e_eff * Kt)
     *
     *  Uncomment เพื่อเปิดใช้:
     */
    /* float V_dist = hkf.est_disturbance * KF_R
                      / (KF_N * KF_EEFF * KF_KT);
       Motor_Drive(V_VEL + V_FF + V_dist);          */

    Motor_Drive(V_VEL + V_FF);
}



/* ── 5. E-STOP / RE-HOMING ───────────────────────────────────────────── */
/*
 *  เมื่อ homing เสร็จหรือ e-stop clear -> reset KF ด้วย
 *
 *  ตัวอย่างการเรียกใน main.c หรือ interrupt handler:
 *
 *    extern KalmanFilter_t hkf;
 *    extern Encoder_t      henc2;
 *    Encoder_Update(&henc2);
 *    KF_Reset(&hkf, Encoder_GetPositionRad(&henc2));
 */


/* ── 6. MODBUS MONITOR (ตัวเลือก) ───────────────────────────────────── */
/*
 *  Register map (ตัวอย่าง) -- encode float เป็น int16 (*100 หรือ *1000)
 *
 *  #define REG_KF_POS    10    //  q       *100  [0.01 rad/bit]
 *  #define REG_KF_VEL    11    //  q_dot   *100  [0.01 rad/s per bit]
 *  #define REG_KF_CUR    12    //  i       *1000 [0.001 A per bit]
 *  #define REG_KF_TAUD   13    //  tau_d   *1000 [0.001 N.m per bit]
 *
 *  // เรียกใน while(1) หลัง Heartbeat_Update():
 *  modbus_registers[REG_KF_POS]  = (int16_t)(hkf.est_position    * 100.0f);
 *  modbus_registers[REG_KF_VEL]  = (int16_t)(hkf.est_velocity    * 100.0f);
 *  modbus_registers[REG_KF_CUR]  = (int16_t)(hkf.est_current     * 1000.0f);
 *  modbus_registers[REG_KF_TAUD] = (int16_t)(hkf.est_disturbance * 1000.0f);
 */
