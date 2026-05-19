/*
 * cascade_control.h
 *
 *  Created on: 29 เม.ย. 2569
 *      Author: POND
 */

#ifndef CASCADE_CONTROL_H_
#define CASCADE_CONTROL_H_

#include "main.h"

// ---------------- Hardware Parameters ----------------
#define DT_VEL 0.001f  // 1 ms สำหรับลูปความเร็ว
#define DT_POS 0.010f  // 10 ms สำหรับลูปตำแหน่ง
#define MAX_VOLTAGE 24.0f   // Motor Maximum Voltage
#define PWM_ARR_MAX 9999.0f // TIM1 ARR value for 17 kHz PWM
#define GEAR_RATIO 2.0f

typedef struct {
    float Kp;
    float Ki;
    float Kd;
    float integral;
    float prev_error;
    float integral_limit;
} PID_Controller;

extern float          target_q;       // ตำแหน่งเป้าหมาย (Radian)
extern float          target_qd;      // ความเร็วเป้าหมาย (Radian/s) สำหรับ Trajectory
extern volatile float q_out;          // ตำแหน่งจริง (volatile สำหรับ CubeMonitor)
extern volatile float qd_out;         // ความเร็วจริง จาก KF (volatile สำหรับ CubeMonitor)
extern volatile float monitor_V_in;   // แรงดันที่สั่ง motor (volatile สำหรับ CubeMonitor)

// ---------------- Public Function Prototypes ----------------
void Cascade_Control_Init(void);
void Cascade_Control_Update(float ref_q, float ref_qd);


#endif /* CASCADE_CONTROL_H_ */
