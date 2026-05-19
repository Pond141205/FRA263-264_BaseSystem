/*
 * encoder.h
 *
 *  Created on: 29 เม.ย. 2569
 *      Author: POND
 */

#ifndef ENCODER_H_
#define ENCODER_H_

#include "main.h"

#define ENCODER_CPR 8192.0f
#define PI 3.1415926535f

typedef struct {
    TIM_HandleTypeDef *htim;
    uint32_t last_counter;
    int32_t position_raw;
    float position_rad;
    float position_deg;
} Encoder_t;

// Function Prototypes
void Encoder_Init(Encoder_t *enc, TIM_HandleTypeDef *htim);
void Encoder_Update(Encoder_t *enc);
int32_t Encoder_GetPositionRaw(Encoder_t *enc);
float Encoder_GetPositionRad(Encoder_t *enc);
float Encoder_GetPositionDeg(Encoder_t *enc);

#endif /* ENCODER_H_ */
