/*
 * encoder.c
 *
 *  Created on: 29 เม.ย. 2569
 *      Author: POND
 */


#include "encoder.h"

void Encoder_Init(Encoder_t *enc, TIM_HandleTypeDef *htim) {
    enc->htim = htim;
    enc->last_counter = 0;
    enc->position_rad = 0;
    enc ->position_raw = 0;
    HAL_TIM_Encoder_Start(enc->htim, TIM_CHANNEL_ALL);
}

void Encoder_Update(Encoder_t *enc) {

	uint32_t current_counter = __HAL_TIM_GET_COUNTER(enc->htim);

	int32_t diff = (int32_t)(current_counter - enc->last_counter);

	enc->position_raw += diff;
	enc->last_counter = current_counter;

	enc->position_rad = (float)enc->position_raw * (2.0f * PI / ENCODER_CPR);
	enc->position_deg = enc->position_rad * (180.0f / PI);
}

// ---- Getters ---
int32_t Encoder_GetPositionRaw(Encoder_t *enc) {
    return enc->position_raw;
}

float Encoder_GetPositionRad(Encoder_t *enc) {
    return enc->position_rad;
}

float Encoder_GetPositionDeg(Encoder_t *enc) {
    return enc->position_deg;
}
