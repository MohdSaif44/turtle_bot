/*
 * PWMEncoder.c
 *
 *  Created on: May 9, 2024
 *      Author: mohds
 */

#include "PWMEncoder.h"

float PWMEncoder_Angle(TIM_HandleTypeDef* htimx, GPIO_TypeDef * GPIOx, uint16_t GPIO_Pin, PWMEnc_t *enc){

	enc->enc_GPIOX = GPIOx;

	enc->enc_PIN = GPIO_Pin;

	if(HAL_GPIO_ReadPin(GPIOx,GPIO_Pin)){

		enc->fullCount = htimx->Instance->CNT - enc->prevStart;

		enc->start = htimx->Instance->CNT;

	}


	if(!HAL_GPIO_ReadPin(GPIOx,GPIO_Pin)){

		enc->prevStart = enc->start;

		enc->end = htimx->Instance->CNT;

	}

	enc->diffrence = enc->end - enc->start;

	if(enc->diffrence < 0) {

		enc->diffrence = 65355 + enc->diffrence ;

	}

	if(enc->fullCount < 0) {

		enc->fullCount = 65355 + enc->fullCount ;

	}

	enc->diffrence = enc->end - enc->start;

	enc->dutyc = 100 * (float)enc->diffrence / 4140.0;

	if(enc->diffrence > 40.0 && enc->diffrence <= 4132.0){

		enc->Angle = 360 * (enc->dutyc - 0.3884438) / (99.805778 - 0.3884438);

	}

	else{

		enc->error = 1;

	}

		return enc->Angle;

}

