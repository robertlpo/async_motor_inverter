/*
 * peripherals.h
 *
 *  Created on: 15 maj 2015
 *      Author: Robert Łapucha
 *
 *  Microcontroller and board peripherals
 *
 */


#include <stdio.h>
#include <string.h>

#define ENC_FIRST_NONE						0			//encoder unchanged
#define ENC_FIRST_CH1						1			//forward
#define ENC_FIRST_CH2					   -1			//backward

typedef struct
{
	//public

	GPIO_TypeDef *Port;
	uint16_t Pin_Ch1;
	uint16_t Pin_Ch2;
	int Pulse_Width;

	//private

	int count_ch1;										//for measure pulse width on encoder ch1
	int count_ch2;										//for measure pulse width on encoder ch2
	int first_ch;										//which pulse activated first - ch1 (forward) or ch2 (backward)

} ENCODER_InitTypeDef;


void ENCODER_Init(ENCODER_InitTypeDef *enc)
{
	enc->count_ch1 = 0;
	enc->count_ch2 = 0;
	enc->first_ch = ENC_FIRST_NONE;
}


int ENCODER_GetStatus(ENCODER_InitTypeDef *enc)
{
	uint8_t ch1_state = HAL_GPIO_ReadPin(enc->Port, enc->Pin_Ch1);
	uint8_t ch2_state = HAL_GPIO_ReadPin(enc->Port, enc->Pin_Ch2);
	int first;


	//pulse on ch1


	if (ch1_state)
	{
		enc->count_ch1++;

		if (enc->first_ch == ENC_FIRST_NONE)
			enc->first_ch = ENC_FIRST_CH1;
	}


	//pulse on ch2


	if (ch2_state)
	{
		enc->count_ch2++;

		if (enc->first_ch == ENC_FIRST_NONE)
			enc->first_ch = ENC_FIRST_CH2;
	}


	//all channels not pulses now


	if (!ch1_state && !ch2_state)
	{
		if (enc->count_ch1 >= enc->Pulse_Width && enc->count_ch2 >= enc->Pulse_Width)
		{
			first = enc->first_ch;

			enc->count_ch1 = 0;
			enc->count_ch2 = 0;
			enc->first_ch = ENC_FIRST_NONE;

			return first;
		}
		else
		{
			enc->count_ch1 = 0;
			enc->count_ch2 = 0;
			enc->first_ch = ENC_FIRST_NONE;
			return 0;
		}
	}
	else
		return 0;
}


