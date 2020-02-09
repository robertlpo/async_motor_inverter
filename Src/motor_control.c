/*
 * motor_control.c
 *
 *  Created on: 16 gru 2016
 *      Author: Robert Łapucha
 */


#include <stdlib.h>
#include "motor_control.h"
#include "params.h"
#include "peripherals.h"
#include "encoder.h"
#include "calc.h"


/* Timer2 configuration: APB1 = 72MHz / (Prescaler 7200) = 10kHz / (Period 10000) = 1Hz (1s) */
/* Timer2 configuration: APB1 = 72MHz / (Prescaler 72) = 1MHz / (Period 10) = 100KHz (10us) */

MovAvrg_TypeDef encoder_avrg;
MovAvrg_TypeDef throttle_avrg;
MovAvrgFloat_TypeDef motor_delay_avrg;
Throttle_TypeDef throttle;
Throttle_TypeDef reduced;
Throttle_TypeDef throttle_min = { .freq = 10, .period = 0.1, .freq_mech = 10 / PAIR_OF_POLES };
Throttle_TypeDef throttle_reduced;
Encoder_TypeDef encoder;
Motor_TypeDef motor;

volatile int count10us = 0;
volatile int count1ms = 0;
volatile int count10ms = 0;
volatile int count10us_per_idx = 0;


/* sin pattern pwm 100% */

//int sin_pattern_current[SIN_PATTERN_SIZE] = {9, 26, 42, 57, 71, 82, 91, 97, 99, 99, 97, 91, 82, 71, 57, 42, 26, 9,
//											-9, -26, -42, -57, -71, -82, -91, -97, -99, -99, -97, -91, -82, -71, -57, -42, -26, -9};

int sin_pattern_current[SIN_PATTERN_SIZE] = {0, 4, 9, 13, 17, 22, 26, 30, 34, 38, 42, 46, 50, 54, 57, 61, 64, 68, 71, 74, 77, 79, 82, 84, 87, 89, 91, 92, 94, 95, 97, 98, 98, 99, 100, 100, 100, 100, 100, 99, 98, 98, 97, 95, 94, 92, 91, 89, 87, 84, 82, 79, 77, 74, 71, 68, 64, 61, 57, 54, 50, 46, 42, 38, 34, 30, 26, 22, 17, 13, 9, 4,
										    -0, -4, -9, -13, -17, -22, -26, -30, -34, -38, -42, -46, -50, -54, -57, -61, -64, -68, -71, -74, -77, -79, -82, -84, -87, -89, -91, -92, -94, -95, -97, -98, -98, -99, -100, -100, -100, -100, -100, -99, -98, -98, -97, -95, -94, -92, -91, -89, -87, -84, -82, -79, -77, -74, -71, -68, -64, -61, -57, -54, -50, -46, -42, -38, -34, -30, -26, -22, -17, -13, -9, -4};

/* Attention! 0 first means no PWM - delay between transistor switching */

//int sin_pattern[SIN_PATTERN_SIZE] = {0, 32, 45, 57, 71, 82, 91, 97, 99, 99, 97, 91, 82, 71, 57, 45, 32, 0,
//		                             0, 32, 45, 57, 71, 82, 91, 97, 99, 99, 97, 91, 82, 71, 57, 45, 32, 0};

int sin_pattern[SIN_PATTERN_SIZE] = {0, 4, 9, 13, 17, 22, 26, 30, 34, 38, 42, 46, 50, 54, 57, 61, 64, 68, 71, 74, 77, 79, 82, 84, 87, 89, 91, 92, 94, 95, 97, 98, 98, 99, 100, 100, 100, 100, 100, 99, 98, 98, 97, 95, 94, 92, 91, 89, 87, 84, 82, 79, 77, 74, 71, 68, 64, 61, 57, 54, 50, 46, 42, 38, 34, 30, 26, 22, 17, 13, 9, 4,
									 0, 4, 9, 13, 17, 22, 26, 30, 34, 38, 42, 46, 50, 54, 57, 61, 64, 68, 71, 74, 77, 79, 82, 84, 87, 89, 91, 92, 94, 95, 97, 98, 98, 99, 100, 100, 100, 100, 100, 99, 98, 98, 97, 95, 94, 92, 91, 89, 87, 84, 82, 79, 77, 74, 71, 68, 64, 61, 57, 54, 50, 46, 42, 38, 34, 30, 26, 22, 17, 13, 9, 4};


/* correction % for sin pattern */

//int sin_correct[SIN_PATTERN_SIZE] = {-30, -30, -30, -30, -30, -30, -30, -30, -30, -30, -30, -30, -30, -30, -30, -30, -30, -30,
//									 -30, -30, -30, -30, -30, -30, -30, -30, -30, -30, -30, -30, -30, -30, -30, -30, -30, -30};

int sin_correct;												// % of correction pwm, for example -30%, +50%, ...

//int sin_simul_curr[SIN_PATTERN_SIZE] = {9, 26, 42, 57, 71, 82, 91, 97, 99, 99, 97, 91, 82, 71, 57, 42, 26, 9,
//										-9, -26, -42, -57, -71, -82, -91, -97, -99, -99, -97, -91, -82, -71, -57, -42, -26, -9};
//int sin_simul_curr[SIN_PATTERN_SIZE] = {1, 5, 11, 13, 80, 82, 91, 97, 99, 99, 97, 91, 82, 80, 13, 11, 5, 1
//									   -1, -5, -11, -13, -80, -82, -91, -97, -99, -99, -97, -91, -82, -80, -13, -11, -5, -1,};
//int sin_simul_curr[SIN_PATTERN_SIZE] = {0, 4, 9, 13, 17, 22, 26, 30, 34, 38, 42, 46, 50, 54, 57, 61, 64, 68, 71, 74, 77, 79, 82, 84, 87, 89, 91, 92, 94, 95, 97, 98, 98, 99, 100, 100, 100, 100, 100, 99, 98, 98, 97, 95, 94, 92, 91, 89, 87, 84, 82, 79, 77, 74, 71, 68, 64, 61, 57, 54, 50, 46, 42, 38, 34, 30, 26, 22, 17, 13, 9, 4,
//									   -0, -4, -9, -13, -17, -22, -26, -30, -34, -38, -42, -46, -50, -54, -57, -61, -64, -68, -71, -74, -77, -79, -82, -84, -87, -89, -91, -92, -94, -95, -97, -98, -98, -99, -100, -100, -100, -100, -100, -99, -98, -98, -97, -95, -94, -92, -91, -89, -87, -84, -82, -79, -77, -74, -71, -68, -64, -61, -57, -54, -50, -46, -42, -38, -34, -30, -26, -22, -17, -13, -9, -4};

//int max_sin_correct[50] = {-10,-10,-10,-10,-10,-10,-10,-10,-10,-10,-8,-6,-6,-5,6,6,6,33,35,40,60,60,60,60,60,60,60,60,60,60,60,60,60,60,60,60,60,60,60,60,60,60,60,60,60,60,60,60,60,60};
//int max_sin_correct[50] = {-30,-30,-30,-30,-30,-30,-30,-30,-30,-30,-8,-6,-6,-5,6,6,6,33,35,40,60,60,60,60,60,60,60,60,60,60,60,60,60,60,60,60,60,60,60,60,60,60,60,60,60,60,60,60,60,60};


//float select_freq = INIT_HZ;
//float select_period = INIT_PERIOD;
int amount10us_per_idx = INIT_AMOUNT_10US_IDX;
int curr_sin_patt_idx_L1 = 2*SIN_PATTERN_SIZE/3;				//24 gdy 36
int curr_sin_patt_idx_L2 = 1*SIN_PATTERN_SIZE/3;				//12 gdy 36
int curr_sin_patt_idx_L3 = 0;									// 0 gdy 36
//float sin_period = MAX_PERIOD;									//changed to motor.period
//MovAvrg_TypeDef curr_L1_mov_avrg;								//current measured on L1 with moving average operation
//MovAvrg_TypeDef curr_L2_mov_avrg;								//current measured on L2 with moving average operation
//MovAvrg_TypeDef curr_L3_mov_avrg;								//current measured on L3 with moving average operation
Current_TypeDef curr_L1;
Current_TypeDef curr_L2;
Current_TypeDef curr_L3;
int test_msrd_curr_L1[SIN_PATTERN_SIZE];						//test measured current
int test_correction_L1[SIN_PATTERN_SIZE];						//test correction values for current
int test_corrected[SIN_PATTERN_SIZE];							//test corrected value for pwm
//int test_msrd_curr_L1_count = 0;								//amount of current probes achived
int test_curr_correct_L1 = 0;									//test of current correction through pwm percent change

//int curr_sum = 0;												//sum values of next current probes
//int curr_avg_value = 0;										//current average value
//
///* percent of changing pwm in one step */
//int sin_change_steps[SIN_PATTERN_SIZE] = {1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1,
//		                                  1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1};
//int curr_change_step = CURR_CHANGE_STEP_MAX;					//step * percent of changing pwm
int sin_pattern_corrected_L1;									//with corrected current value
int sin_pattern_corrected_L2;									//with corrected current value
int sin_pattern_corrected_L3;									//with corrected current value
int pwm_value_L1;
int pwm_value_L2;
int pwm_value_L3;
//int proc_L1;
//int proc_L2;
//int proc_L3;
//int proc_L1_value;
//int proc_L2_value;
//int proc_L3_value;
//int action = 0;
//int action_status = 0;										//for debug rs232
//int motor_stopped = 1;
int encoder_value = 10;
//int event_encoder_check = EVENT_WAIT;
//int event_current_check = EVENT_WAIT;
//enum motor_state_t { MOTOR_OFF, MOTOR_STOPPED, MOTOR_RUN } motor_state = MOTOR_OFF;
ENCODER_InitTypeDef ENCODER_InitStructure;						//encoder for setting speed

volatile int started = 0;
volatile int add_perc = 0;
volatile float current = 0;
volatile int nominal_current = 0;
volatile int proportional_current = 0;							//test current proportional to delay
volatile uint32_t max_delay_value = 0;							//test MAX_DELAY dynamicznego

float const throttle_factor = PAIR_OF_POLES * ((float)MAX_MECH_FREQ / (THROTTLE_MAX - THROTTLE_OFFSET));

//float percent_throttle;
//float over_curr;

void HAL_TIM_PeriodElapsedCallback(TIM_HandleTypeDef *htim)
{
	if (htim->Instance == TIM_INTERVAL)
	{
		/* increment, rest in main */
		count10us++;

		/* pwm generating */
		count10us_per_idx++;
	}
}


void motor_control_init(void)
{
	/* encoder for setting speed */

	ENCODER_InitStructure.Port = GPIOC;
	ENCODER_InitStructure.Pin_Ch1 = GPIO_PIN_4;
	ENCODER_InitStructure.Pin_Ch2 = GPIO_PIN_5;
	ENCODER_InitStructure.Pulse_Width = 5;							//5 * 1ms pulse width checked in 1ms interval
	ENCODER_Init(&ENCODER_InitStructure);

	/* PI controller */

	curr_L1.pi_ctrl.kp_up = KP_UP;
	curr_L1.pi_ctrl.ki_up = KI_UP;
	curr_L1.pi_ctrl.kp_down = KP_DOWN;
	curr_L1.pi_ctrl.ki_down = KI_DOWN;
//	curr_L2.pi_ctrl.kp_up = KP_UP;
//	curr_L2.pi_ctrl.ki_up = KI_UP;
//	curr_L3.pi_ctrl.kp_up = KP_UP;
//	curr_L3.pi_ctrl.ki_up = KI_UP;

}



void motor_control_start(void)
{

}



void motor_control_stop(void)
{

}



void motor_control_speed(uint32_t speed)
{

}


void ctrl_pwm_L1_half1(uint32_t value)
{
	CCR_L1_GATE_U = value;
	CCR_L1_GATE_D = PWM_LOW_LEVEL;
}



void ctrl_pwm_L1_half2(uint32_t value)
{
	CCR_L1_GATE_U = PWM_LOW_LEVEL;
	CCR_L1_GATE_D = value;
}



void ctrl_pwm_L2_half1(uint32_t value)
{
	CCR_L2_GATE_U = value;
	CCR_L2_GATE_D = PWM_LOW_LEVEL;
}



void ctrl_pwm_L2_half2(uint32_t value)
{
	CCR_L2_GATE_U = PWM_LOW_LEVEL;
	CCR_L2_GATE_D = value;
}



void ctrl_pwm_L3_half1(uint32_t value)
{
	CCR_L3_GATE_U = value;
	CCR_L3_GATE_D = PWM_LOW_LEVEL;
}



void ctrl_pwm_L3_half2(uint32_t value)
{
	CCR_L3_GATE_U = PWM_LOW_LEVEL;
	CCR_L3_GATE_D = value;
}



void next_L1_sin_index(void)
{
	if (curr_sin_patt_idx_L1 < SIN_PATTERN_SIZE - 1)
		curr_sin_patt_idx_L1++;
	else
		curr_sin_patt_idx_L1 = 0;
}



void next_L2_sin_index(void)
{
	if (curr_sin_patt_idx_L2 < SIN_PATTERN_SIZE - 1)
		curr_sin_patt_idx_L2++;
	else
		curr_sin_patt_idx_L2 = 0;
}



void next_L3_sin_index(void)
{
	if (curr_sin_patt_idx_L3 < SIN_PATTERN_SIZE - 1)
		curr_sin_patt_idx_L3++;
	else
		curr_sin_patt_idx_L3 = 0;
}



int get_sin_pattern_corrected_L1(void)
{
	//sin_correct = get_potent_correction(); //TODO temoporary !!!

	int corrected = sin_pattern[curr_sin_patt_idx_L1] + (float)sin_pattern[curr_sin_patt_idx_L1] * ((float)sin_correct / 100);
	if (corrected < 0) corrected = 0;
	if (corrected > 99) corrected = 99;
	return corrected;
}



int get_sin_pattern_corrected_L2(void)
{
	//sin_correct = get_potent_correction(); //TODO temoporary !!!

	int corrected = sin_pattern[curr_sin_patt_idx_L2] + (float)sin_pattern[curr_sin_patt_idx_L2] * ((float)sin_correct / 100);
	if (corrected < 0) corrected = 0;
	if (corrected > 99) corrected = 99;
	return corrected;
}



int get_sin_pattern_corrected_L3(void)
{
	//sin_correct = get_potent_correction(); //TODO temporary !!!

	int corrected = sin_pattern[curr_sin_patt_idx_L3] + (float)sin_pattern[curr_sin_patt_idx_L3] * ((float)sin_correct / 100);
	if (corrected < 0) corrected = 0;
	if (corrected > 99) corrected = 99;
	return corrected;
}


/* detect begin each half of sinusoide */


int is_begin_new_half_L1(void)
{
	static int begin_half_L1;

	if (curr_sin_patt_idx_L1 == 0 && begin_half_L1 != 1)								//begin first half of sinusoide
	{
		begin_half_L1 = 1;																//mark moment for only one call
		return begin_half_L1;															//return with number
	}
	else
	if (curr_sin_patt_idx_L1 == (SIN_PATTERN_SIZE/2) && begin_half_L1 != 2)				//begin second half of sinusoide
	{
		begin_half_L1 = 2;																//mark moment for only one call
		return begin_half_L1;															//return with number
	}
	else
		return 0;																		//return result its not begin
}



uint32_t get_L1_current(void)
{
	if (mc_init.adc_L1 != 0)
	{
		int shifted = get_current_shifted_idx(curr_sin_patt_idx_L1);

		/* real current */

		int adc_curr_L1 = HAL_ADC_GetValue(mc_init.adc_L1) - CURR_MEASURE_OFFSET;
		calc_mov_avrg(&curr_L1.mov_avrg, adc_curr_L1);										//moving average from last 3 probes
		curr_L1.probes[shifted] = curr_L1.mov_avrg.value;									//save to probes TODO only to show in STM32Studio?
		calc_avrg_curr(&curr_L1.avrg_curr, curr_L1.mov_avrg.value);							//average current from whole periode, 144 probes
		current = get_current_value(curr_L1.avrg_curr.value);								//get real current in A
//		int target  = calc_target_sin_value_L1(560, shifted);
//		int correct = calc_correction(&curr_L1, target, curr_L1.mov_avrg.value);
//		int percent = correction_to_percent(target, correct);
//		curr_L1.corrections[shifted] = percent;
//		update_correction(shifted, percent);

		/* simulated current */

//		float adc_potent = HAL_ADC_GetValue(mc_init.adc_L2) / 409.6;	//0.0 .. 10.0
//		curr_L1.probes[shifted] = adc_potent * sin_simul_curr[shifted];
//		int target  = calc_target_sin_value_L1(260, shifted);
//		int correct = calc_correction(&curr_L1, target, curr_L1.probes[shifted]);
//		int percent = correction_to_percent(target, correct);
//		curr_L1.corrections[shifted] = percent;
//		update_correction(shifted, percent);

		/* correction current calculation */



		/* only for test in STM Studio */

//		if (count1s_current >= 1)
//		{
//			for (int k = 0; k < SIN_PATTERN_SIZE / 4; k++)
//			{
//				test_msrd_curr_L1[k] = curr_L1.probes[k];
//				test_correction_L1[k] = curr_L1.corrections[k];
//			}
//
//			count1s_current = 0;								//wait for next fillment
//		}
	}

	return 0;
}



uint32_t get_L2_current(void)
{
	if (mc_init.adc_L2 != 0)
		return HAL_ADC_GetValue(mc_init.adc_L2);
	else
		return 0;
}



uint32_t get_L3_current(void)
{
	if (mc_init.adc_L3 != 0)
		return HAL_ADC_GetValue(mc_init.adc_L3);
	else
		return 0;
}



float get_current_value(int adc_value)
{
	float val_per_1a = POLOLU_VAL_PER_1A;
	float result = adc_value / val_per_1a;							//proportional: 1A = POLOLU_VAL_PER_1A, xA = value_int
	return result;
}



/* remove phase shift between U and I */



int get_current_shifted_idx(int sin_patt_idx)
{
	int shifted = sin_patt_idx - CURR_SHIFT;

	if (shifted >= 0)
		return shifted;
	else
		return SIN_PATTERN_SIZE + shifted;
}


/* example 260 * 45/100 */


int calc_target_sin_value_L1(int total_peak, int curr_idx)
{
//	if (curr_idx < SIN_PATTERN_SIZE / 2)
		return total_peak * ((float)sin_pattern_current[curr_idx] / 100);
//	else
//		return -total_peak * ((float)sin_pattern_current[curr_idx] / 100);
}



/* how many percent of target is correction value
 * target = 100%
 * correct = x%
 * x% = (correct * 100) / target
 */


int correction_to_percent(int target, int correct)
{
	int perc = (correct * 100) / target;
	return perc;
}


/* test correction of current with pcb potentiometer */


int get_potent_correction()
{
	int middle = 2048;
	int one_perc = 21;

	int value = ((int)adc1_values[ADC_POTENT1] - middle) / one_perc;

	return value;
}


/* teraz MAX_DELAY obliczany procentowo */


uint32_t max_delay(void)
{
	/* z potencjometru */
//	uint32_t potent_max = 4096;
//	uint32_t delay_max = 40;
//	uint32_t result = (adc1_values[ADC_POTENT1] * delay_max) / potent_max;
//	if (result < 1) result = 1;
//	return result;

	/* obliczany dynamicznie dla zadanej wartosci procentowej */
	uint32_t result = encoder.freq * MAX_DELAY_PROC;
	if (result < MIN_DELAY) result = MIN_DELAY;
	return result;
}



void ctrl_pwm_all_channels()
{
	//follow sin patterns L1..L3

	next_L1_sin_index();
	next_L2_sin_index();
	next_L3_sin_index();

	//example -50% from 98: corrected = 98 + 98 * -50/100 = 98 + 98 * -0.5 = 98 + -49 = 49

	sin_pattern_corrected_L1 = get_sin_pattern_corrected_L1();
	sin_pattern_corrected_L2 = get_sin_pattern_corrected_L2();
	sin_pattern_corrected_L3 = get_sin_pattern_corrected_L3();

	pwm_value_L1 = TIM_PWM_PERIOD * ((float)sin_pattern_corrected_L1 / 100);
	pwm_value_L2 = TIM_PWM_PERIOD * ((float)sin_pattern_corrected_L2 / 100);
	pwm_value_L3 = TIM_PWM_PERIOD * ((float)sin_pattern_corrected_L3 / 100);

	if (curr_sin_patt_idx_L1 < (SIN_PATTERN_SIZE/2) )
		ctrl_pwm_L1_half1(pwm_value_L1);
	else
		ctrl_pwm_L1_half2(pwm_value_L1);

	if (curr_sin_patt_idx_L2 < (SIN_PATTERN_SIZE/2))
		ctrl_pwm_L2_half1(pwm_value_L2);
	else
		ctrl_pwm_L2_half2(pwm_value_L2);

	if (curr_sin_patt_idx_L3 < (SIN_PATTERN_SIZE/2))
		ctrl_pwm_L3_half1(pwm_value_L3);
	else
		ctrl_pwm_L3_half2(pwm_value_L3);

}



void set_speed(Throttle_TypeDef *th)
{
	amount10us_per_idx = (th->period/(float)SIN_PATTERN_SIZE)/VAL_10US;

	motor.period = th->period;
	motor.freq = th->freq;
	motor.freq_mech = th->freq_mech;
}


/* calculate new reduced or increased speed */


Throttle_TypeDef* factor_speed(float factor)
{
	reduced.freq = motor.freq + factor;
	reduced.period = 1 / reduced.freq;
	reduced.freq_mech = reduced.freq / PAIR_OF_POLES;

	return &reduced;
}


/* control speed given to motor in reference to motor delay (poslizg) */


void ctrl_speed(Throttle_TypeDef *th)
{
//	/* normal operating */
//
//	if (!stopped() && !starting())
//	{
//		if (motor_delay_avrg.value > 30 && motor_delay_avrg.value < 80 &&	//poslizg przekroczony (przez przeciazenie lub gwaltowne przyspieszanie)
//			motor.freq > FREQ_MIN)
//		{
//			set_speed(factor_speed(FACTOR_SPEED_DOWN));			//nie przekraczaj dozwolonego poslizgu, zmniejszaj czestotl. podawana na silnik
//		}
//		else													//poslizg w normie lub wrocil do normy
//		{
//			if (throttle.freq - motor.freq > 5)					//poslizg wrocil do normy ale zadajnik chce szybciej, zwiekszaj czestotl. podawana na silnik
//				set_speed(factor_speed(FACTOR_SPEED_UP));
//			else												//poslizg w normie wiec ustaw od razu czestotl. podawana na silnik
//				set_speed(&throttle);
//		}
//	}
//	else
//
//	/* keep always minimum freq to motor */
//
//	{
//		set_speed(&throttle_min);
//	}



	/* normal operating */

	if (!stopped() && !starting())
	{
		set_speed(th);
	}
	else

	/* keep always minimum freq to motor */

	{
		set_speed(&throttle_min);
	}

}



void check_throttle(void)
{
//	encoder_value += ENCODER_GetStatus(&ENCODER_InitStructure);
//
//	//pobierz zadana predkosc dla silnika
//
//	select_freq = encoder_value * 5;							//0, 5, 10, 15, 20, 25, ...
//	if (select_freq < 1) select_freq = 1;
//	if (select_freq > 100) select_freq = 100;
//	select_period = 1/(float)select_freq;
//
//	if (select_freq < 5)										//motor off
//	{
//		set_speed_ms(MAX_PERIOD);
//	}
//	else														//motor on
//	{
//		set_speed_ms(select_period);
//	}

	throttle.adc = adc1_values[ADC_THROTTLE];

	/* prevent values less than zero and calculate new value starting from zero */

	if (throttle.adc >= THROTTLE_OFFSET)
		throttle.new_value = throttle.adc - THROTTLE_OFFSET;
	else
		throttle.new_value = 0;

	calc_mov_avrg(&throttle_avrg, throttle.new_value);

	throttle.freq = throttle_avrg.value * throttle_factor;
	throttle.period = 1 / throttle.freq;
	throttle.freq_mech = throttle.freq / PAIR_OF_POLES;

	max_delay_value = max_delay();

	if (stopped() || starting() || (throttle.freq_mech <= (encoder.freq + max_delay_value)))
	{
		ctrl_speed(&throttle);

		throttle.last_value = throttle_avrg.value;
	}
	else
	{
		throttle_reduced.freq_mech = encoder.freq + max_delay_value;
		throttle_reduced.freq = throttle_reduced.freq_mech * PAIR_OF_POLES;
		throttle_reduced.period = 1 / throttle_reduced.freq;

		ctrl_speed(&throttle_reduced);
	}
}


/* main encoder calculations */


void check_encoder(void)
{
	encoder.cnt = mc_init.tim_encoder->Instance->CNT;													//0..403
	encoder.dir = mc_init.tim_encoder->Instance->CR1;													//1 CW, 17 CCW
	mc_init.tim_encoder->Instance->CNT = 0;																//reset timer every 10ms

	encoder.cnt -= 2;																					//korekta aby nie rwać predkoscia kiedy enkoder wyprzedza throttle bo wtedy nie jest podawany prad

	if (encoder.dir == ENC_DIR_FORWARD)
	{
		calc_mov_avrg(&encoder_avrg, encoder.cnt);														//calculate moving average

		//if (encoder_avrg.value != 0)																	//TODO what if = 0
		{
			if (ENC_SIMULATION)
				encoder.full_rotation_time = throttle.period * PAIR_OF_POLES;
			else
				encoder.full_rotation_time = (ENC_IMP_PER_ROTATION * ENC_PERIOD_TIME) / encoder_avrg.value;	//40imp in 10ms, 400imp in xs (= 0.1s)

			encoder.freq = 1 / encoder.full_rotation_time;												//1 rotat. in full_rotation_time (0.1s), x rotat. in 1s (= 10Hz)
//			encoder.delay = motor.freq_mech - encoder.freq;												//poslizg silnika
			encoder.delay = ((motor.freq_mech - encoder.freq) / motor.freq_mech) * 100;					//poslizg w %
			calc_mov_avrg_float(&motor_delay_avrg, encoder.delay);										//wygladz poslizg motoru aby nie bylo skokow przy dociaganiu pradem
		}
	}
}


/* calculate correction for current (pwm) */


int calc_current_correction(Current_TypeDef *current, int target, int measured)
{
	/* 1. keep constant target current */

	int sto = 100;																		//130 = trik to make biggest influence to control current through pwm
	int perc_of_target = (measured * sto) / target;										//how many percent is measured of target, example 360=100% -> 250=69%
	int target_perc = sto - perc_of_target;												//how many missing to 100%, example 100%-69%=31% or 100%-111%=-11%
	int curr_perc = sin_correct;

	/* 2. additional increase target current proportional to motor delay (poslizg) */

//	if (motor_delay_avrg.value > 0)										//TODO uwaga wykorzystane tylko jako wygladzacz/opozniacz bo nadal surowy encoder.delay aby natychmiastowa reakcja przy zwieksz. obc.
//	{
//		target_perc += encoder.delay * OVER_CURRENT_FACTOR;
//		target_perc += motor_delay_avrg.value * 6;
//		target_perc += encoder.delay * 5;

//		float delay_sel;
//		if (encoder.delay >= motor_delay_avrg.value)
//			delay_sel = encoder.delay;
//		else
//			delay_sel = motor_delay_avrg.value;
//		target_perc += delay_sel * 6;
//	}



//	if (motor_delay_avrg.value > 10)
//	{
//		if (add_perc < 200)					//TODO ile max
//			add_perc++;
//	}
//	else
//	if (motor_delay_avrg.value < 5)
//	{
//		if (add_perc > 0)
//			add_perc--;
//	}
//
//	target_perc += add_perc;

	/* 3. result from PI regulator */

	int pi_value = get_PI_ctrl(&current->pi_ctrl, target_perc, curr_perc);

	if (driving())
		return pi_value;
	else
		return -99;											//zwolnienie gazu, aby nie palic bezpiecznikow (nawet 15A) podczas hamowania i nie haratac paskiem
}


/* control current during starting and constant current during normal work */


void ctrl_current(void)
{
	/* Step 1. stop, current zero */

	if (stopped())
	{
		nominal_current = MINIMAL_CURRENT;
		sin_correct = CURR_STOP;
	}
	else

	/* Step 2. starting, nominal current inceased proportional */

	if (starting())
	{
		float throttle_range = FREQ_MIN - FREQ_STOP;
		float current_range = NOMINAL_CURRENT - MINIMAL_CURRENT;
		float throttle_val = throttle.freq - FREQ_STOP;
		float current_val = MINIMAL_CURRENT + (throttle_val * current_range / throttle_range);

		nominal_current = current_val;
	}
	else

	/* Step 3. operating with normal current increased proportional */

	{
		//nominal_current = NOMINAL_CURRENT;
//		if (throttle.freq <= 30)						//TODO TEST
//			nominal_current = NOMINAL_CURRENT;
//		else
//			nominal_current = NOMINAL_CURRENT + 800;

		//nominal_current = calc_nominal_current();

		float throttle_range = (MAX_MECH_FREQ * PAIR_OF_POLES) - FREQ_MIN;
		float current_range = NOMINAL_CURRENT_MAX - NOMINAL_CURRENT;
		float throttle_val = throttle.freq - FREQ_MIN;
		float current_val = NOMINAL_CURRENT + (throttle_val * current_range / throttle_range);

		nominal_current = current_val;
	}

	/* Work - dynamic current correction under variable load */

	{
		/* current proportional to motor delay/load */

		proportional_current = nominal_current * (motor_delay_avrg.value / 100) * 3;				//proportional_current = nominal_current * % delay * factor more

		test_curr_correct_L1 = calc_current_correction(&curr_L1, nominal_current + proportional_current, curr_L1.avrg_curr.value);
		sin_correct += test_curr_correct_L1;

		if (sin_correct > 99) sin_correct = 99;
		if (sin_correct < -99) sin_correct = -99;
	}

}



int stopped(void)
{
	if (throttle.freq < FREQ_STOP)
		return 1;
	else
		return 0;
}



int starting(void)
{
	if ((throttle.freq >= FREQ_STOP) && (throttle.freq < FREQ_MIN))
		return 1;
	else
		return 0;
}



int driving(void)
{
	return encoder.delay >= 0;
}


///* calculate proportional increased nominal_current. Now it is same as Step 2. in ctrl_current() !!! */
///* TODO ZOPTYMALIZOWAC. obie zmianne jako int licza wtedy co 10. Przeniesc je z globalnych tutaj */
//
//
//int calc_nominal_current(void)
//{
//	float nominal_curr = NOMINAL_CURRENT;
//
//	if (throttle.freq >= FREQ_MIN)
//	{
//		percent_throttle = ((throttle.freq - FREQ_MIN) * 100) / ((MAX_MECH_FREQ * PAIR_OF_POLES) - FREQ_MIN);
//		over_curr = (percent_throttle * (NOMINAL_CURRENT_MAX - NOMINAL_CURRENT)) / 100;
//		nominal_curr += over_curr;
//	}
//
//	return (int)nominal_curr;
//}


/**
 * main loop
 */


void usart_send_status(void)
{
	static int usart_cnt;
	static uint8_t buf_tx[4];

	if (usart_cnt++ >= 10)
	{
		usart_cnt = 0;

		buf_tx[0] = throttle.freq_mech;
		buf_tx[1] = motor.freq_mech;
		buf_tx[2] = encoder.freq;
		//buf_tx[3] = curr_L1.avrg_curr.value;
		buf_tx[3] = current;

		HAL_UART_Transmit_DMA(mc_init.usart, buf_tx, 4);
	}
}


void motor_control_loop(void)
{
	/* PWM sine generating - next sin index */

	if (count10us_per_idx > amount10us_per_idx)	//TODO >= ?
	{
		count10us_per_idx = 0;

		ctrl_pwm_all_channels();

		get_L1_current();
		get_L2_current();
		get_L3_current();

		/* current controlled after gathering each half sinusoide */

		if (is_begin_new_half_L1() > 0)
			ctrl_current();

	}

	/* intervals for main */

	if (count10us >= 100)												/* 1ms period */
	{
		count10us = 0;

		if (++count1ms >= 10)											/* 10ms period */
		{
			count1ms = 0;

			check_throttle();
			check_encoder();
			usart_send_status();

			if (++count10ms >= 100)										/* 1s period */
			{
				count10ms = 0;

				HAL_GPIO_TogglePin(LD2_GPIO_Port, LD2_Pin);
			}
		}

	}


}

