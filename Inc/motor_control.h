/*
 * motor_control.h
 *
 *  Created on: 16 gru 2016
 *      Author: r.lapucha
 */

#ifndef MOTOR_CONTROL_H_
#define MOTOR_CONTROL_H_

#include "stm32f1xx_hal.h"
#include "params.h"
#include "calc.h"


typedef struct
{
    TIM_HandleTypeDef *tim_encoder;
    ADC_HandleTypeDef *adc_L1;
    ADC_HandleTypeDef *adc_L2;
    ADC_HandleTypeDef *adc_L3;
    UART_HandleTypeDef *usart;

} MC_InitTypeDef;

MC_InitTypeDef mc_init;



typedef struct
{
    MovAvrg_TypeDef mov_avrg;                            //current measured with moving average operation, last 3 probes
    AvrgCurr_TypeDef avrg_curr;                          //current measurod from last 144 probes (whole periode)
    int probes[SIN_PATTERN_SIZE];                        //array with current measurements from whole period

    PI_TypeDef pi_ctrl;                                  //PI controller
    int corrections[SIN_PATTERN_SIZE];                   //array with corrections for current

} Current_TypeDef;



typedef struct
{
    uint32_t adc;
    uint32_t last_value;
    uint32_t new_value;
    float freq;                                          //freq readed from throttle
    float period;                                        //period calculated from freq
    float freq_mech;                                     //mechanical freq of engine rotor consider electric pair of poles. 2 pair = 1 pair speed / 2

} Throttle_TypeDef;



typedef struct
{
    uint32_t cnt;                                        //counting impulses
    uint32_t dir;                                        //direction of rotation
    float full_rotation_time;
    float freq;                                          //freq of motor
    float delay;                                         //poslizg silnika czyli roznica predkosci pola magnetycznego i fizycznego obrotu wirnika

} Encoder_TypeDef;


typedef struct
{
    float freq;                                          //electrical freq gived to motor
    float period;                                        //electrical period of electrical freq
    float freq_mech;                                     //mechanical freq of engine rotor consider electric pair of poles. 2 pair = 1 pair speed / 2

} Motor_TypeDef;


uint32_t adc1_values[2];


void motor_control_init(void);
void motor_control_start(void);
void motor_control_loop(void);
void motor_control_stop(void);
void motor_control_speed(uint32_t speed);

void ctrl_pwm_L1_half1(uint32_t value);
void ctrl_pwm_L1_half2(uint32_t value);
void ctrl_pwm_L2_half1(uint32_t value);
void ctrl_pwm_L2_half2(uint32_t value);
void ctrl_pwm_L3_half1(uint32_t value);
void ctrl_pwm_L3_half2(uint32_t value);

void ctrl_pwm_all_channels(void);

void next_L1_sin_index(void);
void next_L2_sin_index(void);
void next_L3_sin_index(void);

int get_sin_pattern_corrected_L1(void);
int get_sin_pattern_corrected_L2(void);
int get_sin_pattern_corrected_L3(void);

uint32_t get_L1_current(void);
uint32_t get_L2_current(void);
uint32_t get_L3_current(void);

void check_throttle(void);
void check_encoder(void);
void ctrl_speed(Throttle_TypeDef *th);
void set_speed(Throttle_TypeDef *th);
Throttle_TypeDef* factor_speed(float factor);
int get_current_shifted_idx(int sin_patt_idx);
int calc_current_correction(Current_TypeDef *current, int target, int measured);
void ctrl_current(void);
int stopped(void);
int starting(void);
int get_potent_correction(void);
float get_current_value(int adc_value);
int is_begin_new_half_L1(void);
int driving(void);
uint32_t max_delay(void);                                //test dynamicznego MAX_DELAY z potencjometru do testow dynamicznych z roznymi predkosciami
void usart_send_status(void);

#endif /* MOTOR_CONTROL_H_ */
