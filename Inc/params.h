/*
 * params.h
 *
 *  Created on: 18 gru 2016
 *      Author: r.lapucha
 *
 *  Value parameters for motor control
 *
 */

#ifndef PARAMS_H_
#define PARAMS_H_

//#define SIN_PATTERN_SIZE			36											//360 stopni
#define SIN_PATTERN_SIZE			144
//#define SIN_120_DEGREE			12										    //120 stopni przesuniecia fazowego
#define TIM_PWM_CLK					72000000									//72MHz
//#define TIM_PWM_PERIOD			72000									    //okres = 72MHz/72000 = 1kHz
//#define TIM_PWM_PERIOD			34666									    //okres = 72MHz/34666 = 1,5kHz
//#define TIM_PWM_PERIOD			36000									    //okres = 72MHz/36000 = 2kHz
//#define TIM_PWM_PERIOD			24000									    //okres = 72MHz/24000 = 3kHz
#define TIM_PWM_PERIOD				18000										//okres = 72MHz/18000 = 4kHz UWAGA aby taka sama wartosc byla w period TIM PWM (CubeMX)
//#define TIM_PWM_PERIOD			12000									    //okres = 72MHz/12000 = 6kHz
//#define TIM_PWM_PERIOD			9000									    //okres = 72MHz/9000 = 8kHz
//#define TIM_PWM_PERIOD			7200									    //okres = 72MHz/7200 = 10kHz
//#define TIM_PWM_PERIOD			6000									    //okres = 72MHz/6000 = 12kHz
//#define TIM_PWM_PERIOD			3600									    //okres = 72MHz/3600 = 20kHz
#define PWM_HZ						TIM_PWM_CLK/TIM_PWM_PERIOD					//72MHz / 18000 = 4KHz
#define PWM_HIGH_LEVEL				TIM_PWM_PERIOD + 1							//set high level to igbt
#define PWM_LOW_LEVEL				0											//set low level to igbt
//#define CURR_PROBES_AMOUNT			8										//how many current probes to calculate average value
//#define CURR_CHANGE_STEP_MAX		100
//#define CURR_CHANGE_STEP_MIN		0
#define CURR_MEASURE_OFFSET			1982										//pololu after diode offset
//#define HALL_MIN_ACTIVE_TIME		5											//min active time in ms to accept signal from hall sensor
#define MAX_PERIOD					0.02										//ms min. dozwolony okres, od tego startuje po zatrzymaniu
//#define DELAY_GEN_PWM				1 + 2										//* 10us
//#define START_COUNTING				1
//#define MAX_PERIOD_PERCENT_DIFF		50										//max % of speed difference, default 5
#define MAX_CURRENT_RMS				5
#define MAX_CURRENT_PEAK			MAX_CURRENT_RMS * 1.4142
#define COS_FI						50											//cos fi = 0.64 = 50 degrees
#define CURR_SHIFT					COS_FI/(360/SIN_PATTERN_SIZE)				//current to tension shift. 50/(360/36) = 50/10 = 5 unit in sin array
#define PAIR_OF_POLES				2											//Sh71-4A 4 poles = 2 pair of poles
#define ENC_IMP_PER_ROTATION		400											//how many impulses are reading by software. See configuration of encoder timer
#define ENC_DIR_FORWARD				1											//direction forward
#define ENC_DIR_REVERSE				17											//direction reverse. Sometimes with big load tire reverse.
#define ENC_PERIOD_TIME				0.01										//time of counting encoder impulses - 10ms
#define ENC_SIMULATION				0											//encoder simulation as associated to throttle
#define KP_UP						0.05//0.04									//proportional controller part of PI, im mniej tym mniejszymi krokami dazy do zadanej
#define KI_UP						0.00										//integral controller part of PI
#define KP_DOWN						0.05//0.008
#define KI_DOWN						0.00

#define VAL_10US					0.00001
#define INIT_HZ						50
#define INIT_PERIOD					1/INIT_HZ									//0.02
#define INIT_AMOUNT_10US_IDX		(INIT_PERIOD/VAL_10US)/SIN_PATTERN_SIZE		//20ms/10us=2000/36=55.55

#define ADC_POTENT1					0											//ADC1 DMA array, first test potentiometer on universal pcb
#define ADC_THROTTLE				1											//ADC1 DMA array, throttle potentiometer

#define THROTTLE_OFFSET				980											//offset 0,84V from universal e-bike hall throttle
#define THROTTLE_MAX				3115										//max 2,55V from universal e-bike hall throttle
#define MAX_MECH_FREQ				80//50
//#define MAX_DELAY					8											//%, max motor delay, not increase more speed by throttle
#define MAX_DELAY_PROC				0.23										//max delay 23% see calc in docs
#define MIN_DELAY					8											//min delay 8Hz

#define NOMINAL_CURRENT				100											//normal current to keeping (200 default, 600 OK). 2018-05-05 zmiana z 400 na 100 aby lagodnie jezdzil z malymi V
#define NOMINAL_CURRENT_MAX			1300
#define MINIMAL_CURRENT				10			//TODO TEST
#define OVER_CURRENT_FACTOR			40											//overcurrent factor for keeping speed during increasing of load

#define FACTOR_SPEED_DOWN		   -0.2											//controlling speed in reference to motor delay (poslizg) in steps
#define FACTOR_SPEED_UP				0.5											//controlling speed in reference to speed given from throttle in steps

#define FREQ_STOP					2											//throttle value, under this current to motor are zero
#define FREQ_MIN					10											//throttle value, under this current (torque) are proportional increased to starting move
#define CURR_STOP				   -70											//current zero
#define CURR_MIN				   -8											//max current value to start moving with load under FREQ_MIN

#define POLOLU_ADC_RES				4096										//ADC resoltution
#define POLOLU_MAX_U				3.3											//max. ADC tension
#define POLOLU_U_PER_1A				0.09										//sensor output in mV at 1A current
#define POLOLU_VAL_PER_1A			(POLOLU_ADC_RES * POLOLU_U_PER_1A) / POLOLU_MAX_U	//proportional: 3.3V = 4096, 90mV = x, x = 111,709090
//#define POLOLU_U_ZERO				POLOLU_PWR / 2								//sensor output tension at current 0A
//#define POLOLU_VAL_ZERO			POLOLU_ADC_RES / 2							//ADC value at current 0A

//#define PROBES_PER_IDX			6											//how many current probes per idx
//#define AMOUNT_10US_PROBE			(1/PWM_HZ)/VAL_10US/PROBES_PER_IDX			//6

#endif /* PARAMS_H_ */
