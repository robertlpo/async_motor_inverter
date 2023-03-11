/*
 * peripherals.h
 *
 *  Created on: 29 gru 2016
 *      Author: r.lapucha
 *
 *  Microcontroller and board peripherals
 *
 */

#ifndef PERIPHERALS_H_
#define PERIPHERALS_H_


#define TIM_ENCODER      TIM1                    /* motor speed encoder */
#define TIM_INTERVAL     TIM2                    /* standard timer for main loop interval */

#define TIM_PWM_L1       TIM3                    /* timer PWM transistor phase L1 */
#define TIM_PWM_L2       TIM3                    /* timer PWM transistor phase L2 */
#define TIM_PWM_L3       TIM4                    /* timer PWM transistor phase L3 */
#define CCR_L1_GATE_U    TIM3->CCR1              /* PWM % duty register transistor L1 Up */
#define CCR_L1_GATE_D    TIM3->CCR2              /* PWM % duty register transistor L1 Down */
#define CCR_L2_GATE_U    TIM3->CCR3              /* PWM % duty register transistor L2 Up */
#define CCR_L2_GATE_D    TIM3->CCR4              /* PWM % duty register transistor L2 Down */
#define CCR_L3_GATE_U    TIM4->CCR1              /* PWM % duty register transistor L3 Up */
#define CCR_L3_GATE_D    TIM4->CCR2              /* PWM % duty register transistor L3 Down */

#endif /* PERIPHERALS_H_ */
