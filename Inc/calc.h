/*
 * calc.h
 *
 *  Created on: 8 sty 2017
 *      Author: r.lapucha
 */

#ifndef CALC_H_
#define CALC_H_


#include <stdio.h>
#include <stdlib.h>                            //abs() function
#include "params.h"


#define MOV_AVRG_SIZE                8         //moving average sum of elements
#define MOV_AVRG_FLOAT_SIZE          25


/* moving average calculations */


typedef struct
{
    int value;

    /* private */

    int _values[MOV_AVRG_SIZE];
    int _idx;
    int _sum;

} MovAvrg_TypeDef;


/* float moving average */


typedef struct
{
    float value;

    /* private */

    float _values[MOV_AVRG_FLOAT_SIZE];
    int _idx;
    float _sum;

} MovAvrgFloat_TypeDef;


/* average current from 144/2 probes */


typedef struct
{
    int value;

    /* private */

    int _values[SIN_PATTERN_SIZE/2];
    int _idx;
    int _sum;

} AvrgCurr_TypeDef;


/* PI controller */


typedef struct
{
    float kp_up;
    float ki_up;
    float kp_down;
    float ki_down;

    /* private */

    float kp;
    float ki;
    int diff;
    int diff_sum;
    int p;
    int i;
    int value;
    int last_target;

} PI_TypeDef;


void calc_mov_avrg(MovAvrg_TypeDef *av, int new_value);
void calc_avrg_curr(AvrgCurr_TypeDef *av, int new_value);
void calc_mov_avrg_float(MovAvrgFloat_TypeDef *av, float new_value);
int get_PI_ctrl(PI_TypeDef *pi_ctrl, int target, int measured);


#endif /* CALC_H_ */

