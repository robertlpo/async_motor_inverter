/*
 * calc.c
 *
 *  Created on: 8 sty 2017
 *      Author: r.lapucha
 *
 *  Calculations library
 *
 */


#include "calc.h"


/* moving average calculations */


void calc_mov_avrg(MovAvrg_TypeDef *av, int new_value)
{
	/* find next/oldest index */

	if (av->_idx < MOV_AVRG_SIZE - 1)
		av->_idx++;
	else
		av->_idx = 0;

	av->_sum -= av->_values[av->_idx];				//remove from sum oldest value
	av->_sum += new_value;							//add to sum newest value
	av->_values[av->_idx] = new_value;				//save newest value

	av->value = av->_sum / MOV_AVRG_SIZE;
}


/* float version */


void calc_mov_avrg_float(MovAvrgFloat_TypeDef *av, float new_value)
{
	/* find next/oldest index */

	if (av->_idx < MOV_AVRG_FLOAT_SIZE - 1)
		av->_idx++;
	else
		av->_idx = 0;

	av->_sum -= av->_values[av->_idx];				//remove from sum oldest value
	av->_sum += new_value;							//add to sum newest value
	av->_values[av->_idx] = new_value;				//save newest value

	av->value = av->_sum / MOV_AVRG_FLOAT_SIZE;
}


/* average current of 144/2 probes */


void calc_avrg_curr(AvrgCurr_TypeDef *av, int new_value)
{
	int abs_new_value = abs(new_value);

	/* find next/oldest index */

	if (av->_idx < (SIN_PATTERN_SIZE/2) - 1)
		av->_idx++;
	else
		av->_idx = 0;

	av->_sum -= av->_values[av->_idx];				//remove from sum oldest value
	av->_sum += abs_new_value;						//add to sum newest value
	av->_values[av->_idx] = abs_new_value;			//save newest value

	av->value = av->_sum / (SIN_PATTERN_SIZE/2);
}


/* PI controller - gdy regulacja nie przekracza dwukrotnie wartosci zadanej lub zadana maleje to licz
 * ale w przciwnym warunku wyczysc sume calkowania bo to przez nia i zwroc ostatnia dobra wartosc */


//float get_PI_ctrl(PI_TypeDef *pi_ctrl, float target, float measured)
//{
//	pi_ctrl->diff = target - measured;
//
//	if ((pi_ctrl->value < 2 * target) || (target < pi_ctrl->last_target))
//	{
//		pi_ctrl->diff_sum += pi_ctrl->diff;
//		pi_ctrl->p = pi_ctrl->kp * pi_ctrl->diff;
//		pi_ctrl->i = pi_ctrl->ki * pi_ctrl->diff_sum;
//		pi_ctrl->value = pi_ctrl->p + pi_ctrl->i;
//	}
//	else
//		pi_ctrl->diff_sum = 0;
//
//	pi_ctrl->last_target = target;
//
//	return pi_ctrl->value;
//}

int get_PI_ctrl(PI_TypeDef *pi_ctrl, int target, int measured)
{
	if (target >= measured)
	{
		pi_ctrl->kp = pi_ctrl->kp_up;
		pi_ctrl->ki = pi_ctrl->ki_up;
	}
	else
	{
		pi_ctrl->kp = pi_ctrl->kp_down;
		pi_ctrl->ki = pi_ctrl->ki_down;
	}

	pi_ctrl->diff = target - measured;

	pi_ctrl->diff_sum += pi_ctrl->diff;
	pi_ctrl->p = pi_ctrl->kp * pi_ctrl->diff;
	pi_ctrl->i = pi_ctrl->ki * pi_ctrl->diff_sum;
	pi_ctrl->value = pi_ctrl->p + pi_ctrl->i;

	pi_ctrl->last_target = target;

	return pi_ctrl->value;
}
