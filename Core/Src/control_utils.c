/*
 * control_utils.c
 *
 *  Created on: 2026年9月23日
 *      Author: 83734
 */

#include "control_utils.h"

float mc_clamp_f32(float value, float min_value, float max_value)
{
	if (value < min_value)
	{
		return min_value;
	}

	if (value > max_value)
	{
		return max_value;
	}

	return value;
}

void pi_controller_init(PI_Controller *pi,
						float kp,
						float ki,
						float output_min,
						float output_max)
{
	pi->kp = kp;
	pi->ki = ki;
	pi->integral = 0.0f;
	pi->output_min = output_min;
	pi->output_max = output_max;
}

float pi_controller_update(PI_Controller *pi,
							float error,
							float dt_seconds)
{
	pi->integral += pi->ki * error * dt_seconds;
	pi->integral = mc_clamp_f32(pi->integral,
								pi->output_min,
								pi->output_max);

	float output = pi->kp * error + pi->integral;

	return mc_clamp_f32(output,
						pi->output_min,
						pi->output_max);
}
