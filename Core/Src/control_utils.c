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
	pi->anti_windup_gain = 0.0f;
	pi->last_unsaturated_output = 0.0f;
	pi->last_output = 0.0f;
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
	pi->last_unsaturated_output = output;
	pi->last_output = mc_clamp_f32(output,
								   pi->output_min,
								   pi->output_max);

	return pi->last_output;
}

void pi_controller_set_anti_windup(PI_Controller *pi,
                                    float anti_windup_gain)
{
    if (pi == 0)
    {
        return;
    }

    pi->anti_windup_gain =
        (anti_windup_gain < 0.0f) ? 0.0f : anti_windup_gain;
}

void pi_controller_apply_output_feedback(PI_Controller *pi,
                                          float applied_output,
                                          float dt_seconds)
{
    if (pi == 0)
    {
        return;
    }

    applied_output = mc_clamp_f32(applied_output,
                                  pi->output_min,
                                  pi->output_max);

    /*
     * Back-calculation feeds the difference between the voltage requested by
     * the PI and the voltage actually applied after vector limiting back into
     * the integrator.
     */
    pi->integral += pi->anti_windup_gain *
                    (applied_output - pi->last_unsaturated_output) *
                    dt_seconds;
    pi->integral = mc_clamp_f32(pi->integral,
                                pi->output_min,
                                pi->output_max);
    pi->last_output = applied_output;
}
