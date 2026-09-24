/*
 * control_utils.h
 *
 *  Created on: 2026年9月23日
 *      Author: 83734
 */

#ifndef CONTROL_UTILS_H_
#define CONTROL_UTILS_H_

typedef struct
{
	float kp;
	float ki;
	float integral;
	float output_min;
	float output_max;
} PI_Controller;

float mc_clamp_f32(float value, float min_value, float max_value);

void pi_controller_init(PI_Controller *pi,
						float kp,
						float ki,
						float output_min,
						float output_max);

float pi_controller_update(PI_Controller *pi,
							float error,
							float dt_seconds);

#endif /* CONTROL_UTILS_H_ */
