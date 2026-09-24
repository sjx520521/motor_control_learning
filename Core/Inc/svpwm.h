/*
 * svpwm.h
 *
 * Space-vector PWM calculation.
 *
 * The input voltage vector is expressed in the stationary alpha-beta frame.
 * The output duties are normalized to the range [0, 1].
 */

#ifndef SVPWM_H_
#define SVPWM_H_

#include <stdint.h>

typedef struct
{
    float duty_a;
    float duty_b;
    float duty_c;
    uint8_t sector;
} SvpwmOutput;

/*
 * Calculate centered SVPWM duties from an alpha-beta voltage vector.
 *
 * v_bus must be positive. If it is zero or negative, all duties are set
 * to 50 percent and sector is set to zero.
 */
SvpwmOutput svpwm_calculate(float v_alpha,
                            float v_beta,
                            float v_bus);

#endif /* SVPWM_H_ */
