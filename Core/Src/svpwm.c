/*
 * svpwm.c
 *
 * Centered space-vector PWM using zero-sequence injection.
 */

#include "svpwm.h"

#include <math.h>

#define SVPWM_PI (3.14159265359f)
#define SVPWM_TWO_PI (6.28318530718f)
#define SVPWM_HALF (0.5f)
#define SVPWM_SQRT3_OVER_2 (0.86602540378f)
#define SVPWM_MIN_DUTY (0.0f)
#define SVPWM_MAX_DUTY (1.0f)

static float svpwm_clamp(float value, float minimum, float maximum)
{
    if (value < minimum)
    {
        return minimum;
    }

    if (value > maximum)
    {
        return maximum;
    }

    return value;
}

static uint8_t svpwm_sector_from_angle(float angle)
{
    float normalized = angle;

    while (normalized < 0.0f)
    {
        normalized += SVPWM_TWO_PI;
    }

    while (normalized >= SVPWM_TWO_PI)
    {
        normalized -= SVPWM_TWO_PI;
    }

    if (normalized < SVPWM_PI / 3.0f)
    {
        return 1U;
    }

    if (normalized < 2.0f * SVPWM_PI / 3.0f)
    {
        return 2U;
    }

    if (normalized < SVPWM_PI)
    {
        return 3U;
    }

    if (normalized < 4.0f * SVPWM_PI / 3.0f)
    {
        return 4U;
    }

    if (normalized < 5.0f * SVPWM_PI / 3.0f)
    {
        return 5U;
    }

    return 6U;
}

SvpwmOutput svpwm_calculate(float v_alpha,
                            float v_beta,
                            float v_bus)
{
    SvpwmOutput output = {
        .duty_a = SVPWM_HALF,
        .duty_b = SVPWM_HALF,
        .duty_c = SVPWM_HALF,
        .sector = 0U
    };

    if (v_bus <= 0.0f)
    {
        return output;
    }

    /*
     * First convert the alpha-beta vector to the three phase voltage
     * references. These are centered around zero volts.
     */
    const float phase_a = v_alpha;
    const float phase_b = -SVPWM_HALF * v_alpha
                        + SVPWM_SQRT3_OVER_2 * v_beta;
    const float phase_c = -SVPWM_HALF * v_alpha
                        - SVPWM_SQRT3_OVER_2 * v_beta;

    float phase_max = fmaxf(phase_a, fmaxf(phase_b, phase_c));
    float phase_min = fminf(phase_a, fminf(phase_b, phase_c));

    /*
     * The common-mode voltage keeps the three references centered in the
     * available DC bus. This is the zero-sequence form of centered SVPWM.
     */
    const float common_mode = SVPWM_HALF * (phase_max + phase_min);

    output.duty_a = SVPWM_HALF
                  + (phase_a - common_mode) / v_bus;
    output.duty_b = SVPWM_HALF
                  + (phase_b - common_mode) / v_bus;
    output.duty_c = SVPWM_HALF
                  + (phase_c - common_mode) / v_bus;

    output.duty_a = svpwm_clamp(output.duty_a,
                                SVPWM_MIN_DUTY,
                                SVPWM_MAX_DUTY);
    output.duty_b = svpwm_clamp(output.duty_b,
                                SVPWM_MIN_DUTY,
                                SVPWM_MAX_DUTY);
    output.duty_c = svpwm_clamp(output.duty_c,
                                SVPWM_MIN_DUTY,
                                SVPWM_MAX_DUTY);

    /*
     * Sector is metadata for diagnostics at this stage. The duty calculation
     * above does not need a switch statement for each sector.
     */
    const float angle = atan2f(v_beta, v_alpha);
    output.sector = svpwm_sector_from_angle(angle);

    return output;
}
