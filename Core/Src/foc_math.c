/*
 * foc_math.c
 *
 * Coordinate transforms used by field-oriented control.
 */

#include "foc_math.h"

#include <math.h>

#define FOC_INV_SQRT3 (0.57735026919f)
#define FOC_SQRT3 (1.73205080757f)

AlphaBeta foc_clarke(Phase3 phase)
{
    AlphaBeta result;

    /*
     * Amplitude-invariant Clarke transform. For a balanced motor current,
     * phase.a + phase.b + phase.c is approximately zero.
     */
    result.alpha = phase.a;
    result.beta = (phase.a + 2.0f * phase.b) * FOC_INV_SQRT3;

    return result;
}

DqAxis foc_park(AlphaBeta stationary, float sin_theta, float cos_theta)
{
    DqAxis result;

    result.d = stationary.alpha * cos_theta
             + stationary.beta * sin_theta;
    result.q = -stationary.alpha * sin_theta
             + stationary.beta * cos_theta;

    return result;
}

AlphaBeta foc_inverse_park(DqAxis rotating,
                           float sin_theta,
                           float cos_theta)
{
    AlphaBeta result;

    result.alpha = rotating.d * cos_theta
                 - rotating.q * sin_theta;
    result.beta = rotating.d * sin_theta
                + rotating.q * cos_theta;

    return result;
}

Phase3 foc_inverse_clarke(AlphaBeta stationary)
{
    Phase3 result;

    result.a = stationary.alpha;
    result.b = -0.5f * stationary.alpha
             + 0.5f * FOC_SQRT3 * stationary.beta;
    result.c = -0.5f * stationary.alpha
             - 0.5f * FOC_SQRT3 * stationary.beta;

    return result;
}
