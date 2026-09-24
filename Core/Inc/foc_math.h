/*
 * foc_math.h
 *
 * Coordinate transforms used by field-oriented control.
 */

#ifndef FOC_MATH_H_
#define FOC_MATH_H_

typedef struct
{
    float a;
    float b;
    float c;
} Phase3;

typedef struct
{
    float alpha;
    float beta;
} AlphaBeta;

typedef struct
{
    float d;
    float q;
} DqAxis;

AlphaBeta foc_clarke(Phase3 phase);

DqAxis foc_park(AlphaBeta stationary, float sin_theta, float cos_theta);

AlphaBeta foc_inverse_park(DqAxis rotating,
                           float sin_theta,
                           float cos_theta);

Phase3 foc_inverse_clarke(AlphaBeta stationary);

#endif /* FOC_MATH_H_ */
