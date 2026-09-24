#include <assert.h>
#include <math.h>
#include <stdio.h>

#include "../Core/Inc/foc_math.h"

#define TEST_EPSILON (0.0001f)

static void assert_near(float actual, float expected)
{
    assert(fabsf(actual - expected) < TEST_EPSILON);
}

static void test_clarke_and_inverse_clarke(void)
{
    Phase3 input = {1.0f, -0.5f, -0.5f};
    AlphaBeta stationary = foc_clarke(input);
    Phase3 output = foc_inverse_clarke(stationary);

    assert_near(stationary.alpha, 1.0f);
    assert_near(stationary.beta, 0.0f);
    assert_near(output.a, input.a);
    assert_near(output.b, input.b);
    assert_near(output.c, input.c);
}

static void test_park_at_zero_angle(void)
{
    AlphaBeta stationary = {1.0f, 0.0f};
    DqAxis rotating = foc_park(stationary, 0.0f, 1.0f);

    assert_near(rotating.d, 1.0f);
    assert_near(rotating.q, 0.0f);
}

static void test_park_rotation(void)
{
    const float angle = 0.5f * 3.14159265359f;
    const float sin_theta = sinf(angle);
    const float cos_theta = cosf(angle);
    AlphaBeta stationary = {0.0f, 1.0f};
    DqAxis rotating = foc_park(stationary, sin_theta, cos_theta);
    AlphaBeta recovered = foc_inverse_park(rotating, sin_theta, cos_theta);

    assert_near(rotating.d, 1.0f);
    assert_near(rotating.q, 0.0f);
    assert_near(recovered.alpha, stationary.alpha);
    assert_near(recovered.beta, stationary.beta);
}

int main(void)
{
    test_clarke_and_inverse_clarke();
    test_park_at_zero_angle();
    test_park_rotation();

    puts("All foc_math tests passed.");
    return 0;
}
