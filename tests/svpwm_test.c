#include <assert.h>
#include <math.h>
#include <stdio.h>

#include "../Core/Inc/svpwm.h"

#define TEST_EPSILON (0.0001f)

static void assert_near(float actual, float expected)
{
    assert(fabsf(actual - expected) < TEST_EPSILON);
}

static void assert_duty_range(SvpwmOutput output)
{
    assert(output.duty_a >= 0.0f && output.duty_a <= 1.0f);
    assert(output.duty_b >= 0.0f && output.duty_b <= 1.0f);
    assert(output.duty_c >= 0.0f && output.duty_c <= 1.0f);
}

static void test_zero_vector(void)
{
    SvpwmOutput output = svpwm_calculate(0.0f, 0.0f, 24.0f);

    assert_near(output.duty_a, 0.5f);
    assert_near(output.duty_b, 0.5f);
    assert_near(output.duty_c, 0.5f);
    assert_duty_range(output);
}

static void test_alpha_axis(void)
{
    SvpwmOutput output = svpwm_calculate(6.0f, 0.0f, 24.0f);

    assert(output.sector == 1U);
    assert(output.duty_a > output.duty_b);
    assert(output.duty_a > output.duty_c);
    assert_near(output.duty_b, output.duty_c);
    assert_duty_range(output);
}

static void test_sector_boundaries(void)
{
    const float magnitude = 6.0f;
    const float angles[] = {
        0.0f,
        0.5f * 3.14159265359f,
        3.14159265359f,
        1.5f * 3.14159265359f,
        2.0f * 3.14159265359f
    };

    for (unsigned int i = 0U; i < sizeof(angles) / sizeof(angles[0]); ++i)
    {
        SvpwmOutput output = svpwm_calculate(magnitude * cosf(angles[i]),
                                             magnitude * sinf(angles[i]),
                                             24.0f);
        assert_duty_range(output);
    }
}

static void test_invalid_bus_voltage(void)
{
    SvpwmOutput output = svpwm_calculate(5.0f, 2.0f, 0.0f);

    assert_near(output.duty_a, 0.5f);
    assert_near(output.duty_b, 0.5f);
    assert_near(output.duty_c, 0.5f);
    assert(output.sector == 0U);
}

int main(void)
{
    test_zero_vector();
    test_alpha_axis();
    test_sector_boundaries();
    test_invalid_bus_voltage();

    puts("All svpwm tests passed.");
    return 0;
}
