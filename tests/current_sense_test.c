#include <assert.h>
#include <math.h>
#include <stdio.h>

#include "../Core/Inc/current_sense.h"

#define TEST_EPSILON (0.0001f)
static void assert_near(float actual, float expected)
{
    assert(fabsf(actual - expected) < TEST_EPSILON);
}

static void test_zero_current_and_phase_reconstruction(void)
{
    CurrentSenseConfig config =
    {
        .offset_raw_a = 2048U,
        .offset_raw_b = 2048U,
        .amp_per_count_a = 0.01f,
        .amp_per_count_b = 0.01f
    };

    current_sense_init(&config);

    PhaseCurrent current =
        current_sense_get_phase_current(2048U, 2048U);

    assert_near(current.phase_a, 0.0f);
    assert_near(current.phase_b, 0.0f);
    assert_near(current.phase_c, 0.0f);
}

static void test_current_conversion(void)
{
    CurrentSenseConfig config =
    {
        .offset_raw_a = 2048U,
        .offset_raw_b = 2048U,
        .amp_per_count_a = 0.01f,
        .amp_per_count_b = 0.01f
    };

    current_sense_init(&config);

    PhaseCurrent current =
        current_sense_get_phase_current(2148U, 1948U);

    assert_near(current.phase_a, 1.0f);
    assert_near(current.phase_b, -1.0f);
    assert_near(current.phase_c, 0.0f);
}

static void test_invalid_sensitivity_is_safe(void)
{
    CurrentSenseConfig config =
    {
        .offset_raw_a = 2048U,
        .offset_raw_b = 2048U,
        .amp_per_count_a = 0.0f,
        .amp_per_count_b = 0.01f
    };

    current_sense_init(&config);

    assert(current_sense_raw_to_amp(4095U, 2048U, 0.0f) == 0.0f);
}

int main(void)
{
    test_zero_current_and_phase_reconstruction();
    test_current_conversion();
    test_invalid_sensitivity_is_safe();

    puts("All current_sense tests passed.");
    return 0;
}
