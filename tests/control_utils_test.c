#include <assert.h>
#include <math.h>
#include <stdio.h>

#include "../Core/Inc/control_utils.h"

static void test_clamp(void)
{
    assert(mc_clamp_f32(2.0f, -1.0f, 1.0f) == 1.0f);
    assert(mc_clamp_f32(-2.0f, -1.0f, 1.0f) == -1.0f);
    assert(mc_clamp_f32(0.5f, -1.0f, 1.0f) == 0.5f);
}

static void test_pi_step(void)
{
    PI_Controller pi;

    pi_controller_init(&pi, 2.0f, 5.0f, -3.0f, 3.0f);

    float output = pi_controller_update(&pi, 0.4f, 0.01f);

    /* P = 2 * 0.4 = 0.8
       I = 0 + 5 * 0.4 * 0.01 = 0.02
       output = 0.82 */
    assert(fabsf(output - 0.82f) < 0.0001f);
}

static void test_output_limit(void)
{
    PI_Controller pi;

    pi_controller_init(&pi, 10.0f, 0.0f, -1.0f, 1.0f);

    float output = pi_controller_update(&pi, 2.0f, 0.01f);

    assert(output == 1.0f);
}

static void test_back_calculation_anti_windup(void)
{
    PI_Controller pi;

    pi_controller_init(&pi, 1.0f, 100.0f, -10.0f, 10.0f);
    pi_controller_set_anti_windup(&pi, 50.0f);

    float requested = pi_controller_update(&pi, 5.0f, 0.01f);
    assert(fabsf(requested - 5.05f) < 0.0001f);

    pi_controller_apply_output_feedback(&pi, 2.0f, 0.01f);

    /* The feedback must pull the integral down after an external limit. */
    assert(pi.integral < 5.0f);
    assert(pi.last_output == 2.0f);
}

int main(void)
{
    test_clamp();
    test_pi_step();
    test_output_limit();
    test_back_calculation_anti_windup();

    puts("All control_utils tests passed.");
    return 0;
}
