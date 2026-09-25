#include <assert.h>
#include <math.h>
#include <stdio.h>

#include "../Core/Inc/motion_control.h"

static void assert_near(float actual, float expected)
{
    assert(fabsf(actual - expected) < 0.001f);
}

static MotionControlConfig test_config(void)
{
    MotionControlConfig config =
    {
        .velocity_filter_alpha = 1.0f,
        .velocity_kp = 2.0f,
        .velocity_ki = 1.0f,
        .velocity_output_min = -5.0f,
        .velocity_output_max = 5.0f,
        .position_kp = 4.0f,
        .position_min = -1.0f,
        .position_max = 1.0f,
        .velocity_limit = 3.0f,
        .torque_feedforward_gain = 1.0f,
        .velocity_feedforward_gain = 1.0f
    };

    return config;
}

static void test_velocity_calculation_and_filter(void)
{
    MotionController controller;
    const MotionControlConfig config = test_config();

    motion_controller_init(&controller, &config, 0.0f);
    MotionFeedback feedback =
        motion_controller_update_feedback(&controller, 0.1f, 0.01f);

    assert_near(feedback.velocity, 10.0f);

    feedback =
        motion_controller_update_feedback(&controller, 0.2f, 0.01f);
    assert_near(feedback.velocity, 10.0f);
}

static void test_position_limit_and_speed_limit(void)
{
    MotionController controller;
    const MotionControlConfig config = test_config();

    motion_controller_init(&controller, &config, 0.0f);

    const float velocity_reference =
        motion_controller_update_position_loop(&controller,
                                               5.0f,
                                               0.0f,
                                               0.01f);
    assert_near(velocity_reference, 3.0f);
    assert_near(motion_controller_get_position_reference(&controller), 1.0f);
}

static void test_speed_pi_and_feedforward(void)
{
    MotionController controller;
    const MotionControlConfig config = test_config();

    motion_controller_init(&controller, &config, 0.0f);
    motion_controller_update_feedback(&controller, 0.0f, 0.01f);

    const float torque =
        motion_controller_update_speed_loop(&controller,
                                            1.0f,
                                            0.5f,
                                            0.1f);
    assert(torque > 2.0f);
    assert(torque <= 5.0f);
}

static void test_reset_clears_outer_loop_state(void)
{
    MotionController controller;
    const MotionControlConfig config = test_config();

    motion_controller_init(&controller, &config, 0.0f);
    motion_controller_update_feedback(&controller, 0.1f, 0.01f);
    motion_controller_update_speed_loop(&controller, 1.0f, 0.0f, 0.1f);

    motion_controller_reset(&controller, 0.25f);

    assert_near(controller.filtered_velocity, 0.0f);
    assert_near(controller.velocity_integral, 0.0f);
    assert_near(controller.last_position, 0.25f);
    assert_near(controller.velocity_reference, 0.0f);
    assert_near(controller.torque_reference, 0.0f);
}

static void test_impedance_mode_generates_direct_torque(void)
{
    MotionController controller;
    const MotionControlConfig config = test_config();

    motion_controller_init(&controller, &config, 0.0f);
    motion_controller_set_feedback(&controller, 0.1f, 0.2f);

    const float torque =
        motion_controller_update_mode(
            &controller,
            MOTION_CONTROL_MODE_IMPEDANCE,
            0.5f,
            0.0f,
            2.0f,
            1.0f,
            0.3f,
            0.001f);

    assert_near(torque, 1.1f);
}

static void test_torque_mode_bypasses_velocity_integrator(void)
{
    MotionController controller;
    const MotionControlConfig config = test_config();

    motion_controller_init(&controller, &config, 0.0f);
    motion_controller_set_feedback(&controller, 0.0f, 0.0f);

    const float torque =
        motion_controller_update_mode(
            &controller,
            MOTION_CONTROL_MODE_TORQUE,
            0.0f,
            0.0f,
            0.0f,
            0.0f,
            2.5f,
            0.001f);

    assert_near(torque, 2.5f);
    assert_near(controller.velocity_integral, 0.0f);
}

int main(void)
{
    test_velocity_calculation_and_filter();
    test_position_limit_and_speed_limit();
    test_speed_pi_and_feedforward();
    test_reset_clears_outer_loop_state();
    test_impedance_mode_generates_direct_torque();
    test_torque_mode_bypasses_velocity_integrator();

    puts("All motion-control tests passed.");
    return 0;
}
