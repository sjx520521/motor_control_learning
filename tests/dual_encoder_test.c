#include <assert.h>
#include <math.h>
#include <stdio.h>

#include "../Core/Inc/dual_encoder.h"

#define PI_F (3.14159265359f)

static void assert_near(float actual, float expected)
{
    assert(fabsf(actual - expected) < 0.001f);
}

static DualEncoderConfig test_config(void)
{
    DualEncoderConfig config =
    {
        .pole_pairs = 4U,
        .electrical_offset = 0.1f,
        .gear_ratio = 50.0f,
        .motor_velocity_filter_alpha = 1.0f,
        .joint_velocity_filter_alpha = 1.0f,
        .position_error_limit = 0.2f,
        .speed_error_limit = 1.0f
    };

    return config;
}

static void test_angle_conversion_and_speed(void)
{
    DualEncoderController controller;
    const DualEncoderConfig config = test_config();

    dual_encoder_init(&controller, &config, 1.0f, 0.02f);
    assert_near(controller.state.motor_electrical_angle, 4.1f);
    assert_near(controller.state.estimated_joint_position, 0.02f);

    assert(dual_encoder_update(&controller,
                               1.1f,
                               0.022f,
                               0.1f) != 0U);
    assert_near(controller.state.motor_speed, 1.0f);
    assert_near(controller.state.joint_speed, 0.02f);
    assert_near(controller.state.motor_electrical_angle, 4.5f);
}

static void test_wraparound_delta(void)
{
    DualEncoderController controller;
    const DualEncoderConfig config = test_config();

    dual_encoder_init(&controller, 2.0f * PI_F - 0.01f, 0.0f);
    assert(dual_encoder_update(&controller,
                               0.01f,
                               0.0f,
                               0.02f) != 0U);
    assert_near(controller.state.motor_speed, 1.0f);
}

static void test_consistency_fault(void)
{
    DualEncoderController controller;
    const DualEncoderConfig config = test_config();

    dual_encoder_init(&controller, 0.0f, 0.0f);
    assert(dual_encoder_update(&controller,
                               10.0f,
                               0.0f,
                               0.1f) == 0U);
    assert(controller.state.consistency_valid == 0U);
    assert(controller.state.fault != 0U);
}

int main(void)
{
    test_angle_conversion_and_speed();
    test_wraparound_delta();
    test_consistency_fault();

    puts("All dual-encoder tests passed.");
    return 0;
}
