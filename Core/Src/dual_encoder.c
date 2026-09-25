#include "dual_encoder.h"

#include <math.h>

#define DUAL_ENCODER_PI (3.14159265359f)
#define DUAL_ENCODER_TWO_PI (6.28318530718f)

static float dual_encoder_clamp(float value,
                                float minimum,
                                float maximum)
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

float dual_encoder_wrap_angle(float angle)
{
    while (angle >= DUAL_ENCODER_TWO_PI)
    {
        angle -= DUAL_ENCODER_TWO_PI;
    }

    while (angle < 0.0f)
    {
        angle += DUAL_ENCODER_TWO_PI;
    }

    return angle;
}

float dual_encoder_unwrap_delta(float delta)
{
    while (delta > DUAL_ENCODER_PI)
    {
        delta -= DUAL_ENCODER_TWO_PI;
    }

    while (delta < -DUAL_ENCODER_PI)
    {
        delta += DUAL_ENCODER_TWO_PI;
    }

    return delta;
}

static void dual_encoder_clear_state(DualEncoderController *controller,
                                     float motor_angle,
                                     float joint_angle)
{
    controller->state.motor_valid = 1U;
    controller->state.joint_valid = 1U;
    controller->state.consistency_valid = 1U;
    controller->state.fault = 0U;
    controller->state.motor_mechanical_angle = motor_angle;
    controller->state.motor_speed = 0.0f;
    controller->state.motor_electrical_angle =
        dual_encoder_wrap_angle(
            controller->config.pole_pairs * motor_angle +
            controller->config.electrical_offset);
    controller->state.joint_position = joint_angle;
    controller->state.joint_speed = 0.0f;
    controller->state.estimated_joint_position =
        motor_angle / controller->config.gear_ratio;
    controller->state.position_error =
        controller->state.estimated_joint_position - joint_angle;
    controller->state.speed_error = 0.0f;
    controller->state.twist_angle =
        controller->state.position_error;
    controller->last_motor_angle = motor_angle;
    controller->last_joint_angle = joint_angle;
}

void dual_encoder_init(DualEncoderController *controller,
                       const DualEncoderConfig *config,
                       float motor_angle,
                       float joint_angle)
{
    if (controller == 0 || config == 0)
    {
        return;
    }

    controller->config = *config;

    if (controller->config.gear_ratio <= 0.0f)
    {
        controller->config.gear_ratio = 1.0f;
    }

    dual_encoder_clear_state(controller,
                             motor_angle,
                             joint_angle);
    controller->initialized = 1U;
}

void dual_encoder_reset(DualEncoderController *controller,
                        float motor_angle,
                        float joint_angle)
{
    if (controller == 0 || controller->initialized == 0U)
    {
        return;
    }

    dual_encoder_clear_state(controller,
                             motor_angle,
                             joint_angle);
}

uint8_t dual_encoder_update(DualEncoderController *controller,
                            float motor_angle,
                            float joint_angle,
                            float dt_seconds)
{
    if (controller == 0 ||
        controller->initialized == 0U ||
        dt_seconds <= 0.0f)
    {
        return 0U;
    }

    const float motor_delta =
        dual_encoder_unwrap_delta(
            motor_angle - controller->last_motor_angle);
    const float joint_delta =
        dual_encoder_unwrap_delta(
            joint_angle - controller->last_joint_angle);
    const float raw_motor_speed = motor_delta / dt_seconds;
    const float raw_joint_speed = joint_delta / dt_seconds;
    float motor_alpha =
        dual_encoder_clamp(
            controller->config.motor_velocity_filter_alpha,
            0.0f,
            1.0f);
    float joint_alpha =
        dual_encoder_clamp(
            controller->config.joint_velocity_filter_alpha,
            0.0f,
            1.0f);

    controller->state.motor_mechanical_angle += motor_delta;
    controller->state.joint_position += joint_delta;
    controller->state.motor_speed +=
        motor_alpha * (raw_motor_speed -
                       controller->state.motor_speed);
    controller->state.joint_speed +=
        joint_alpha * (raw_joint_speed -
                       controller->state.joint_speed);
    controller->state.motor_electrical_angle =
        dual_encoder_wrap_angle(
            controller->config.pole_pairs *
            controller->state.motor_mechanical_angle +
            controller->config.electrical_offset);
    controller->state.estimated_joint_position =
        controller->state.motor_mechanical_angle /
        controller->config.gear_ratio;
    controller->state.position_error =
        controller->state.estimated_joint_position -
        controller->state.joint_position;
    controller->state.speed_error =
        controller->state.motor_speed /
        controller->config.gear_ratio -
        controller->state.joint_speed;
    controller->state.twist_angle =
        controller->state.position_error;

    controller->state.consistency_valid =
        (fabsf(controller->state.position_error) <=
         controller->config.position_error_limit) &&
        (fabsf(controller->state.speed_error) <=
         controller->config.speed_error_limit);
    controller->state.fault =
        controller->state.consistency_valid ? 0U : 1U;

    controller->last_motor_angle = motor_angle;
    controller->last_joint_angle = joint_angle;

    return controller->state.consistency_valid;
}

const DualEncoderState *dual_encoder_get_state(
    const DualEncoderController *controller)
{
    if (controller == 0)
    {
        return 0;
    }

    return &controller->state;
}
