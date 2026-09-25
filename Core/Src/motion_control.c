#include "motion_control.h"

static float motion_clamp(float value, float minimum, float maximum)
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

void motion_controller_init(MotionController *controller,
                            const MotionControlConfig *config,
                            float initial_position)
{
    if (controller == 0 || config == 0)
    {
        return;
    }

    controller->config = *config;
    controller->filtered_velocity = 0.0f;
    controller->velocity_integral = 0.0f;
    controller->last_position = initial_position;
    controller->last_position_error = 0.0f;
    controller->velocity_reference = 0.0f;
    controller->torque_reference = 0.0f;
    controller->position_reference = initial_position;
    controller->initialized = 1U;
}

void motion_controller_reset(MotionController *controller,
                             float current_position)
{
    if (controller == 0)
    {
        return;
    }

    controller->filtered_velocity = 0.0f;
    controller->velocity_integral = 0.0f;
    controller->last_position = current_position;
    controller->last_position_error = 0.0f;
    controller->velocity_reference = 0.0f;
    controller->torque_reference = 0.0f;
    controller->position_reference = current_position;
}

void motion_controller_set_feedback(MotionController *controller,
                                    float position,
                                    float velocity)
{
    if (controller == 0 || controller->initialized == 0U)
    {
        return;
    }

    controller->last_position = position;
    controller->filtered_velocity = velocity;
}

MotionFeedback motion_controller_update_feedback(
    MotionController *controller,
    float position,
    float dt_seconds)
{
    MotionFeedback feedback = {0.0f, 0.0f};

    if (controller == 0 || controller->initialized == 0U)
    {
        return feedback;
    }

    if (dt_seconds <= 0.0f)
    {
        feedback.position = position;
        feedback.velocity = controller->filtered_velocity;
        return feedback;
    }

    const float raw_velocity =
        (position - controller->last_position) / dt_seconds;
    float alpha = controller->config.velocity_filter_alpha;

    alpha = motion_clamp(alpha, 0.0f, 1.0f);
    controller->filtered_velocity +=
        alpha * (raw_velocity - controller->filtered_velocity);
    controller->last_position = position;

    feedback.position = position;
    feedback.velocity = controller->filtered_velocity;
    return feedback;
}

float motion_controller_update_position_loop(
    MotionController *controller,
    float position_reference,
    float velocity_feedforward,
    float dt_seconds)
{
    if (controller == 0 || controller->initialized == 0U)
    {
        return 0.0f;
    }

    controller->position_reference =
        motion_clamp(position_reference,
                     controller->config.position_min,
                     controller->config.position_max);

    const float position_error =
        controller->position_reference - controller->last_position;
    float velocity_reference =
        controller->config.position_kp * position_error +
        controller->config.velocity_feedforward_gain *
        velocity_feedforward;

    controller->last_position_error = position_error;
    controller->velocity_reference =
        motion_clamp(velocity_reference,
                     -controller->config.velocity_limit,
                     controller->config.velocity_limit);

    (void)dt_seconds;
    return controller->velocity_reference;
}

float motion_controller_update_speed_loop(
    MotionController *controller,
    float velocity_reference,
    float torque_feedforward,
    float dt_seconds)
{
    if (controller == 0 || controller->initialized == 0U)
    {
        return 0.0f;
    }

    controller->velocity_reference =
        motion_clamp(velocity_reference,
                     -controller->config.velocity_limit,
                     controller->config.velocity_limit);

    const float error =
        controller->velocity_reference -
        controller->filtered_velocity;

    controller->velocity_integral +=
        controller->config.velocity_ki * error * dt_seconds;
    controller->velocity_integral =
        motion_clamp(controller->velocity_integral,
                     controller->config.velocity_output_min,
                     controller->config.velocity_output_max);

    const float feedback_torque =
        controller->config.velocity_kp * error +
        controller->velocity_integral;

    controller->torque_reference =
        motion_clamp(feedback_torque +
                     controller->config.torque_feedforward_gain *
                     torque_feedforward,
                     controller->config.velocity_output_min,
                     controller->config.velocity_output_max);

    return controller->torque_reference;
}

float motion_controller_get_position_reference(
    const MotionController *controller)
{
    return (controller == 0) ? 0.0f : controller->position_reference;
}

float motion_controller_get_velocity_reference(
    const MotionController *controller)
{
    return (controller == 0) ? 0.0f : controller->velocity_reference;
}

float motion_controller_get_torque_reference(
    const MotionController *controller)
{
    return (controller == 0) ? 0.0f : controller->torque_reference;
}
