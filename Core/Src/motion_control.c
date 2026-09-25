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
    controller->mode = MOTION_CONTROL_MODE_POSITION;
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

void motion_controller_set_impedance_gains(MotionController *controller,
                                           float position_stiffness,
                                           float velocity_damping)
{
    if (controller == 0 || controller->initialized == 0U)
    {
        return;
    }

    if (position_stiffness < 0.0f)
    {
        position_stiffness = 0.0f;
    }
    if (velocity_damping < 0.0f)
    {
        velocity_damping = 0.0f;
    }

    controller->config.position_kp = position_stiffness;
    controller->config.velocity_kp = velocity_damping;
    /*
     * MIT-style impedance control has proportional position and velocity
     * terms plus feedforward torque; it does not use a velocity integrator.
     */
    controller->config.velocity_ki = 0.0f;
}

void motion_controller_set_mode(MotionController *controller,
                                MotionControlMode mode)
{
    if (controller == 0 || controller->initialized == 0U)
    {
        return;
    }

    if (mode > MOTION_CONTROL_MODE_IMPEDANCE)
    {
        mode = MOTION_CONTROL_MODE_POSITION;
    }

    controller->mode = mode;
    controller->velocity_integral = 0.0f;
}

float motion_controller_update_mode(
    MotionController *controller,
    MotionControlMode mode,
    float position_reference,
    float velocity_reference,
    float position_stiffness,
    float velocity_damping,
    float torque_feedforward,
    float dt_seconds)
{
    return motion_controller_update_mode_scheduled(
        controller,
        mode,
        position_reference,
        velocity_reference,
        position_stiffness,
        velocity_damping,
        torque_feedforward,
        dt_seconds,
        1U);
}

float motion_controller_update_mode_scheduled(
    MotionController *controller,
    MotionControlMode mode,
    float position_reference,
    float velocity_reference,
    float position_stiffness,
    float velocity_damping,
    float torque_feedforward,
    float dt_seconds,
    uint8_t update_position)
{
    float torque_reference;

    if (controller == 0 || controller->initialized == 0U)
    {
        return 0.0f;
    }

    if (controller->mode != mode)
    {
        motion_controller_set_mode(controller, mode);
    }

    switch (controller->mode)
    {
    case MOTION_CONTROL_MODE_POSITION:
        controller->config.position_kp = position_stiffness;
        if (update_position != 0U)
        {
            motion_controller_update_position_loop(
                controller,
                position_reference,
                velocity_reference,
                dt_seconds);
        }
        return motion_controller_update_speed_loop(
            controller,
            motion_controller_get_velocity_reference(controller),
            torque_feedforward,
            dt_seconds);

    case MOTION_CONTROL_MODE_VELOCITY:
        return motion_controller_update_speed_loop(
            controller,
            velocity_reference,
            torque_feedforward,
            dt_seconds);

    case MOTION_CONTROL_MODE_TORQUE:
        controller->velocity_integral = 0.0f;
        torque_reference =
            motion_clamp(torque_feedforward,
                         controller->config.velocity_output_min,
                         controller->config.velocity_output_max);
        controller->torque_reference = torque_reference;
        return torque_reference;

    case MOTION_CONTROL_MODE_IMPEDANCE:
        controller->velocity_integral = 0.0f;
        controller->position_reference =
            motion_clamp(position_reference,
                         controller->config.position_min,
                         controller->config.position_max);
        torque_reference =
            torque_feedforward +
            position_stiffness *
                (controller->position_reference -
                 controller->last_position) +
            velocity_damping *
                (velocity_reference -
                 controller->filtered_velocity);
        controller->torque_reference =
            motion_clamp(torque_reference,
                         controller->config.velocity_output_min,
                         controller->config.velocity_output_max);
        return controller->torque_reference;

    default:
        return 0.0f;
    }
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
