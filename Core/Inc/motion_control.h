#ifndef MOTION_CONTROL_H
#define MOTION_CONTROL_H

#include <stdint.h>

typedef struct
{
    float position;
    float velocity;
} MotionFeedback;

typedef struct
{
    float velocity_filter_alpha;
    float velocity_kp;
    float velocity_ki;
    float velocity_output_min;
    float velocity_output_max;
    float position_kp;
    float position_min;
    float position_max;
    float velocity_limit;
    float torque_feedforward_gain;
    float velocity_feedforward_gain;
} MotionControlConfig;

typedef struct
{
    MotionControlConfig config;
    float filtered_velocity;
    float velocity_integral;
    float last_position;
    float last_position_error;
    float velocity_reference;
    float torque_reference;
    float position_reference;
    uint8_t initialized;
} MotionController;

void motion_controller_init(MotionController *controller,
                            const MotionControlConfig *config,
                            float initial_position);

void motion_controller_reset(MotionController *controller,
                             float current_position);

void motion_controller_set_feedback(MotionController *controller,
                                    float position,
                                    float velocity);

MotionFeedback motion_controller_update_feedback(
    MotionController *controller,
    float position,
    float dt_seconds);

float motion_controller_update_position_loop(
    MotionController *controller,
    float position_reference,
    float velocity_feedforward,
    float dt_seconds);

float motion_controller_update_speed_loop(
    MotionController *controller,
    float velocity_reference,
    float torque_feedforward,
    float dt_seconds);

float motion_controller_get_position_reference(
    const MotionController *controller);

float motion_controller_get_velocity_reference(
    const MotionController *controller);

float motion_controller_get_torque_reference(
    const MotionController *controller);

#endif
