#ifndef DUAL_ENCODER_H
#define DUAL_ENCODER_H

#include <stdint.h>

typedef struct
{
    uint8_t motor_valid;
    uint8_t joint_valid;
    uint8_t consistency_valid;
    uint8_t fault;
    float motor_mechanical_angle;
    float motor_speed;
    float motor_electrical_angle;
    float joint_position;
    float joint_speed;
    float estimated_joint_position;
    float position_error;
    float speed_error;
    float twist_angle;
} DualEncoderState;

typedef struct
{
    uint32_t pole_pairs;
    float electrical_offset;
    float gear_ratio;
    float motor_velocity_filter_alpha;
    float joint_velocity_filter_alpha;
    float position_error_limit;
    float speed_error_limit;
} DualEncoderConfig;

typedef struct
{
    DualEncoderConfig config;
    DualEncoderState state;
    float last_motor_angle;
    float last_joint_angle;
    uint8_t initialized;
} DualEncoderController;

void dual_encoder_init(DualEncoderController *controller,
                       const DualEncoderConfig *config,
                       float motor_angle,
                       float joint_angle);

void dual_encoder_reset(DualEncoderController *controller,
                        float motor_angle,
                        float joint_angle);

uint8_t dual_encoder_update(DualEncoderController *controller,
                            float motor_angle,
                            float joint_angle,
                            float dt_seconds);

const DualEncoderState *dual_encoder_get_state(
    const DualEncoderController *controller);

float dual_encoder_wrap_angle(float angle);

float dual_encoder_unwrap_delta(float delta);

#endif
