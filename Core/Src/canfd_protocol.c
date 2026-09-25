#include "canfd_protocol.h"

#include <math.h>
#include <string.h>

#define CANFD_CONTROL_ID_BASE (0x100UL)
#define CANFD_COMMAND_ID_BASE (0x110UL)
#define CANFD_STATUS_ID_BASE  (0x200UL)

static const CanFdProtocolLimits canfd_default_limits =
{
    .position_min = -3.14159265359f,
    .position_max =  3.14159265359f,
    .velocity_max = 20.0f,
    .kp_max = 100.0f,
    .kd_max = 10.0f,
    .torque_max = 8.0f
};

static void canfd_write_float(uint8_t *destination, float value)
{
    memcpy(destination, &value, sizeof(value));
}

static float canfd_read_float(const uint8_t *source)
{
    float value;

    memcpy(&value, source, sizeof(value));
    return value;
}

static uint8_t canfd_target_is_valid(
    const CanFdProtocol *protocol,
    const CanFdControlTarget *target)
{
    const CanFdProtocolLimits *limits = &protocol->limits;

    if (target->mode > CANFD_CONTROL_MODE_IMPEDANCE ||
        !isfinite(target->position) ||
        !isfinite(target->velocity) ||
        !isfinite(target->kp) ||
        !isfinite(target->kd) ||
        !isfinite(target->torque_feedforward))
    {
        return 0U;
    }

    if (target->mode == CANFD_CONTROL_MODE_POSITION &&
        (target->position < limits->position_min ||
         target->position > limits->position_max))
    {
        return 0U;
    }

    if (target->mode == CANFD_CONTROL_MODE_IMPEDANCE &&
        (target->position < limits->position_min ||
         target->position > limits->position_max))
    {
        return 0U;
    }

    if (fabsf(target->velocity) > limits->velocity_max ||
        target->kp < 0.0f ||
        target->kp > limits->kp_max ||
        target->kd < 0.0f ||
        target->kd > limits->kd_max ||
        fabsf(target->torque_feedforward) > limits->torque_max)
    {
        return 0U;
    }

    return 1U;
}

void canfd_protocol_init(CanFdProtocol *protocol,
                         uint8_t node_id,
                         uint32_t control_timeout_ms)
{
    if (protocol == 0)
    {
        return;
    }

    memset(protocol, 0, sizeof(*protocol));
    protocol->node_id = node_id;
    protocol->control_id = CANFD_CONTROL_ID_BASE + node_id;
    protocol->command_id = CANFD_COMMAND_ID_BASE + node_id;
    protocol->status_id = CANFD_STATUS_ID_BASE + node_id;
    protocol->control_timeout_ms =
        (control_timeout_ms == 0U)
            ? CANFD_DEFAULT_CONTROL_TIMEOUT_MS
            : control_timeout_ms;
    protocol->limits = canfd_default_limits;
}

void canfd_protocol_set_limits(CanFdProtocol *protocol,
                               const CanFdProtocolLimits *limits)
{
    if (protocol != 0 && limits != 0 &&
        limits->position_min < limits->position_max &&
        limits->velocity_max >= 0.0f &&
        limits->kp_max >= 0.0f &&
        limits->kd_max >= 0.0f &&
        limits->torque_max >= 0.0f)
    {
        protocol->limits = *limits;
    }
}

uint8_t canfd_protocol_handle_frame(CanFdProtocol *protocol,
                                    const CanFdFrame *frame,
                                    uint32_t now_ms)
{
    if (protocol == 0 || frame == 0 ||
        frame->is_extended != 0U ||
        frame->length > CANFD_MAX_DATA_BYTES)
    {
        return 0U;
    }

    if (frame->identifier == protocol->control_id &&
        (frame->length == CANFD_CONTROL_DATA_LENGTH ||
         frame->length == CANFD_LEGACY_CONTROL_DATA_LENGTH))
    {
        CanFdControlTarget target;

        target.position = canfd_read_float(&frame->data[0]);
        target.velocity = canfd_read_float(&frame->data[4]);
        target.kp = canfd_read_float(&frame->data[8]);
        target.kd = canfd_read_float(&frame->data[12]);
        target.torque_feedforward = canfd_read_float(&frame->data[16]);
        target.mode = (frame->length >= CANFD_CONTROL_DATA_LENGTH)
                          ? frame->data[20]
                          : CANFD_CONTROL_MODE_POSITION;
        target.enabled = 1U;
        target.clear_fault_requested = 0U;

        if (canfd_target_is_valid(protocol, &target) == 0U)
        {
            protocol->fault_code |= CANFD_FAULT_RANGE;
            return 0U;
        }

        protocol->target = target;
        protocol->last_control_time_ms = now_ms;
        protocol->fault_code &= ~CANFD_FAULT_TIMEOUT;
        return 1U;
    }

    if (frame->identifier == protocol->command_id &&
        frame->length == CANFD_COMMAND_DATA_LENGTH)
    {
        const CanFdCommandCode command =
            (CanFdCommandCode)frame->data[0];

        if (command == CANFD_COMMAND_ENABLE)
        {
            protocol->target.enabled = 1U;
        }
        else if (command == CANFD_COMMAND_DISABLE)
        {
            protocol->target.enabled = 0U;
        }
        else if (command == CANFD_COMMAND_CLEAR_FAULT)
        {
            protocol->target.clear_fault_requested = 1U;
            protocol->fault_code = CANFD_FAULT_NONE;
        }
        else
        {
            protocol->fault_code |= CANFD_FAULT_BAD_FRAME;
            return 0U;
        }

        return 1U;
    }

    return 0U;
}

void canfd_protocol_update(CanFdProtocol *protocol,
                           uint32_t now_ms)
{
    if (protocol == 0 || protocol->target.enabled == 0U)
    {
        return;
    }

    if ((uint32_t)(now_ms - protocol->last_control_time_ms) >
        protocol->control_timeout_ms)
    {
        protocol->target.enabled = 0U;
        protocol->fault_code |= CANFD_FAULT_TIMEOUT;
    }
}

uint8_t canfd_protocol_is_control_alive(
    const CanFdProtocol *protocol)
{
    if (protocol == 0)
    {
        return 0U;
    }

    return (protocol->target.enabled != 0U &&
            (protocol->fault_code & CANFD_FAULT_TIMEOUT) == 0U)
               ? 1U
               : 0U;
}

uint8_t canfd_protocol_pack_status(const CanFdProtocol *protocol,
                                   const CanFdStatus *status,
                                   CanFdFrame *frame)
{
    if (protocol == 0 || status == 0 || frame == 0)
    {
        return 0U;
    }

    memset(frame, 0, sizeof(*frame));
    frame->identifier = protocol->status_id;
    frame->is_fd = 1U;
    frame->bitrate_switch = 1U;
    frame->length = CANFD_STATUS_DATA_LENGTH;
    canfd_write_float(&frame->data[0], status->position);
    canfd_write_float(&frame->data[4], status->velocity);
    canfd_write_float(&frame->data[8], status->torque);
    canfd_write_float(&frame->data[12], status->current);
    canfd_write_float(&frame->data[16], status->bus_voltage);
    canfd_write_float(&frame->data[20], status->temperature);
    memcpy(&frame->data[24],
           &status->fault_code,
           sizeof(status->fault_code));
    return 1U;
}

uint8_t canfd_protocol_pack_command(const CanFdProtocol *protocol,
                                    CanFdCommandCode command,
                                    uint8_t sequence,
                                    CanFdFrame *frame)
{
    if (protocol == 0 || frame == 0 ||
        (command != CANFD_COMMAND_ENABLE &&
         command != CANFD_COMMAND_DISABLE &&
         command != CANFD_COMMAND_CLEAR_FAULT))
    {
        return 0U;
    }

    memset(frame, 0, sizeof(*frame));
    frame->identifier = protocol->command_id;
    frame->is_fd = 1U;
    frame->bitrate_switch = 1U;
    frame->length = CANFD_COMMAND_DATA_LENGTH;
    frame->data[0] = (uint8_t)command;
    frame->data[1] = sequence;
    return 1U;
}

const CanFdControlTarget *canfd_protocol_get_target(
    const CanFdProtocol *protocol)
{
    return (protocol == 0) ? 0 : &protocol->target;
}
