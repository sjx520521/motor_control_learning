#ifndef CANFD_PROTOCOL_H
#define CANFD_PROTOCOL_H

#include <stdint.h>

#ifdef __cplusplus
extern "C" {
#endif

#define CANFD_MAX_DATA_BYTES      (64U)
#define CANFD_CONTROL_DATA_LENGTH (24U)
#define CANFD_LEGACY_CONTROL_DATA_LENGTH (20U)
#define CANFD_COMMAND_DATA_LENGTH (2U)
#define CANFD_STATUS_DATA_LENGTH  (32U)
#define CANFD_DEFAULT_CONTROL_TIMEOUT_MS (20U)

typedef enum
{
    CANFD_COMMAND_DISABLE = 0x00U,
    CANFD_COMMAND_ENABLE = 0x01U,
    CANFD_COMMAND_CLEAR_FAULT = 0x02U
} CanFdCommandCode;

typedef enum
{
    CANFD_FAULT_NONE = 0U,
    CANFD_FAULT_BAD_FRAME = (1UL << 0),
    CANFD_FAULT_RANGE = (1UL << 1),
    CANFD_FAULT_TIMEOUT = (1UL << 2)
} CanFdProtocolFault;

typedef enum
{
    CANFD_CONTROL_MODE_POSITION = 0U,
    CANFD_CONTROL_MODE_VELOCITY = 1U,
    CANFD_CONTROL_MODE_TORQUE = 2U,
    CANFD_CONTROL_MODE_IMPEDANCE = 3U
} CanFdControlMode;

typedef struct
{
    uint32_t identifier;
    uint8_t is_extended;
    uint8_t is_fd;
    uint8_t bitrate_switch;
    uint8_t length;
    uint8_t data[CANFD_MAX_DATA_BYTES];
} CanFdFrame;

typedef struct
{
    float position;
    float velocity;
    float kp;
    float kd;
    float torque_feedforward;
    uint8_t mode;
    uint8_t enabled;
    uint8_t clear_fault_requested;
} CanFdControlTarget;

typedef struct
{
    float position;
    float velocity;
    float torque;
    float current;
    float bus_voltage;
    float temperature;
    uint32_t fault_code;
} CanFdStatus;

typedef struct
{
    float position_min;
    float position_max;
    float velocity_max;
    float kp_max;
    float kd_max;
    float torque_max;
} CanFdProtocolLimits;

typedef struct
{
    uint8_t node_id;
    uint32_t control_id;
    uint32_t command_id;
    uint32_t status_id;
    uint32_t fault_code;
    uint32_t last_control_time_ms;
    uint32_t control_timeout_ms;
    CanFdProtocolLimits limits;
    CanFdControlTarget target;
} CanFdProtocol;

void canfd_protocol_init(CanFdProtocol *protocol,
                         uint8_t node_id,
                         uint32_t control_timeout_ms);

void canfd_protocol_set_limits(CanFdProtocol *protocol,
                               const CanFdProtocolLimits *limits);

uint8_t canfd_protocol_handle_frame(CanFdProtocol *protocol,
                                    const CanFdFrame *frame,
                                    uint32_t now_ms);

void canfd_protocol_update(CanFdProtocol *protocol,
                           uint32_t now_ms);

uint8_t canfd_protocol_is_control_alive(
    const CanFdProtocol *protocol);

uint8_t canfd_protocol_pack_status(const CanFdProtocol *protocol,
                                   const CanFdStatus *status,
                                   CanFdFrame *frame);

uint8_t canfd_protocol_pack_command(const CanFdProtocol *protocol,
                                    CanFdCommandCode command,
                                    uint8_t sequence,
                                    CanFdFrame *frame);

const CanFdControlTarget *canfd_protocol_get_target(
    const CanFdProtocol *protocol);

#ifdef __cplusplus
}
#endif

#endif /* CANFD_PROTOCOL_H */
