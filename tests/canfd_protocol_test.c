#include "../Core/Inc/canfd_protocol.h"

#include <assert.h>
#include <math.h>
#include <stdio.h>
#include <string.h>

static void write_float(uint8_t *destination, float value)
{
    memcpy(destination, &value, sizeof(value));
}

static float read_float(const uint8_t *source)
{
    float value;

    memcpy(&value, source, sizeof(value));
    return value;
}

static CanFdFrame make_control_frame(uint32_t id,
                                     float position,
                                     float velocity,
                                     float kp,
                                     float kd,
                                     float torque,
                                     uint8_t mode)
{
    CanFdFrame frame = {0};

    frame.identifier = id;
    frame.is_fd = 1U;
    frame.length = CANFD_CONTROL_DATA_LENGTH;
    write_float(&frame.data[0], position);
    write_float(&frame.data[4], velocity);
    write_float(&frame.data[8], kp);
    write_float(&frame.data[12], kd);
    write_float(&frame.data[16], torque);
    frame.data[20] = mode;
    return frame;
}

static void test_control_decode_and_timeout(void)
{
    CanFdProtocol protocol;
    CanFdFrame frame;
    const CanFdControlTarget *target;

    canfd_protocol_init(&protocol, 1U, 20U);
    frame = make_control_frame(protocol.control_id,
                               0.5f,
                               -1.0f,
                               5.0f,
                               0.2f,
                               0.1f,
                               CANFD_CONTROL_MODE_IMPEDANCE);

    assert(canfd_protocol_handle_frame(&protocol, &frame, 100U) == 1U);
    assert(canfd_protocol_is_control_alive(&protocol) == 1U);
    target = canfd_protocol_get_target(&protocol);
    assert(target != 0);
    assert(fabsf(target->position - 0.5f) < 0.0001f);
    assert(fabsf(target->velocity + 1.0f) < 0.0001f);
    assert(target->mode == CANFD_CONTROL_MODE_IMPEDANCE);

    canfd_protocol_update(&protocol, 121U);
    assert(canfd_protocol_is_control_alive(&protocol) == 0U);
    assert((protocol.fault_code & CANFD_FAULT_TIMEOUT) != 0U);
}

static void test_rejects_invalid_range(void)
{
    CanFdProtocol protocol;
    CanFdFrame frame;

    canfd_protocol_init(&protocol, 2U, 20U);
    frame = make_control_frame(protocol.control_id,
                               10.0f,
                               0.0f,
                               1.0f,
                               0.1f,
                               0.0f,
                               CANFD_CONTROL_MODE_POSITION);

    assert(canfd_protocol_handle_frame(&protocol, &frame, 0U) == 0U);
    assert((protocol.fault_code & CANFD_FAULT_RANGE) != 0U);
}

static void test_status_pack(void)
{
    CanFdProtocol protocol;
    CanFdStatus status =
    {
        .position = 1.0f,
        .velocity = 2.0f,
        .torque = 3.0f,
        .current = 4.0f,
        .bus_voltage = 24.0f,
        .temperature = 35.0f,
        .fault_code = 0U
    };
    CanFdFrame frame;

    canfd_protocol_init(&protocol, 3U, 20U);
    assert(canfd_protocol_pack_status(&protocol, &status, &frame) == 1U);
    assert(frame.identifier == protocol.status_id);
    assert(frame.length == CANFD_STATUS_DATA_LENGTH);
    assert(frame.is_fd == 1U);
    assert(fabsf(read_float(&frame.data[16]) - 24.0f) < 0.0001f);
}

int main(void)
{
    test_control_decode_and_timeout();
    test_rejects_invalid_range();
    test_status_pack();
    puts("All CAN FD protocol tests passed.");
    return 0;
}
