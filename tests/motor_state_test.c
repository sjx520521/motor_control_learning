#include <assert.h>
#include <stdio.h>

#include "../Core/Inc/motor_state.h"

static void test_normal_start_stop_sequence(void)
{
    MotorController controller;

    motor_controller_init(&controller);
    assert(controller.state == MOTOR_STATE_DISABLED);
    assert(controller.fault == MOTOR_FAULT_NONE);
    assert(motor_controller_is_output_allowed(&controller) == 0U);

    motor_controller_step(&controller, MOTOR_CMD_ENABLE, MOTOR_FAULT_NONE);
    assert(controller.state == MOTOR_STATE_CALIBRATING);

    motor_controller_step(&controller,
                           MOTOR_CMD_CALIBRATION_DONE,
                           MOTOR_FAULT_NONE);
    assert(controller.state == MOTOR_STATE_READY);

    motor_controller_step(&controller, MOTOR_CMD_START, MOTOR_FAULT_NONE);
    assert(controller.state == MOTOR_STATE_RUNNING);
    assert(motor_controller_is_output_allowed(&controller) == 1U);

    motor_controller_step(&controller, MOTOR_CMD_STOP, MOTOR_FAULT_NONE);
    assert(controller.state == MOTOR_STATE_DISABLED);
    assert(motor_controller_is_output_allowed(&controller) == 0U);
}

static void test_fault_is_latched(void)
{
    MotorController controller;

    motor_controller_init(&controller);
    motor_controller_step(&controller, MOTOR_CMD_ENABLE, MOTOR_FAULT_NONE);
    motor_controller_step(&controller,
                           MOTOR_CMD_CALIBRATION_DONE,
                           MOTOR_FAULT_NONE);
    motor_controller_step(&controller, MOTOR_CMD_START, MOTOR_FAULT_NONE);

    motor_controller_step(&controller,
                           MOTOR_CMD_NONE,
                           MOTOR_FAULT_OVERCURRENT);
    assert(controller.state == MOTOR_STATE_FAULT);
    assert(controller.fault == MOTOR_FAULT_OVERCURRENT);
    assert(motor_controller_is_output_allowed(&controller) == 0U);

    motor_controller_step(&controller, MOTOR_CMD_START, MOTOR_FAULT_NONE);
    assert(controller.state == MOTOR_STATE_FAULT);

    motor_controller_step(&controller,
                           MOTOR_CMD_CLEAR_FAULT,
                           MOTOR_FAULT_NONE);
    assert(controller.state == MOTOR_STATE_DISABLED);
    assert(controller.fault == MOTOR_FAULT_NONE);
}

static void test_invalid_commands_do_not_skip_safety_states(void)
{
    MotorController controller;

    motor_controller_init(&controller);
    motor_controller_step(&controller, MOTOR_CMD_START, MOTOR_FAULT_NONE);
    assert(controller.state == MOTOR_STATE_DISABLED);

    motor_controller_step(&controller, MOTOR_CMD_ENABLE, MOTOR_FAULT_NONE);
    motor_controller_step(&controller, MOTOR_CMD_START, MOTOR_FAULT_NONE);
    assert(controller.state == MOTOR_STATE_CALIBRATING);
}

int main(void)
{
    test_normal_start_stop_sequence();
    test_fault_is_latched();
    test_invalid_commands_do_not_skip_safety_states();

    puts("All motor_state tests passed.");
    return 0;
}
