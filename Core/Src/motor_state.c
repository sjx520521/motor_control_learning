/*
 * motor_state.c
 *
 * Basic motor-control state machine used by the application layer.
 */

#include "motor_state.h"

void motor_controller_init(MotorController *controller)
{
    if (controller == 0)
    {
        return;
    }

    controller->state = MOTOR_STATE_DISABLED;
    controller->fault = MOTOR_FAULT_NONE;
}

void motor_controller_step(MotorController *controller,
                           MotorCommand command,
                           MotorFault fault)
{
    if (controller == 0)
    {
        return;
    }

    /*
     * A fault has priority over normal commands and is latched until
     * MOTOR_CMD_CLEAR_FAULT is received while the controller is stopped.
     */
    if (command == MOTOR_CMD_FAULT || fault != MOTOR_FAULT_NONE)
    {
        controller->fault = (fault == MOTOR_FAULT_NONE)
                                ? MOTOR_FAULT_DRIVER
                                : fault;
        controller->state = MOTOR_STATE_FAULT;
        return;
    }

    switch (controller->state)
    {
    case MOTOR_STATE_DISABLED:
        if (command == MOTOR_CMD_ENABLE)
        {
            controller->state = MOTOR_STATE_CALIBRATING;
        }
        break;

    case MOTOR_STATE_CALIBRATING:
        if (command == MOTOR_CMD_STOP)
        {
            controller->state = MOTOR_STATE_DISABLED;
        }
        else if (command == MOTOR_CMD_CALIBRATION_DONE)
        {
            controller->state = MOTOR_STATE_READY;
        }
        break;

    case MOTOR_STATE_READY:
        if (command == MOTOR_CMD_STOP)
        {
            controller->state = MOTOR_STATE_DISABLED;
        }
        else if (command == MOTOR_CMD_START)
        {
            controller->state = MOTOR_STATE_RUNNING;
        }
        break;

    case MOTOR_STATE_RUNNING:
        if (command == MOTOR_CMD_STOP)
        {
            controller->state = MOTOR_STATE_DISABLED;
        }
        break;

    case MOTOR_STATE_FAULT:
        if (command == MOTOR_CMD_CLEAR_FAULT)
        {
            controller->fault = MOTOR_FAULT_NONE;
            controller->state = MOTOR_STATE_DISABLED;
        }
        break;

    default:
        controller->state = MOTOR_STATE_FAULT;
        controller->fault = MOTOR_FAULT_DRIVER;
        break;
    }
}

uint8_t motor_controller_is_output_allowed(
    const MotorController *controller)
{
    if (controller == 0)
    {
        return 0U;
    }

    return (controller->state == MOTOR_STATE_RUNNING) ? 1U : 0U;
}
