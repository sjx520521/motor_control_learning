/*
 * motor_state.h
 *
 * Basic motor-control state machine used by the application layer.
 */

#ifndef MOTOR_STATE_H_
#define MOTOR_STATE_H_

#include <stdint.h>

typedef enum
{
    MOTOR_STATE_DISABLED = 0,
    MOTOR_STATE_CALIBRATING,
    MOTOR_STATE_READY,
    MOTOR_STATE_RUNNING,
    MOTOR_STATE_FAULT
} MotorState;

typedef enum
{
    MOTOR_FAULT_NONE = 0,
    MOTOR_FAULT_OVERCURRENT,
    MOTOR_FAULT_OVERVOLTAGE,
    MOTOR_FAULT_UNDERVOLTAGE,
    MOTOR_FAULT_ENCODER,
    MOTOR_FAULT_DRIVER
} MotorFault;

typedef enum
{
    MOTOR_CMD_NONE = 0,
    MOTOR_CMD_ENABLE,
    MOTOR_CMD_CALIBRATION_DONE,
    MOTOR_CMD_START,
    MOTOR_CMD_STOP,
    MOTOR_CMD_FAULT,
    MOTOR_CMD_CLEAR_FAULT
} MotorCommand;

typedef struct
{
    MotorState state;
    MotorFault fault;
} MotorController;

void motor_controller_init(MotorController *controller);

void motor_controller_step(MotorController *controller,
                           MotorCommand command,
                           MotorFault fault);

uint8_t motor_controller_is_output_allowed(
    const MotorController *controller);

#endif /* MOTOR_STATE_H_ */
