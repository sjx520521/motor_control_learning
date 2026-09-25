#include <assert.h>
#include <math.h>
#include <stdio.h>

#include "../Core/Inc/control_utils.h"
#include "../Core/Inc/current_sense.h"
#include "../Core/Inc/foc_math.h"
#include "../Core/Inc/motor_state.h"
#include "../Core/Inc/svpwm.h"

#define TEST_EPSILON (0.001f)

static void assert_near(float actual, float expected)
{
    assert(fabsf(actual - expected) < TEST_EPSILON);
}

static uint16_t current_to_raw(float current, uint16_t offset)
{
    return (uint16_t)lroundf((float)offset + current / 0.01f);
}

static void test_current_to_dq_chain(void)
{
    CurrentSenseConfig current_config =
    {
        .offset_raw_a = 2048U,
        .offset_raw_b = 2048U,
        .amp_per_count_a = 0.01f,
        .amp_per_count_b = 0.01f
    };

    current_sense_init(&current_config);

    const float angles[] = {0.0f, 0.7f, 2.1f, 4.8f};
    for (unsigned int i = 0U; i < sizeof(angles) / sizeof(angles[0]); ++i)
    {
        const float angle = angles[i];
        const float sin_theta = sinf(angle);
        const float cos_theta = cosf(angle);
        const DqAxis expected_dq = {.d = 2.0f, .q = 1.0f};
        const AlphaBeta alpha_beta =
            foc_inverse_park(expected_dq, sin_theta, cos_theta);
        const Phase3 phase = foc_inverse_clarke(alpha_beta);
        const PhaseCurrent measured =
            current_sense_get_phase_current(
                current_to_raw(phase.a, 2048U),
                current_to_raw(phase.b, 2048U));
        const Phase3 measured_phase =
        {
            .a = measured.phase_a,
            .b = measured.phase_b,
            .c = measured.phase_c
        };
        const AlphaBeta measured_alpha_beta =
            foc_clarke(measured_phase);
        const DqAxis measured_dq =
            foc_park(measured_alpha_beta, sin_theta, cos_theta);

        assert_near(measured_dq.d, expected_dq.d);
        assert_near(measured_dq.q, expected_dq.q);
    }
}

static void test_pi_vector_limit_and_svpwm(void)
{
    PI_Controller pi_d;
    PI_Controller pi_q;
    const float bus_voltage = 24.0f;
    const float voltage_limit = bus_voltage * 0.57735026919f;

    pi_controller_init(&pi_d, 1.0f, 100.0f, -13.0f, 13.0f);
    pi_controller_init(&pi_q, 1.0f, 100.0f, -13.0f, 13.0f);
    pi_controller_set_anti_windup(&pi_d, 500.0f);
    pi_controller_set_anti_windup(&pi_q, 500.0f);

    DqAxis voltage_dq =
    {
        .d = pi_controller_update(&pi_d, 20.0f, 1.0f / 20000.0f),
        .q = pi_controller_update(&pi_q, 20.0f, 1.0f / 20000.0f)
    };
    const float magnitude =
        sqrtf(voltage_dq.d * voltage_dq.d +
              voltage_dq.q * voltage_dq.q);

    assert(magnitude > voltage_limit);

    const float scale = voltage_limit / magnitude;
    voltage_dq.d *= scale;
    voltage_dq.q *= scale;
    pi_controller_apply_output_feedback(&pi_d,
                                        voltage_dq.d,
                                        1.0f / 20000.0f);
    pi_controller_apply_output_feedback(&pi_q,
                                        voltage_dq.q,
                                        1.0f / 20000.0f);

    assert_near(sqrtf(voltage_dq.d * voltage_dq.d +
                      voltage_dq.q * voltage_dq.q),
                voltage_limit);

    const AlphaBeta voltage_alpha_beta =
        foc_inverse_park(voltage_dq, 0.0f, 1.0f);
    const SvpwmOutput pwm =
        svpwm_calculate(voltage_alpha_beta.alpha,
                        voltage_alpha_beta.beta,
                        bus_voltage);

    assert(pwm.duty_a >= 0.0f && pwm.duty_a <= 1.0f);
    assert(pwm.duty_b >= 0.0f && pwm.duty_b <= 1.0f);
    assert(pwm.duty_c >= 0.0f && pwm.duty_c <= 1.0f);
}

static void test_fault_lock(void)
{
    MotorController controller;

    motor_controller_init(&controller);
    motor_controller_step(&controller, MOTOR_CMD_ENABLE, MOTOR_FAULT_NONE);
    motor_controller_step(&controller,
                           MOTOR_CMD_CALIBRATION_DONE,
                           MOTOR_FAULT_NONE);
    motor_controller_step(&controller, MOTOR_CMD_START, MOTOR_FAULT_NONE);
    motor_controller_step(&controller,
                           MOTOR_CMD_FAULT,
                           MOTOR_FAULT_OVERCURRENT);

    assert(controller.state == MOTOR_STATE_FAULT);
    assert(controller.fault == MOTOR_FAULT_OVERCURRENT);
    assert(motor_controller_is_output_allowed(&controller) == 0U);

    motor_controller_step(&controller, MOTOR_CMD_START, MOTOR_FAULT_NONE);
    assert(controller.state == MOTOR_STATE_FAULT);
}

int main(void)
{
    test_current_to_dq_chain();
    test_pi_vector_limit_and_svpwm();
    test_fault_lock();

    puts("All FOC control-chain tests passed.");
    return 0;
}
