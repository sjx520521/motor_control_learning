/* USER CODE BEGIN Header */
/**
  ******************************************************************************
  * @file           : main.c
  * @brief          : Main program body
  ******************************************************************************
  * @attention
  *
  * Copyright (c) 2026 STMicroelectronics.
  * All rights reserved.
  *
  * This software is licensed under terms that can be found in the LICENSE file
  * in the root directory of this software component.
  * If no LICENSE file comes with this software, it is provided AS-IS.
  *
  ******************************************************************************
  */
/* USER CODE END Header */
/* Includes ------------------------------------------------------------------*/
#include "main.h"

/* Private includes ----------------------------------------------------------*/
/* USER CODE BEGIN Includes */
#include <stdio.h>
#include "svpwm.h"
#include "current_sense.h"
#include "bus_voltage.h"
#include "foc_math.h"
#include "control_utils.h"
#include "motor_state.h"
#include "motion_control.h"
#include "dual_encoder.h"
#include <math.h>
/* USER CODE END Includes */

/* Private typedef -----------------------------------------------------------*/
/* USER CODE BEGIN PTD */

/* USER CODE END PTD */

/* Private define ------------------------------------------------------------*/
/* USER CODE BEGIN PD */

/* USER CODE END PD */

/* Private macro -------------------------------------------------------------*/
/* USER CODE BEGIN PM */

/* USER CODE END PM */

/* Private variables ---------------------------------------------------------*/

ADC_HandleTypeDef hadc1;
ADC_HandleTypeDef hadc2;

UART_HandleTypeDef hlpuart1;

TIM_HandleTypeDef htim1;
TIM_HandleTypeDef htim6;

/* USER CODE BEGIN PV */
volatile uint32_t g_tick_ms = 0;
uint32_t last_report_ms = 0;

/* ADC injected conversion results */
volatile uint16_t g_adc1_current_raw = 0;
volatile uint16_t g_adc2_current_raw = 0;

/* ADC conversion synchronization flags */
volatile uint8_t g_adc1_sample_done = 0;
volatile uint8_t g_adc2_sample_done = 0;
volatile uint8_t g_adc_sample_ready = 0;

CurrentSenseConfig g_current_sense_config =
{
    .offset_raw_a = 2048U,
    .offset_raw_b = 2048U,
    .amp_per_count_a = 0.008058608f,
    .amp_per_count_b = 0.008058608f
};

BusVoltageConfig g_bus_voltage_config =
{
    .adc_reference_voltage = 3.3f,
    .divider_ratio = 11.0f,
    .filter_alpha = 0.1f
};

PhaseCurrent g_phase_current =
{
    .phase_a = 0.0f,
    .phase_b = 0.0f,
    .phase_c = 0.0f
};

float g_i_alpha = 0.0f;
float g_i_beta = 0.0f;
float g_i_d = 0.0f;
float g_i_q = 0.0f;

/* Dual-encoder state: simulated inputs are used until hardware is attached. */
DualEncoderController g_dual_encoder;
float g_motor_mechanical_angle = 0.0f;
float g_joint_position = 0.0f;
float g_motor_speed = 0.0f;
float g_joint_speed = 0.0f;
float g_electrical_angle = 0.0f;
float g_simulated_motor_encoder_angle = 0.0f;
float g_simulated_joint_encoder_angle = 0.0f;

/* Open-loop commissioning values until bus-voltage and encoder drivers exist. */
#define MOTOR_CONTROL_PERIOD_SECONDS (1.0f / 20000.0f)
#define MOTOR_DEFAULT_BUS_VOLTAGE    (24.0f)
#define MOTOR_MAX_MODULATION_VOLTAGE (0.57735026919f)
#define MOTOR_CURRENT_REFERENCE_D    (0.0f)
#define MOTOR_CURRENT_REFERENCE_Q    (g_iq_reference)
#define MOTOR_TORQUE_CONSTANT        (0.1f)
#define MOTOR_MAX_IQ_REFERENCE       (8.0f)
#define MOTOR_SPEED_LOOP_PERIOD_SECONDS (0.001f)
#define MOTOR_POSITION_LOOP_PERIOD_SECONDS (0.010f)
#define MOTOR_SPEED_LOOP_DIVIDER       (20U)
#define MOTOR_POSITION_LOOP_DIVIDER    (10U)
#define MOTOR_OVERCURRENT_LIMIT_AMP  (10.0f)
#define MOTOR_MIN_BUS_VOLTAGE        (8.0f)
#define MOTOR_MAX_BUS_VOLTAGE        (30.0f)

PI_Controller g_current_pi_d;
PI_Controller g_current_pi_q;
MotorController g_motor_controller;
MotionController g_motion_controller;
float g_iq_reference = 0.0f;
float g_simulated_position_reference = 0.0f;
float g_simulated_velocity_feedforward = 0.0f;
float g_simulated_torque_feedforward = 0.0f;
uint16_t g_speed_loop_divider = 0U;
uint8_t g_position_loop_divider = 0U;
volatile MotorCommand g_pending_motor_command = MOTOR_CMD_NONE;

/* USER CODE END PV */

/* Private function prototypes -----------------------------------------------*/
void SystemClock_Config(void);
static void MX_GPIO_Init(void);
static void MX_LPUART1_UART_Init(void);
static void MX_TIM6_Init(void);
static void MX_TIM1_Init(void);
static void MX_ADC1_Init(void);
static void MX_ADC2_Init(void);
/* USER CODE BEGIN PFP */

/* USER CODE END PFP */

/* Private user code ---------------------------------------------------------*/
/* USER CODE BEGIN 0 */
static uint32_t motor_pwm_duty_to_ccr(float duty)
{
    uint32_t period_counts = htim1.Init.Period + 1U;

    if (duty < 0.0f)
    {
        duty = 0.0f;
    }
    else if (duty > 1.0f)
    {
        duty = 1.0f;
    }

    return (uint32_t)(duty * (float)period_counts + 0.5f);
}

static void motor_pwm_set_duty(float duty_a, float duty_b, float duty_c)
{
    __HAL_TIM_SET_COMPARE(&htim1, TIM_CHANNEL_1,
                          motor_pwm_duty_to_ccr(duty_a));
    __HAL_TIM_SET_COMPARE(&htim1, TIM_CHANNEL_2,
                          motor_pwm_duty_to_ccr(duty_b));
    __HAL_TIM_SET_COMPARE(&htim1, TIM_CHANNEL_3,
                          motor_pwm_duty_to_ccr(duty_c));
}

static void motor_pwm_update_voltage(float v_alpha,
                                     float v_beta,
                                     float v_bus)
{
    SvpwmOutput pwm = svpwm_calculate(v_alpha, v_beta, v_bus);

    motor_pwm_set_duty(pwm.duty_a,
                        pwm.duty_b,
                        pwm.duty_c);
}

static void motor_pwm_set_neutral(void)
{
    motor_pwm_set_duty(0.5f, 0.5f, 0.5f);
}

static void motor_pwm_disable_outputs(void)
{
    __HAL_TIM_MOE_DISABLE(&htim1);
    motor_pwm_set_neutral();
}

static void motor_pwm_enable_outputs(void)
{
    __HAL_TIM_MOE_ENABLE(&htim1);
}

static void motor_control_latch_fault(MotorFault fault);

static void motor_control_reset_current_pi(void)
{
    g_current_pi_d.integral = 0.0f;
    g_current_pi_q.integral = 0.0f;
    g_current_pi_d.last_unsaturated_output = 0.0f;
    g_current_pi_d.last_output = 0.0f;
    g_current_pi_q.last_unsaturated_output = 0.0f;
    g_current_pi_q.last_output = 0.0f;
}

static void motor_control_init(void)
{
    /*
     * These gains are deliberately conservative commissioning values. They
     * must be retuned after the motor resistance, inductance and sensor
     * scaling are known.
     */
    pi_controller_init(&g_current_pi_d,
                       1.0f,
                       100.0f,
                       -13.0f,
                       13.0f);
    pi_controller_init(&g_current_pi_q,
                       1.0f,
                       100.0f,
                       -13.0f,
                       13.0f);
    pi_controller_set_anti_windup(&g_current_pi_d, 500.0f);
    pi_controller_set_anti_windup(&g_current_pi_q, 500.0f);

    const MotionControlConfig motion_config =
    {
        .velocity_filter_alpha = 0.2f,
        .velocity_kp = 0.08f,
        .velocity_ki = 2.0f,
        .velocity_output_min = -8.0f,
        .velocity_output_max = 8.0f,
        .position_kp = 10.0f,
        .position_min = -3.14159265359f,
        .position_max = 3.14159265359f,
        .velocity_limit = 20.0f,
        .torque_feedforward_gain = 1.0f,
        .velocity_feedforward_gain = 1.0f
    };

    motion_controller_init(&g_motion_controller,
                           &motion_config,
                           0.0f);

    motor_controller_init(&g_motor_controller);
    motor_controller_step(&g_motor_controller,
                           MOTOR_CMD_ENABLE,
                           MOTOR_FAULT_NONE);
    motor_controller_step(&g_motor_controller,
                           MOTOR_CMD_CALIBRATION_DONE,
                           MOTOR_FAULT_NONE);
    motor_control_reset_current_pi();
    const DualEncoderConfig encoder_config =
    {
        .pole_pairs = 4U,
        .electrical_offset = 0.0f,
        .gear_ratio = 50.0f,
        .motor_velocity_filter_alpha = 0.2f,
        .joint_velocity_filter_alpha = 0.2f,
        .position_error_limit = 0.20f,
        .speed_error_limit = 5.0f
    };

    dual_encoder_init(&g_dual_encoder,
                      &encoder_config,
                      g_simulated_motor_encoder_angle,
                      g_simulated_joint_encoder_angle);
    const DualEncoderState *encoder_state =
        dual_encoder_get_state(&g_dual_encoder);
    g_motor_mechanical_angle = encoder_state->motor_mechanical_angle;
    g_joint_position = encoder_state->joint_position;
    g_motor_speed = encoder_state->motor_speed;
    g_joint_speed = encoder_state->joint_speed;
    g_electrical_angle = encoder_state->motor_electrical_angle;

    motion_controller_reset(&g_motion_controller,
                            g_joint_position);
    g_iq_reference = 0.0f;
    g_speed_loop_divider = 0U;
    g_position_loop_divider = 0U;
    motor_pwm_disable_outputs();
}

static void motor_control_reset_outer_loops(void)
{
    motion_controller_reset(&g_motion_controller,
                            g_joint_position);
    g_iq_reference = 0.0f;
    g_speed_loop_divider = 0U;
    g_position_loop_divider = 0U;
}

static void motor_control_update_outer_loops(void)
{
    if (++g_speed_loop_divider < MOTOR_SPEED_LOOP_DIVIDER)
    {
        return;
    }

    g_speed_loop_divider = 0U;

    if (dual_encoder_update(&g_dual_encoder,
                            g_simulated_motor_encoder_angle,
                            g_simulated_joint_encoder_angle,
                            MOTOR_SPEED_LOOP_PERIOD_SECONDS) == 0U)
    {
        motor_control_latch_fault(MOTOR_FAULT_ENCODER);
        return;
    }

    const DualEncoderState *encoder_state =
        dual_encoder_get_state(&g_dual_encoder);
    g_motor_mechanical_angle = encoder_state->motor_mechanical_angle;
    g_joint_position = encoder_state->joint_position;
    g_motor_speed = encoder_state->motor_speed;
    g_joint_speed = encoder_state->joint_speed;
    g_electrical_angle = encoder_state->motor_electrical_angle;

    motion_controller_set_feedback(&g_motion_controller,
                                   g_joint_position,
                                   g_joint_speed);

    if (++g_position_loop_divider >= MOTOR_POSITION_LOOP_DIVIDER)
    {
        g_position_loop_divider = 0U;
        motion_controller_update_position_loop(
            &g_motion_controller,
            g_simulated_position_reference,
            g_simulated_velocity_feedforward,
            MOTOR_POSITION_LOOP_PERIOD_SECONDS);
    }

    g_iq_reference =
        motion_controller_update_speed_loop(
            &g_motion_controller,
            motion_controller_get_velocity_reference(
                &g_motion_controller),
            g_simulated_torque_feedforward,
            MOTOR_SPEED_LOOP_PERIOD_SECONDS);

    g_iq_reference =
        mc_clamp_f32(
            motion_controller_get_torque_reference(
                &g_motion_controller) / MOTOR_TORQUE_CONSTANT,
            -MOTOR_MAX_IQ_REFERENCE,
            MOTOR_MAX_IQ_REFERENCE);
}

static void motor_control_apply_pending_command(void)
{
    MotorCommand command;

    __disable_irq();
    command = g_pending_motor_command;
    g_pending_motor_command = MOTOR_CMD_NONE;
    __enable_irq();

    if (command != MOTOR_CMD_NONE)
    {
        motor_controller_step(&g_motor_controller,
                               command,
                               MOTOR_FAULT_NONE);

        if (g_motor_controller.state == MOTOR_STATE_RUNNING)
        {
            motor_pwm_enable_outputs();
        }
        else
        {
            motor_pwm_disable_outputs();
            motor_control_reset_current_pi();
            motor_control_reset_outer_loops();
        }
    }
}

static void motor_control_latch_fault(MotorFault fault)
{
    motor_controller_step(&g_motor_controller,
                           MOTOR_CMD_FAULT,
                           fault);
    motor_control_reset_current_pi();
    motor_control_reset_outer_loops();
    motor_pwm_disable_outputs();
}

static MotorFault motor_control_check_protection(
    const PhaseCurrent *phase_current)
{
    const float bus_voltage = bus_voltage_get();

    if (phase_current == 0)
    {
        return MOTOR_FAULT_DRIVER;
    }

    if (fabsf(phase_current->phase_a) > MOTOR_OVERCURRENT_LIMIT_AMP ||
        fabsf(phase_current->phase_b) > MOTOR_OVERCURRENT_LIMIT_AMP ||
        fabsf(phase_current->phase_c) > MOTOR_OVERCURRENT_LIMIT_AMP)
    {
        return MOTOR_FAULT_OVERCURRENT;
    }

    if (bus_voltage < MOTOR_MIN_BUS_VOLTAGE)
    {
        return MOTOR_FAULT_UNDERVOLTAGE;
    }

    if (bus_voltage > MOTOR_MAX_BUS_VOLTAGE)
    {
        return MOTOR_FAULT_OVERVOLTAGE;
    }

    return MOTOR_FAULT_NONE;
}

static void motor_control_run_current_loop(void)
{
    const float sin_theta = sinf(g_electrical_angle);
    const float cos_theta = cosf(g_electrical_angle);
    const float v_bus = bus_voltage_get();
    const float voltage_limit =
        v_bus * MOTOR_MAX_MODULATION_VOLTAGE;

    DqAxis voltage_dq;
    AlphaBeta voltage_alpha_beta;

    voltage_dq.d = pi_controller_update(&g_current_pi_d,
                                         MOTOR_CURRENT_REFERENCE_D - g_i_d,
                                         MOTOR_CONTROL_PERIOD_SECONDS);
    voltage_dq.q = pi_controller_update(&g_current_pi_q,
                                         MOTOR_CURRENT_REFERENCE_Q - g_i_q,
                                         MOTOR_CONTROL_PERIOD_SECONDS);

    /*
     * Keep the requested voltage vector inside the linear SVPWM range.
     * Scaling both axes preserves its direction when the PI outputs saturate.
     */
    const float voltage_magnitude =
        sqrtf(voltage_dq.d * voltage_dq.d +
              voltage_dq.q * voltage_dq.q);

    if (voltage_magnitude > voltage_limit &&
        voltage_magnitude > 0.0f)
    {
        const float scale = voltage_limit / voltage_magnitude;
        voltage_dq.d *= scale;
        voltage_dq.q *= scale;
    }

    pi_controller_apply_output_feedback(
        &g_current_pi_d,
        voltage_dq.d,
        MOTOR_CONTROL_PERIOD_SECONDS);
    pi_controller_apply_output_feedback(
        &g_current_pi_q,
        voltage_dq.q,
        MOTOR_CONTROL_PERIOD_SECONDS);

    voltage_alpha_beta =
        foc_inverse_park(voltage_dq, sin_theta, cos_theta);

    motor_pwm_update_voltage(voltage_alpha_beta.alpha,
                             voltage_alpha_beta.beta,
                             v_bus);
}
/* USER CODE END 0 */

/**
  * @brief  The application entry point.
  * @retval int
  */
int main(void)
{

  /* USER CODE BEGIN 1 */

  /* USER CODE END 1 */

  /* MCU Configuration--------------------------------------------------------*/

  /* Reset of all peripherals, Initializes the Flash interface and the Systick. */
  HAL_Init();

  /* USER CODE BEGIN Init */

  /* USER CODE END Init */

  /* Configure the system clock */
  SystemClock_Config();

  /* USER CODE BEGIN SysInit */

  /* USER CODE END SysInit */

  /* Initialize all configured peripherals */
  MX_GPIO_Init();
  MX_LPUART1_UART_Init();
  MX_TIM6_Init();
  MX_TIM1_Init();
  MX_ADC1_Init();
  MX_ADC2_Init();
  /* USER CODE BEGIN 2 */

  /*
   * ADC calibration must be completed before starting conversions.
   */

   current_sense_init(&g_current_sense_config);
   bus_voltage_init(&g_bus_voltage_config);
   bus_voltage_set_simulated(MOTOR_DEFAULT_BUS_VOLTAGE);

  if (HAL_ADCEx_Calibration_Start(&hadc1,
                                  ADC_SINGLE_ENDED) != HAL_OK)
  {
    Error_Handler();
  }

  if (HAL_ADCEx_Calibration_Start(&hadc2,
                                  ADC_SINGLE_ENDED) != HAL_OK)
  {
    Error_Handler();
  }

  /*
   * Start injected conversions in interrupt mode.
   * The ADCs will now wait for the TIM1_CC4 trigger.
   */
  if (HAL_ADCEx_InjectedStart_IT(&hadc1) != HAL_OK)
  {
    Error_Handler();
  }

  if (HAL_ADCEx_InjectedStart_IT(&hadc2) != HAL_OK)
  {
    Error_Handler();
  }

  /*
   * Start TIM1 only after both ADCs are armed. CH4 is an internal compare
   * event, so it must be started explicitly to generate the ADC trigger.
   * Begin with a neutral 50 percent duty on all phases.
   */
  motor_pwm_set_duty(0.5f, 0.5f, 0.5f);

  if (HAL_TIM_PWM_Start(&htim1, TIM_CHANNEL_1) != HAL_OK ||
      HAL_TIM_PWM_Start(&htim1, TIM_CHANNEL_2) != HAL_OK ||
      HAL_TIM_PWM_Start(&htim1, TIM_CHANNEL_3) != HAL_OK ||
      HAL_TIMEx_PWMN_Start(&htim1, TIM_CHANNEL_1) != HAL_OK ||
      HAL_TIMEx_PWMN_Start(&htim1, TIM_CHANNEL_2) != HAL_OK ||
      HAL_TIMEx_PWMN_Start(&htim1, TIM_CHANNEL_3) != HAL_OK ||
      HAL_TIM_OC_Start(&htim1, TIM_CHANNEL_4) != HAL_OK)
  {
    Error_Handler();
  }

  if (HAL_TIM_Base_Start_IT(&htim6) != HAL_OK)
  {
    Error_Handler();
  }

  motor_control_init();

  /* USER CODE END 2 */

  /* Initialize led */
  BSP_LED_Init(LED_GREEN);

  /* Initialize USER push-button, will be used to trigger an interrupt each time it's pressed.*/
  BSP_PB_Init(BUTTON_USER, BUTTON_MODE_EXTI);

  /* Infinite loop */
  /* USER CODE BEGIN WHILE */
  while (1)
  {
    motor_control_apply_pending_command();

    if (g_adc_sample_ready != 0U)
    {
      uint16_t raw_a;
      uint16_t raw_b;

      /*
       * Copy ADC results atomically.
       */
      __disable_irq();

      raw_a = g_adc1_current_raw;
      raw_b = g_adc2_current_raw;
      g_adc_sample_ready = 0U;

      __enable_irq();

      /*
       * Convert ADC raw values into Ia, Ib and Ic.
       */
      g_phase_current =
          current_sense_get_phase_current(raw_a, raw_b);

      const MotorFault protection_fault =
          motor_control_check_protection(&g_phase_current);

      if (protection_fault != MOTOR_FAULT_NONE)
      {
        motor_control_latch_fault(protection_fault);
        continue;
      }

      /*
       * Adapt the current-sense structure to the FOC structure.
       */
      Phase3 phase_current =
      {
        .a = g_phase_current.phase_a,
        .b = g_phase_current.phase_b,
        .c = g_phase_current.phase_c
      };

      /*
       * Clarke transform:
       * three-phase currents -> stationary alpha-beta currents.
       */
      AlphaBeta stationary =
          foc_clarke(phase_current);

      g_i_alpha = stationary.alpha;
      g_i_beta = stationary.beta;

      /*
       * Park transform:
       * stationary alpha-beta currents -> rotating d-q currents.
       *
       * The Park transform uses the motor-side encoder electrical angle.
       * The joint position loop uses the output-side encoder instead.
       */
      float sin_theta = sinf(g_electrical_angle);
      float cos_theta = cosf(g_electrical_angle);

      DqAxis rotating =
          foc_park(stationary,
                   sin_theta,
                   cos_theta);

      g_i_d = rotating.d;
      g_i_q = rotating.q;

      motor_control_update_outer_loops();

      if (motor_controller_is_output_allowed(&g_motor_controller) != 0U)
      {
        motor_control_run_current_loop();
      }
      else
      {
        motor_control_reset_current_pi();
        motor_pwm_set_neutral();
      }
    }
  }
  /* USER CODE END 3 */
}

/**
  * @brief System Clock Configuration
  * @retval None
  */
void SystemClock_Config(void)
{
  RCC_OscInitTypeDef RCC_OscInitStruct = {0};
  RCC_ClkInitTypeDef RCC_ClkInitStruct = {0};

  /** Configure the main internal regulator output voltage
  */
  HAL_PWREx_ControlVoltageScaling(PWR_REGULATOR_VOLTAGE_SCALE1_BOOST);

  /** Initializes the RCC Oscillators according to the specified parameters
  * in the RCC_OscInitTypeDef structure.
  */
  RCC_OscInitStruct.OscillatorType = RCC_OSCILLATORTYPE_HSI;
  RCC_OscInitStruct.HSIState = RCC_HSI_ON;
  RCC_OscInitStruct.HSICalibrationValue = RCC_HSICALIBRATION_DEFAULT;
  RCC_OscInitStruct.PLL.PLLState = RCC_PLL_ON;
  RCC_OscInitStruct.PLL.PLLSource = RCC_PLLSOURCE_HSI;
  RCC_OscInitStruct.PLL.PLLM = RCC_PLLM_DIV4;
  RCC_OscInitStruct.PLL.PLLN = 85;
  RCC_OscInitStruct.PLL.PLLP = RCC_PLLP_DIV2;
  RCC_OscInitStruct.PLL.PLLQ = RCC_PLLQ_DIV2;
  RCC_OscInitStruct.PLL.PLLR = RCC_PLLR_DIV2;
  if (HAL_RCC_OscConfig(&RCC_OscInitStruct) != HAL_OK)
  {
    Error_Handler();
  }

  /** Initializes the CPU, AHB and APB buses clocks
  */
  RCC_ClkInitStruct.ClockType = RCC_CLOCKTYPE_HCLK|RCC_CLOCKTYPE_SYSCLK
                              |RCC_CLOCKTYPE_PCLK1|RCC_CLOCKTYPE_PCLK2;
  RCC_ClkInitStruct.SYSCLKSource = RCC_SYSCLKSOURCE_PLLCLK;
  RCC_ClkInitStruct.AHBCLKDivider = RCC_SYSCLK_DIV1;
  RCC_ClkInitStruct.APB1CLKDivider = RCC_HCLK_DIV1;
  RCC_ClkInitStruct.APB2CLKDivider = RCC_HCLK_DIV1;

  if (HAL_RCC_ClockConfig(&RCC_ClkInitStruct, FLASH_LATENCY_4) != HAL_OK)
  {
    Error_Handler();
  }
}

/**
  * @brief ADC1 Initialization Function
  * @param None
  * @retval None
  */
static void MX_ADC1_Init(void)
{

  /* USER CODE BEGIN ADC1_Init 0 */

  /* USER CODE END ADC1_Init 0 */

  ADC_MultiModeTypeDef multimode = {0};
  ADC_ChannelConfTypeDef sConfig = {0};
  ADC_InjectionConfTypeDef sConfigInjected = {0};

  /* USER CODE BEGIN ADC1_Init 1 */

  /* USER CODE END ADC1_Init 1 */

  /** Common config
  */
  hadc1.Instance = ADC1;
  hadc1.Init.ClockPrescaler = ADC_CLOCK_SYNC_PCLK_DIV4;
  hadc1.Init.Resolution = ADC_RESOLUTION_12B;
  hadc1.Init.DataAlign = ADC_DATAALIGN_RIGHT;
  hadc1.Init.GainCompensation = 0;
  hadc1.Init.ScanConvMode = ADC_SCAN_DISABLE;
  hadc1.Init.EOCSelection = ADC_EOC_SINGLE_CONV;
  hadc1.Init.LowPowerAutoWait = DISABLE;
  hadc1.Init.ContinuousConvMode = DISABLE;
  hadc1.Init.NbrOfConversion = 1;
  hadc1.Init.DiscontinuousConvMode = DISABLE;
  hadc1.Init.ExternalTrigConv = ADC_SOFTWARE_START;
  hadc1.Init.ExternalTrigConvEdge = ADC_EXTERNALTRIGCONVEDGE_NONE;
  hadc1.Init.DMAContinuousRequests = DISABLE;
  hadc1.Init.Overrun = ADC_OVR_DATA_OVERWRITTEN;
  hadc1.Init.OversamplingMode = DISABLE;
  if (HAL_ADC_Init(&hadc1) != HAL_OK)
  {
    Error_Handler();
  }

  /** Configure the ADC multi-mode
  */
  multimode.Mode = ADC_MODE_INDEPENDENT;
  if (HAL_ADCEx_MultiModeConfigChannel(&hadc1, &multimode) != HAL_OK)
  {
    Error_Handler();
  }

  /** Configure Regular Channel
  */
  sConfig.Channel = ADC_CHANNEL_1;
  sConfig.Rank = ADC_REGULAR_RANK_1;
  sConfig.SamplingTime = ADC_SAMPLETIME_2CYCLES_5;
  sConfig.SingleDiff = ADC_SINGLE_ENDED;
  sConfig.OffsetNumber = ADC_OFFSET_NONE;
  sConfig.Offset = 0;
  if (HAL_ADC_ConfigChannel(&hadc1, &sConfig) != HAL_OK)
  {
    Error_Handler();
  }

  /** Configure Injected Channel
  */
  sConfigInjected.InjectedChannel = ADC_CHANNEL_1;
  sConfigInjected.InjectedRank = ADC_INJECTED_RANK_1;
  sConfigInjected.InjectedSamplingTime = ADC_SAMPLETIME_47CYCLES_5;
  sConfigInjected.InjectedSingleDiff = ADC_SINGLE_ENDED;
  sConfigInjected.InjectedOffsetNumber = ADC_OFFSET_NONE;
  sConfigInjected.InjectedOffset = 0;
  sConfigInjected.InjectedNbrOfConversion = 1;
  sConfigInjected.InjectedDiscontinuousConvMode = DISABLE;
  sConfigInjected.AutoInjectedConv = DISABLE;
  sConfigInjected.QueueInjectedContext = DISABLE;
  sConfigInjected.ExternalTrigInjecConv = ADC_EXTERNALTRIGINJEC_T1_CC4;
  sConfigInjected.ExternalTrigInjecConvEdge = ADC_EXTERNALTRIGINJECCONV_EDGE_RISING;
  sConfigInjected.InjecOversamplingMode = DISABLE;
  if (HAL_ADCEx_InjectedConfigChannel(&hadc1, &sConfigInjected) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN ADC1_Init 2 */

  /* USER CODE END ADC1_Init 2 */

}

/**
  * @brief ADC2 Initialization Function
  * @param None
  * @retval None
  */
static void MX_ADC2_Init(void)
{

  /* USER CODE BEGIN ADC2_Init 0 */

  /* USER CODE END ADC2_Init 0 */

  ADC_ChannelConfTypeDef sConfig = {0};
  ADC_InjectionConfTypeDef sConfigInjected = {0};

  /* USER CODE BEGIN ADC2_Init 1 */

  /* USER CODE END ADC2_Init 1 */

  /** Common config
  */
  hadc2.Instance = ADC2;
  hadc2.Init.ClockPrescaler = ADC_CLOCK_SYNC_PCLK_DIV4;
  hadc2.Init.Resolution = ADC_RESOLUTION_12B;
  hadc2.Init.DataAlign = ADC_DATAALIGN_RIGHT;
  hadc2.Init.GainCompensation = 0;
  hadc2.Init.ScanConvMode = ADC_SCAN_DISABLE;
  hadc2.Init.EOCSelection = ADC_EOC_SINGLE_CONV;
  hadc2.Init.LowPowerAutoWait = DISABLE;
  hadc2.Init.ContinuousConvMode = DISABLE;
  hadc2.Init.NbrOfConversion = 1;
  hadc2.Init.DiscontinuousConvMode = DISABLE;
  hadc2.Init.ExternalTrigConv = ADC_SOFTWARE_START;
  hadc2.Init.ExternalTrigConvEdge = ADC_EXTERNALTRIGCONVEDGE_NONE;
  hadc2.Init.DMAContinuousRequests = DISABLE;
  hadc2.Init.Overrun = ADC_OVR_DATA_OVERWRITTEN;
  hadc2.Init.OversamplingMode = DISABLE;
  if (HAL_ADC_Init(&hadc2) != HAL_OK)
  {
    Error_Handler();
  }

  /** Configure Regular Channel
  */
  sConfig.Channel = ADC_CHANNEL_17;
  sConfig.Rank = ADC_REGULAR_RANK_1;
  sConfig.SamplingTime = ADC_SAMPLETIME_2CYCLES_5;
  sConfig.SingleDiff = ADC_SINGLE_ENDED;
  sConfig.OffsetNumber = ADC_OFFSET_NONE;
  sConfig.Offset = 0;
  if (HAL_ADC_ConfigChannel(&hadc2, &sConfig) != HAL_OK)
  {
    Error_Handler();
  }

  /** Configure Injected Channel
  */
  sConfigInjected.InjectedChannel = ADC_CHANNEL_17;
  sConfigInjected.InjectedRank = ADC_INJECTED_RANK_1;
  sConfigInjected.InjectedSamplingTime = ADC_SAMPLETIME_47CYCLES_5;
  sConfigInjected.InjectedSingleDiff = ADC_SINGLE_ENDED;
  sConfigInjected.InjectedOffsetNumber = ADC_OFFSET_NONE;
  sConfigInjected.InjectedOffset = 0;
  sConfigInjected.InjectedNbrOfConversion = 1;
  sConfigInjected.InjectedDiscontinuousConvMode = DISABLE;
  sConfigInjected.AutoInjectedConv = DISABLE;
  sConfigInjected.QueueInjectedContext = DISABLE;
  sConfigInjected.ExternalTrigInjecConv = ADC_EXTERNALTRIGINJEC_T1_CC4;
  sConfigInjected.ExternalTrigInjecConvEdge = ADC_EXTERNALTRIGINJECCONV_EDGE_RISING;
  sConfigInjected.InjecOversamplingMode = DISABLE;
  if (HAL_ADCEx_InjectedConfigChannel(&hadc2, &sConfigInjected) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN ADC2_Init 2 */

  /* USER CODE END ADC2_Init 2 */

}

/**
  * @brief LPUART1 Initialization Function
  * @param None
  * @retval None
  */
static void MX_LPUART1_UART_Init(void)
{

  /* USER CODE BEGIN LPUART1_Init 0 */

  /* USER CODE END LPUART1_Init 0 */

  /* USER CODE BEGIN LPUART1_Init 1 */

  /* USER CODE END LPUART1_Init 1 */
  hlpuart1.Instance = LPUART1;
  hlpuart1.Init.BaudRate = 209700;
  hlpuart1.Init.WordLength = UART_WORDLENGTH_8B;
  hlpuart1.Init.StopBits = UART_STOPBITS_1;
  hlpuart1.Init.Parity = UART_PARITY_NONE;
  hlpuart1.Init.Mode = UART_MODE_TX_RX;
  hlpuart1.Init.HwFlowCtl = UART_HWCONTROL_NONE;
  hlpuart1.Init.OneBitSampling = UART_ONE_BIT_SAMPLE_DISABLE;
  hlpuart1.Init.ClockPrescaler = UART_PRESCALER_DIV1;
  hlpuart1.AdvancedInit.AdvFeatureInit = UART_ADVFEATURE_NO_INIT;
  if (HAL_UART_Init(&hlpuart1) != HAL_OK)
  {
    Error_Handler();
  }
  if (HAL_UARTEx_SetTxFifoThreshold(&hlpuart1, UART_TXFIFO_THRESHOLD_1_8) != HAL_OK)
  {
    Error_Handler();
  }
  if (HAL_UARTEx_SetRxFifoThreshold(&hlpuart1, UART_RXFIFO_THRESHOLD_1_8) != HAL_OK)
  {
    Error_Handler();
  }
  if (HAL_UARTEx_DisableFifoMode(&hlpuart1) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN LPUART1_Init 2 */

  /* USER CODE END LPUART1_Init 2 */

}

/**
  * @brief TIM1 Initialization Function
  * @param None
  * @retval None
  */
static void MX_TIM1_Init(void)
{

  /* USER CODE BEGIN TIM1_Init 0 */

  /* USER CODE END TIM1_Init 0 */

  TIM_ClockConfigTypeDef sClockSourceConfig = {0};
  TIM_MasterConfigTypeDef sMasterConfig = {0};
  TIMEx_BreakInputConfigTypeDef sBreakInputConfig = {0};
  TIM_OC_InitTypeDef sConfigOC = {0};
  TIM_BreakDeadTimeConfigTypeDef sBreakDeadTimeConfig = {0};

  /* USER CODE BEGIN TIM1_Init 1 */

  /* USER CODE END TIM1_Init 1 */
  htim1.Instance = TIM1;
  htim1.Init.Prescaler = 0;
  htim1.Init.CounterMode = TIM_COUNTERMODE_CENTERALIGNED1;
  htim1.Init.Period = 4249;
  htim1.Init.ClockDivision = TIM_CLOCKDIVISION_DIV1;
  htim1.Init.RepetitionCounter = 0;
  htim1.Init.AutoReloadPreload = TIM_AUTORELOAD_PRELOAD_ENABLE;
  if (HAL_TIM_Base_Init(&htim1) != HAL_OK)
  {
    Error_Handler();
  }
  sClockSourceConfig.ClockSource = TIM_CLOCKSOURCE_INTERNAL;
  if (HAL_TIM_ConfigClockSource(&htim1, &sClockSourceConfig) != HAL_OK)
  {
    Error_Handler();
  }
  if (HAL_TIM_PWM_Init(&htim1) != HAL_OK)
  {
    Error_Handler();
  }
  if (HAL_TIM_OC_Init(&htim1) != HAL_OK)
  {
    Error_Handler();
  }
  sMasterConfig.MasterOutputTrigger = TIM_TRGO_RESET;
  sMasterConfig.MasterOutputTrigger2 = TIM_TRGO2_OC4REF_RISINGFALLING;
  sMasterConfig.MasterSlaveMode = TIM_MASTERSLAVEMODE_DISABLE;
  if (HAL_TIMEx_MasterConfigSynchronization(&htim1, &sMasterConfig) != HAL_OK)
  {
    Error_Handler();
  }
  sBreakInputConfig.Source = TIM_BREAKINPUTSOURCE_BKIN;
  sBreakInputConfig.Enable = TIM_BREAKINPUTSOURCE_ENABLE;
  sBreakInputConfig.Polarity = TIM_BREAKINPUTSOURCE_POLARITY_HIGH;
  if (HAL_TIMEx_ConfigBreakInput(&htim1, TIM_BREAKINPUT_BRK, &sBreakInputConfig) != HAL_OK)
  {
    Error_Handler();
  }
  sConfigOC.OCMode = TIM_OCMODE_PWM1;
  sConfigOC.Pulse = 0;
  sConfigOC.OCPolarity = TIM_OCPOLARITY_HIGH;
  sConfigOC.OCNPolarity = TIM_OCNPOLARITY_HIGH;
  sConfigOC.OCFastMode = TIM_OCFAST_DISABLE;
  sConfigOC.OCIdleState = TIM_OCIDLESTATE_RESET;
  sConfigOC.OCNIdleState = TIM_OCNIDLESTATE_RESET;
  if (HAL_TIM_PWM_ConfigChannel(&htim1, &sConfigOC, TIM_CHANNEL_1) != HAL_OK)
  {
    Error_Handler();
  }
  if (HAL_TIM_PWM_ConfigChannel(&htim1, &sConfigOC, TIM_CHANNEL_2) != HAL_OK)
  {
    Error_Handler();
  }
  if (HAL_TIM_PWM_ConfigChannel(&htim1, &sConfigOC, TIM_CHANNEL_3) != HAL_OK)
  {
    Error_Handler();
  }
  sConfigOC.OCMode = TIM_OCMODE_TIMING;
  sConfigOC.Pulse = 2125;
  if (HAL_TIM_OC_ConfigChannel(&htim1, &sConfigOC, TIM_CHANNEL_4) != HAL_OK)
  {
    Error_Handler();
  }
  sBreakDeadTimeConfig.OffStateRunMode = TIM_OSSR_DISABLE;
  sBreakDeadTimeConfig.OffStateIDLEMode = TIM_OSSI_DISABLE;
  sBreakDeadTimeConfig.LockLevel = TIM_LOCKLEVEL_OFF;
  sBreakDeadTimeConfig.DeadTime = 0;
  sBreakDeadTimeConfig.BreakState = TIM_BREAK_ENABLE;
  sBreakDeadTimeConfig.BreakPolarity = TIM_BREAKPOLARITY_HIGH;
  sBreakDeadTimeConfig.BreakFilter = 0;
  sBreakDeadTimeConfig.BreakAFMode = TIM_BREAK_AFMODE_INPUT;
  sBreakDeadTimeConfig.Break2State = TIM_BREAK2_DISABLE;
  sBreakDeadTimeConfig.Break2Polarity = TIM_BREAK2POLARITY_HIGH;
  sBreakDeadTimeConfig.Break2Filter = 0;
  sBreakDeadTimeConfig.Break2AFMode = TIM_BREAK_AFMODE_INPUT;
  sBreakDeadTimeConfig.AutomaticOutput = TIM_AUTOMATICOUTPUT_DISABLE;
  if (HAL_TIMEx_ConfigBreakDeadTime(&htim1, &sBreakDeadTimeConfig) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN TIM1_Init 2 */

  /* USER CODE END TIM1_Init 2 */
  HAL_TIM_MspPostInit(&htim1);

}

/**
  * @brief TIM6 Initialization Function
  * @param None
  * @retval None
  */
static void MX_TIM6_Init(void)
{

  /* USER CODE BEGIN TIM6_Init 0 */

  /* USER CODE END TIM6_Init 0 */

  TIM_MasterConfigTypeDef sMasterConfig = {0};

  /* USER CODE BEGIN TIM6_Init 1 */

  /* USER CODE END TIM6_Init 1 */
  htim6.Instance = TIM6;
  htim6.Init.Prescaler = 16999;
  htim6.Init.CounterMode = TIM_COUNTERMODE_UP;
  htim6.Init.Period = 9;
  htim6.Init.AutoReloadPreload = TIM_AUTORELOAD_PRELOAD_DISABLE;
  if (HAL_TIM_Base_Init(&htim6) != HAL_OK)
  {
    Error_Handler();
  }
  sMasterConfig.MasterOutputTrigger = TIM_TRGO_RESET;
  sMasterConfig.MasterSlaveMode = TIM_MASTERSLAVEMODE_DISABLE;
  if (HAL_TIMEx_MasterConfigSynchronization(&htim6, &sMasterConfig) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN TIM6_Init 2 */

  /* USER CODE END TIM6_Init 2 */

}

/**
  * @brief GPIO Initialization Function
  * @param None
  * @retval None
  */
static void MX_GPIO_Init(void)
{
  /* USER CODE BEGIN MX_GPIO_Init_1 */

  /* USER CODE END MX_GPIO_Init_1 */

  /* GPIO Ports Clock Enable */
  __HAL_RCC_GPIOC_CLK_ENABLE();
  __HAL_RCC_GPIOF_CLK_ENABLE();
  __HAL_RCC_GPIOA_CLK_ENABLE();
  __HAL_RCC_GPIOB_CLK_ENABLE();

  /* USER CODE BEGIN MX_GPIO_Init_2 */

  /* USER CODE END MX_GPIO_Init_2 */
}

/* USER CODE BEGIN 4 */

void HAL_ADCEx_InjectedConvCpltCallback(ADC_HandleTypeDef *hadc)
{
  if (hadc->Instance == ADC1)
  {
    g_adc1_current_raw =
        (uint16_t)HAL_ADCEx_InjectedGetValue(&hadc1,
                                             ADC_INJECTED_RANK_1);

    g_adc1_sample_done = 1U;
  }
  else if (hadc->Instance == ADC2)
  {
    g_adc2_current_raw =
        (uint16_t)HAL_ADCEx_InjectedGetValue(&hadc2,
                                             ADC_INJECTED_RANK_1);

    g_adc2_sample_done = 1U;
  }

  /*
   * Only report a complete sample when both ADCs
   * have finished the same trigger event.
   */
  if ((g_adc1_sample_done != 0U) &&
      (g_adc2_sample_done != 0U))
  {
    g_adc_sample_ready = 1U;

    g_adc1_sample_done = 0U;
    g_adc2_sample_done = 0U;
  }
}

void HAL_TIM_PeriodElapsedCallback(TIM_HandleTypeDef *htim)
{
  static uint16_t led_divider = 0;

  if (htim->Instance == TIM6)
  {
    g_tick_ms++;

    if (++led_divider >= 500U)
    {
      led_divider = 0;

      HAL_GPIO_TogglePin(GPIOA, GPIO_PIN_5);
    }
  }
}

void BSP_PB_Callback(Button_TypeDef Button)
{
  if (Button == BUTTON_USER)
  {
    if (g_motor_controller.state == MOTOR_STATE_RUNNING)
    {
      g_pending_motor_command = MOTOR_CMD_STOP;
    }
    else if (g_motor_controller.state == MOTOR_STATE_READY)
    {
      g_pending_motor_command = MOTOR_CMD_START;
    }
  }
}

/* USER CODE END 4 */

/**
  * @brief  This function is executed in case of error occurrence.
  * @retval None
  */
void Error_Handler(void)
{
  /* USER CODE BEGIN Error_Handler_Debug */
  /* User can add his own implementation to report the HAL error return state */
  __disable_irq();
  while (1)
  {
  }
  /* USER CODE END Error_Handler_Debug */
}
#ifdef USE_FULL_ASSERT
/**
  * @brief  Reports the name of the source file and the source line number
  *         where the assert_param error has occurred.
  * @param  file: pointer to the source file name
  * @param  line: assert_param error line source number
  * @retval None
  */
void assert_failed(uint8_t *file, uint32_t line)
{
  /* USER CODE BEGIN 6 */
  /* User can add his own implementation to report the file name and line number,
     ex: printf("Wrong parameters value: file %s on line %d\r\n", file, line) */
  /* USER CODE END 6 */
}
#endif /* USE_FULL_ASSERT */
