#include "dual_encoder_spi.h"

#include <string.h>

int dual_encoder_spi_init(DualEncoderSpiController *controller,
                          const DualEncoderConfig *config,
                          const EncoderSpiDevice *motor_encoder,
                          const EncoderSpiDevice *joint_encoder)
{
    float motor_angle;
    float joint_angle;

    if (controller == 0 ||
        config == 0 ||
        motor_encoder == 0 ||
        joint_encoder == 0)
    {
        return ENCODER_SPI_INVALID_ARGUMENT;
    }

    memset(controller, 0, sizeof(*controller));
    controller->motor_encoder = *motor_encoder;
    controller->joint_encoder = *joint_encoder;

    controller->motor_status =
        encoder_spi_device_read_angle(&controller->motor_encoder,
                                      &motor_angle);
    controller->joint_status =
        encoder_spi_device_read_angle(&controller->joint_encoder,
                                      &joint_angle);

    if (controller->motor_status != ENCODER_SPI_OK ||
        controller->joint_status != ENCODER_SPI_OK)
    {
        controller->read_error_count++;
        return ENCODER_SPI_TRANSFER_ERROR;
    }

    dual_encoder_init(&controller->fusion,
                      config,
                      motor_angle,
                      joint_angle);
    controller->initialized = 1U;

    return ENCODER_SPI_OK;
}

int dual_encoder_spi_update(DualEncoderSpiController *controller,
                            float dt_seconds)
{
    float motor_angle;
    float joint_angle;

    if (controller == 0 ||
        controller->initialized == 0U)
    {
        return ENCODER_SPI_NOT_READY;
    }

    controller->motor_status =
        encoder_spi_device_read_angle(&controller->motor_encoder,
                                      &motor_angle);
    controller->joint_status =
        encoder_spi_device_read_angle(&controller->joint_encoder,
                                      &joint_angle);

    if (controller->motor_status != ENCODER_SPI_OK ||
        controller->joint_status != ENCODER_SPI_OK)
    {
        controller->read_error_count++;
        return ENCODER_SPI_TRANSFER_ERROR;
    }

    if (dual_encoder_update(&controller->fusion,
                            motor_angle,
                            joint_angle,
                            dt_seconds) == 0U)
    {
        return ENCODER_SPI_DECODE_ERROR;
    }

    return ENCODER_SPI_OK;
}

void dual_encoder_spi_reset(DualEncoderSpiController *controller)
{
    float motor_angle;
    float joint_angle;

    if (controller == 0 ||
        controller->initialized == 0U)
    {
        return;
    }

    controller->motor_status =
        encoder_spi_device_read_angle(&controller->motor_encoder,
                                      &motor_angle);
    controller->joint_status =
        encoder_spi_device_read_angle(&controller->joint_encoder,
                                      &joint_angle);

    if (controller->motor_status == ENCODER_SPI_OK &&
        controller->joint_status == ENCODER_SPI_OK)
    {
        dual_encoder_reset(&controller->fusion,
                           motor_angle,
                           joint_angle);
    }
}

const DualEncoderState *dual_encoder_spi_get_state(
    const DualEncoderSpiController *controller)
{
    if (controller == 0 ||
        controller->initialized == 0U)
    {
        return 0;
    }

    return dual_encoder_get_state(&controller->fusion);
}
