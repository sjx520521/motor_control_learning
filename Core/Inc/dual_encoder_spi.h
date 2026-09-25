#ifndef DUAL_ENCODER_SPI_H
#define DUAL_ENCODER_SPI_H

#include <stdint.h>

#include "dual_encoder.h"
#include "encoder_spi.h"

typedef struct
{
    DualEncoderController fusion;
    EncoderSpiDevice motor_encoder;
    EncoderSpiDevice joint_encoder;
    int motor_status;
    int joint_status;
    uint32_t read_error_count;
    uint8_t initialized;
} DualEncoderSpiController;

int dual_encoder_spi_init(DualEncoderSpiController *controller,
                          const DualEncoderConfig *config,
                          const EncoderSpiDevice *motor_encoder,
                          const EncoderSpiDevice *joint_encoder);

int dual_encoder_spi_update(DualEncoderSpiController *controller,
                            float dt_seconds);

void dual_encoder_spi_reset(DualEncoderSpiController *controller);

const DualEncoderState *dual_encoder_spi_get_state(
    const DualEncoderSpiController *controller);

#endif
