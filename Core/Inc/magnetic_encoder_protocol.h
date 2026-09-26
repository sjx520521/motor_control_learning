#ifndef MAGNETIC_ENCODER_PROTOCOL_H
#define MAGNETIC_ENCODER_PROTOCOL_H

#include <stdint.h>

typedef struct
{
    uint32_t crc_error_count;
    uint32_t status_warning_count;
    uint8_t last_status;
} MagneticEncoderDiagnostics;

int kth7112_decode_spi_angle(void *context,
                             const uint8_t *frame,
                             uint16_t length,
                             float *angle_radians);

int ktm5910_decode_spi_angle(void *context,
                             const uint8_t *frame,
                             uint16_t length,
                             float *angle_radians);

#endif
