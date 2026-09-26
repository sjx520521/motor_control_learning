#include "magnetic_encoder_protocol.h"

#define MAG_ENCODER_TWO_PI (6.28318530718f)
#define KTH7112_SPI_FRAME_LENGTH (4U)
#define KTM5910_SPI_FRAME_LENGTH (5U)

static uint8_t crc8_update(uint8_t crc, uint8_t data)
{
    uint8_t bit;

    crc ^= data;

    for (bit = 0U; bit < 8U; ++bit)
    {
        if ((crc & 0x80U) != 0U)
        {
            crc = (uint8_t)((crc << 1U) ^ 0x07U);
        }
        else
        {
            crc <<= 1U;
        }
    }

    return crc;
}

static uint8_t crc8_calculate(const uint8_t *data,
                              uint16_t length,
                              uint8_t result_xor)
{
    uint16_t index;
    uint8_t crc = 0U;

    for (index = 0U; index < length; ++index)
    {
        crc = crc8_update(crc, data[index]);
    }

    return (uint8_t)(crc ^ result_xor);
}

int kth7112_decode_spi_angle(void *context,
                             const uint8_t *frame,
                             uint16_t length,
                             float *angle_radians)
{
    MagneticEncoderDiagnostics *diagnostics =
        (MagneticEncoderDiagnostics *)context;
    uint16_t raw_angle;
    uint8_t expected_crc;

    if (frame == 0 ||
        angle_radians == 0 ||
        length < KTH7112_SPI_FRAME_LENGTH)
    {
        return -1;
    }

    expected_crc = crc8_calculate(&frame[1], 2U, 0x55U);
    if (expected_crc != frame[3])
    {
        if (diagnostics != 0)
        {
            diagnostics->crc_error_count++;
        }
        return -1;
    }

    raw_angle = ((uint16_t)frame[1] << 8U) | frame[2];
    *angle_radians =
        ((float)raw_angle / 65536.0f) * MAG_ENCODER_TWO_PI;

    return 0;
}

int ktm5910_decode_spi_angle(void *context,
                             const uint8_t *frame,
                             uint16_t length,
                             float *angle_radians)
{
    MagneticEncoderDiagnostics *diagnostics =
        (MagneticEncoderDiagnostics *)context;
    uint32_t payload;
    uint32_t raw_angle;
    uint8_t status;
    uint8_t expected_crc;

    if (frame == 0 ||
        angle_radians == 0 ||
        length < KTM5910_SPI_FRAME_LENGTH)
    {
        return -1;
    }

    expected_crc = crc8_calculate(frame, 4U, 0xFFU);
    if (expected_crc != frame[4])
    {
        if (diagnostics != 0)
        {
            diagnostics->crc_error_count++;
        }
        return -1;
    }

    payload = ((uint32_t)frame[0] << 24U) |
              ((uint32_t)frame[1] << 16U) |
              ((uint32_t)frame[2] << 8U) |
              frame[3];
    status = (uint8_t)(payload & 0x03U);
    raw_angle = (payload >> 2U) & 0x00FFFFFFUL;

    if (diagnostics != 0)
    {
        diagnostics->last_status = status;
        if (status != 0U)
        {
            diagnostics->status_warning_count++;
        }
    }

    *angle_radians =
        ((float)raw_angle / 16777216.0f) * MAG_ENCODER_TWO_PI;

    return (status == 0U) ? 0 : -1;
}
