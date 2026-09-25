#include "current_sense.h"

static CurrentSenseConfig g_current_sense_config;

void current_sense_init(const CurrentSenseConfig *config)
{
    if (config == 0)
    {
        return;
    }

    g_current_sense_config = *config;
}

float current_sense_raw_to_amp(uint16_t raw,
                               uint16_t offset_raw,
                               float amp_per_count)
{
    if (amp_per_count == 0.0f)
    {
        return 0.0f;
    }

    return ((float)raw - (float)offset_raw) * amp_per_count;
}

static float current_sense_channel_a_to_amp(uint16_t raw)
{
    return current_sense_raw_to_amp(raw,
                                    g_current_sense_config.offset_raw_a,
                                    g_current_sense_config.amp_per_count_a);
}

static float current_sense_channel_b_to_amp(uint16_t raw)
{
    return current_sense_raw_to_amp(raw,
                                    g_current_sense_config.offset_raw_b,
                                    g_current_sense_config.amp_per_count_b);
}

PhaseCurrent current_sense_get_phase_current(uint16_t raw_a,
                                             uint16_t raw_b)
{
    PhaseCurrent current;

    current.phase_a = current_sense_channel_a_to_amp(raw_a);
    current.phase_b = current_sense_channel_b_to_amp(raw_b);
    current.phase_c = -(current.phase_a + current.phase_b);

    return current;
}
