#ifndef CURRENT_SENSE_H
#define CURRENT_SENSE_H

#include <stdint.h>

typedef struct
{
    uint16_t offset_raw_a;
    uint16_t offset_raw_b;
    float amp_per_count_a;
    float amp_per_count_b;
} CurrentSenseConfig;

typedef struct
{
    float phase_a;
    float phase_b;
    float phase_c;
} PhaseCurrent;

void current_sense_init(const CurrentSenseConfig *config);

float current_sense_raw_to_amp(uint16_t raw,
                               uint16_t offset_raw,
                               float amp_per_count);

PhaseCurrent current_sense_get_phase_current(uint16_t raw_a,
                                             uint16_t raw_b);

#endif
