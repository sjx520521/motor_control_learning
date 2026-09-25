#ifndef BUS_VOLTAGE_H
#define BUS_VOLTAGE_H

#include <stdint.h>

typedef struct
{
    float adc_reference_voltage;
    float divider_ratio;
    float filter_alpha;
} BusVoltageConfig;

void bus_voltage_init(const BusVoltageConfig *config);

float bus_voltage_raw_to_volts(uint16_t raw);

void bus_voltage_update_raw(uint16_t raw);

void bus_voltage_set_simulated(float voltage);

float bus_voltage_get(void);

#endif
