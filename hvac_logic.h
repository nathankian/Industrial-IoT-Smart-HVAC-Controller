#ifndef HVAC_LOGIC_H
#define HVAC_LOGIC_H

#include "hvac_types.h"

float calibrateAdcToPressurePercent(uint16_t raw);
float lowPassFilter(float previousValue, float newValue, float alpha);
bool isSensorAnomalous(const SensorData &sensor, char *message, size_t messageSize);
HvacMode decideHvacMode(const SensorData &sensor, bool faultActive, bool emergencyOverride);
const char *modeToString(HvacMode mode);

#endif