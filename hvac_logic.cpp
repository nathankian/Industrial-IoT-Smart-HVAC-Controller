#include "hvac_logic.h"
#include <math.h>
#include <string.h>

float calibrateAdcToPressurePercent(uint16_t raw) {
  // ESP32 ADC is configured as 12-bit, so raw range is 0-4095.
  // The potentiometer simulates an industrial 0.3V-3.0V pressure/airflow sensor.
  const float voltage = (raw * 3.3f) / 4095.0f;
  float pressurePercent = ((voltage - 0.3f) / (3.0f - 0.3f)) * 100.0f;

  if (pressurePercent < 0.0f) {
    pressurePercent = 0.0f;
  }

  if (pressurePercent > 100.0f) {
    pressurePercent = 100.0f;
  }

  return pressurePercent;
}

float lowPassFilter(float previousValue, float newValue, float alpha) {
  if (alpha < 0.0f) {
    alpha = 0.0f;
  }

  if (alpha > 1.0f) {
    alpha = 1.0f;
  }

  return previousValue + alpha * (newValue - previousValue);
}

bool isSensorAnomalous(const SensorData &sensor, char *message, size_t messageSize) {
  if (!sensor.dhtValid || isnan(sensor.temperatureC) || isnan(sensor.humidityPct)) {
    snprintf(message, messageSize, "DHT22 sensor read failure");
    return true;
  }

  if (sensor.temperatureC < -10.0f || sensor.temperatureC > 80.0f) {
    snprintf(message, messageSize, "Temperature outside safe sensor range");
    return true;
  }

  if (sensor.humidityPct < 0.0f || sensor.humidityPct > 100.0f) {
    snprintf(message, messageSize, "Humidity outside valid range");
    return true;
  }

  if (sensor.pressureSmoothPct < MIN_AIRFLOW_PERCENT) {
    snprintf(message, messageSize, "Airflow/pressure too low");
    return true;
  }

  snprintf(message, messageSize, "OK");
  return false;
}

HvacMode decideHvacMode(const SensorData &sensor, bool faultActive, bool emergencyOverride) {
  if (faultActive || emergencyOverride) {
    return MODE_FAULT;
  }

  if (sensor.temperatureC >= sensor.setPointC + TEMP_COOL_MARGIN_C) {
    return MODE_COOL;
  }

  if (sensor.temperatureC <= sensor.setPointC - TEMP_HEAT_MARGIN_C) {
    return MODE_HEAT;
  }

  if (sensor.humidityPct >= 70.0f) {
    return MODE_VENT;
  }

  return MODE_IDLE;
}

const char *modeToString(HvacMode mode) {
  switch (mode) {
    case MODE_HEAT:
      return "HEAT";

    case MODE_COOL:
      return "COOL";

    case MODE_VENT:
      return "VENT";

    case MODE_FAULT:
      return "FAULT";

    case MODE_IDLE:
    default:
      return "IDLE";
  }
}