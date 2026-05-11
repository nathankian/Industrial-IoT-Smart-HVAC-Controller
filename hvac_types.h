#ifndef HVAC_TYPES_H
#define HVAC_TYPES_H

#include <Arduino.h>

// Industrial HVAC limits used by the control and safety logic
#define TEMP_SHUTDOWN_C          35.0f
#define TEMP_COOL_MARGIN_C        1.0f
#define TEMP_HEAT_MARGIN_C        1.0f
#define MIN_AIRFLOW_PERCENT       5.0f
#define SENSOR_STALE_LIMIT_MS  6000UL
#define DEFAULT_SETPOINT_C        24.0f

typedef enum {
  MODE_IDLE = 0,
  MODE_HEAT,
  MODE_COOL,
  MODE_VENT,
  MODE_FAULT
} HvacMode;

typedef struct {
  float temperatureC;
  float humidityPct;
  uint16_t adcRaw;
  float pressurePct;
  float pressureSmoothPct;
  float setPointC;
  bool dhtValid;
  uint32_t lastSensorUpdateMs;
  uint32_t sampleCounter;
} SensorData;

typedef struct {
  HvacMode mode;
  bool fanOn;
  bool heaterOn;
  bool coolerOn;
  bool faultActive;
  bool emergencyOverride;
  bool sensorAnomaly;
  bool cyberLocked;
  char faultMessage[96];
  uint32_t lastControlMs;
  uint32_t rejectedCommands;
  uint32_t acceptedCommands;
} ControlState;

#endif