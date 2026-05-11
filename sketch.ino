#include <Arduino.h>
#include <DHTesp.h>
#include <Wire.h>
#include <LiquidCrystal_I2C.h>
#include <stdarg.h>
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include "freertos/semphr.h"
#include "hvac_types.h"
#include "hvac_logic.h"

// -------------------- Pin map --------------------
const int PIN_DHT22       = 15;
const int PIN_PRESSURE    = 34;   // Potentiometer simulates airflow/pressure sensor
const int PIN_COOL_RELAY  = 26;   // Blue LED
const int PIN_HEAT_RELAY  = 27;   // Yellow LED
const int PIN_FAN_RELAY   = 25;   // Green LED
const int PIN_FAULT_LED   = 23;   // Red LED
const int PIN_BUZZER      = 19;
const int PIN_EMERGENCY   = 4;    // Emergency button
const int PIN_RESET       = 18;   // Reset button

// -------------------- LCD screen --------------------
LiquidCrystal_I2C lcd(0x27, 16, 2);

// -------------------- Cyber / command settings --------------------
const char DEVICE_TOKEN[] = "HVAC2026";
const uint32_t COMMAND_RATE_LIMIT_MS = 1000UL;

// -------------------- FreeRTOS shared resources --------------------
SemaphoreHandle_t dataMutex;
SemaphoreHandle_t serialMutex;
TaskHandle_t inputTaskHandle = NULL;

DHTesp dhtSensor;
SensorData gSensorData;
ControlState gControlState;

const uint32_t EMERGENCY_BIT = 0x01;
const uint32_t RESET_BIT     = 0x02;
const uint32_t DEBOUNCE_MS   = 180UL;

// -------------------- Interrupt service routines --------------------
void IRAM_ATTR emergencyISR() {
  if (inputTaskHandle != NULL) {
    BaseType_t higherPriorityTaskWoken = pdFALSE;
    xTaskNotifyFromISR(inputTaskHandle, EMERGENCY_BIT, eSetBits, &higherPriorityTaskWoken);

    if (higherPriorityTaskWoken) {
      portYIELD_FROM_ISR();
    }
  }
}

void IRAM_ATTR resetISR() {
  if (inputTaskHandle != NULL) {
    BaseType_t higherPriorityTaskWoken = pdFALSE;
    xTaskNotifyFromISR(inputTaskHandle, RESET_BIT, eSetBits, &higherPriorityTaskWoken);

    if (higherPriorityTaskWoken) {
      portYIELD_FROM_ISR();
    }
  }
}

// -------------------- Safe Serial Printing --------------------
void safeSerialPrintf(const char *format, ...) {
  if (xSemaphoreTake(serialMutex, pdMS_TO_TICKS(100)) == pdTRUE) {
    char buffer[300];

    va_list args;
    va_start(args, format);
    vsnprintf(buffer, sizeof(buffer), format, args);
    va_end(args);

    Serial.print(buffer);
    xSemaphoreGive(serialMutex);
  }
}

// -------------------- LCD mode text --------------------
const char* getDisplayModeText(HvacMode mode) {
  switch (mode) {
    case MODE_HEAT:
      return "HEATING";

    case MODE_COOL:
      return "COOLING";

    case MODE_VENT:
      return "VENT";

    case MODE_FAULT:
      return "FAULT";

    case MODE_IDLE:
    default:
      return "IDLE";
  }
}

// -------------------- Output Control --------------------
void setOutputs(bool heatOn, bool coolOn, bool fanOn, bool faultLedOn, bool buzzerOn) {
  digitalWrite(PIN_HEAT_RELAY, heatOn ? HIGH : LOW);
  digitalWrite(PIN_COOL_RELAY, coolOn ? HIGH : LOW);
  digitalWrite(PIN_FAN_RELAY, fanOn ? HIGH : LOW);
  digitalWrite(PIN_FAULT_LED, faultLedOn ? HIGH : LOW);

  if (buzzerOn) {
    tone(PIN_BUZZER, 1800);
  } else {
    noTone(PIN_BUZZER);
  }
}

// -------------------- Task 1: Sensor + ADC acquisition --------------------
void taskSensorRead(void *pvParameters) {
  TickType_t lastWakeTime = xTaskGetTickCount();

  for (;;) {
    TempAndHumidity reading = dhtSensor.getTempAndHumidity();

    uint16_t rawAdc = analogRead(PIN_PRESSURE);
    float pressurePct = calibrateAdcToPressurePercent(rawAdc);

    bool validDht = !isnan(reading.temperature) && !isnan(reading.humidity);

    if (xSemaphoreTake(dataMutex, pdMS_TO_TICKS(200)) == pdTRUE) {
      gSensorData.temperatureC = reading.temperature;
      gSensorData.humidityPct = reading.humidity;
      gSensorData.adcRaw = rawAdc;
      gSensorData.pressurePct = pressurePct;

      if (gSensorData.sampleCounter == 0) {
        gSensorData.pressureSmoothPct = pressurePct;
      } else {
        gSensorData.pressureSmoothPct = lowPassFilter(gSensorData.pressureSmoothPct, pressurePct, 0.20f);
      }

      gSensorData.dhtValid = validDht;
      gSensorData.lastSensorUpdateMs = millis();
      gSensorData.sampleCounter++;

      xSemaphoreGive(dataMutex);
    }

    vTaskDelayUntil(&lastWakeTime, pdMS_TO_TICKS(2000));
  }
}

// -------------------- Task 2: HVAC control and fail-safe logic --------------------
void taskControl(void *pvParameters) {
  TickType_t lastWakeTime = xTaskGetTickCount();

  for (;;) {
    SensorData sensorCopy;
    ControlState stateCopy;

    if (xSemaphoreTake(dataMutex, pdMS_TO_TICKS(200)) == pdTRUE) {
      sensorCopy = gSensorData;
      stateCopy = gControlState;
      xSemaphoreGive(dataMutex);
    }

    char safetyMessage[96] = "OK";
    bool fault = false;
    bool anomaly = false;

    if (sensorCopy.lastSensorUpdateMs == 0) {
      snprintf(safetyMessage, sizeof(safetyMessage), "Waiting for first sensor sample");
    }
    else if (millis() - sensorCopy.lastSensorUpdateMs > SENSOR_STALE_LIMIT_MS) {
      fault = true;
      anomaly = true;
      snprintf(safetyMessage, sizeof(safetyMessage), "Software watchdog: sensor data stale");
    }
    else if (stateCopy.emergencyOverride) {
      fault = true;
      snprintf(safetyMessage, sizeof(safetyMessage), "Emergency override active");
    }
    else if (sensorCopy.temperatureC >= TEMP_SHUTDOWN_C) {
      fault = true;
      snprintf(safetyMessage, sizeof(safetyMessage), "Overheat shutdown: %.1fC", sensorCopy.temperatureC);
    }
    else {
      anomaly = isSensorAnomalous(sensorCopy, safetyMessage, sizeof(safetyMessage));
      fault = anomaly;
    }

    HvacMode selectedMode = decideHvacMode(sensorCopy, fault, stateCopy.emergencyOverride);

    bool heater = false;
    bool cooler = false;
    bool fan = false;
    bool buzzer = false;
    bool faultLed = false;

    switch (selectedMode) {
      case MODE_HEAT:
        heater = true;
        fan = true;
        break;

      case MODE_COOL:
        cooler = true;
        fan = true;
        break;

      case MODE_VENT:
        fan = true;
        break;

      case MODE_FAULT:
        faultLed = true;
        buzzer = true;
        heater = false;
        cooler = false;
        fan = false;
        break;

      case MODE_IDLE:
      default:
        break;
    }

    setOutputs(heater, cooler, fan, faultLed, buzzer);

    if (xSemaphoreTake(dataMutex, pdMS_TO_TICKS(200)) == pdTRUE) {
      gControlState.mode = selectedMode;
      gControlState.heaterOn = heater;
      gControlState.coolerOn = cooler;
      gControlState.fanOn = fan;
      gControlState.faultActive = fault;
      gControlState.sensorAnomaly = anomaly;
      gControlState.lastControlMs = millis();

      snprintf(gControlState.faultMessage, sizeof(gControlState.faultMessage), "%s", safetyMessage);

      xSemaphoreGive(dataMutex);
    }

    vTaskDelayUntil(&lastWakeTime, pdMS_TO_TICKS(500));
  }
}

// -------------------- Task 3: Interrupt input handling with debounce --------------------
void taskInputs(void *pvParameters) {
  uint32_t notificationValue = 0;
  uint32_t lastEmergencyHandledMs = 0;
  uint32_t lastResetHandledMs = 0;

  for (;;) {
    if (xTaskNotifyWait(0, 0xFFFFFFFF, &notificationValue, portMAX_DELAY) == pdTRUE) {
      uint32_t nowMs = millis();

      if ((notificationValue & EMERGENCY_BIT) && (nowMs - lastEmergencyHandledMs > DEBOUNCE_MS)) {
        if (digitalRead(PIN_EMERGENCY) == LOW) {
          lastEmergencyHandledMs = nowMs;

          if (xSemaphoreTake(dataMutex, pdMS_TO_TICKS(200)) == pdTRUE) {
            // Emergency button now only activates the override.
            // The Reset button is responsible for clearing it.
            gControlState.emergencyOverride = true;
            gControlState.faultActive = true;

            snprintf(
              gControlState.faultMessage,
              sizeof(gControlState.faultMessage),
              "Emergency override enabled"
            );

            xSemaphoreGive(dataMutex);
          }

          safeSerialPrintf("\n[INPUT] Emergency button pressed: override enabled.\n");
        }
      }

      if ((notificationValue & RESET_BIT) && (nowMs - lastResetHandledMs > DEBOUNCE_MS)) {
        if (digitalRead(PIN_RESET) == LOW) {
          lastResetHandledMs = nowMs;

          if (xSemaphoreTake(dataMutex, pdMS_TO_TICKS(200)) == pdTRUE) {
            // Manual reset clears the emergency override and any latched fault flags.
            // If a real sensor fault is still present, taskControl will safely re-enter FAULT mode.
            gControlState.emergencyOverride = false;
            gControlState.faultActive = false;
            gControlState.sensorAnomaly = false;
            gControlState.cyberLocked = false;

            snprintf(gControlState.faultMessage, sizeof(gControlState.faultMessage), "Manual reset completed");

            xSemaphoreGive(dataMutex);
          }

          safeSerialPrintf("\n[INPUT] Reset button pressed: emergency override cleared.\n");
        }
      }
    }
  }
}

// -------------------- Task 4: Cybersecurity and cloud command handling --------------------
void handleCloudCommand(String command) {
  command.trim();

  if (command.length() == 0) {
    return;
  }

  static uint32_t lastCommandMs = 0;
  uint32_t nowMs = millis();

  bool reject = false;
  char reason[96] = "OK";

  if (nowMs - lastCommandMs < COMMAND_RATE_LIMIT_MS) {
    reject = true;
    snprintf(reason, sizeof(reason), "Rate limit protection triggered");
  }
  else if (command.length() > 80) {
    reject = true;
    snprintf(reason, sizeof(reason), "Command length rejected");
  }
  else if (command.indexOf("AUTH=HVAC2026") < 0) {
    reject = true;
    snprintf(reason, sizeof(reason), "Invalid authentication token");
  }

  lastCommandMs = nowMs;

  if (reject) {
    if (xSemaphoreTake(dataMutex, pdMS_TO_TICKS(200)) == pdTRUE) {
      gControlState.rejectedCommands++;
      gControlState.cyberLocked = true;
      snprintf(gControlState.faultMessage, sizeof(gControlState.faultMessage), "%s", reason);
      xSemaphoreGive(dataMutex);
    }

    safeSerialPrintf("\n[CYBER SECURITY] Command rejected.\n");
    safeSerialPrintf("Command: %s\n", command.c_str());
    safeSerialPrintf("Reason:  %s\n", reason);
    return;
  }

  if (command.indexOf("SET=") >= 0) {
    int setIndex = command.indexOf("SET=");
    float requestedSetPoint = command.substring(setIndex + 4).toFloat();

    if (requestedSetPoint < 16.0f || requestedSetPoint > 30.0f) {
      if (xSemaphoreTake(dataMutex, pdMS_TO_TICKS(200)) == pdTRUE) {
        gControlState.rejectedCommands++;
        snprintf(gControlState.faultMessage, sizeof(gControlState.faultMessage), "Rejected unsafe setpoint");
        xSemaphoreGive(dataMutex);
      }

      safeSerialPrintf("\n[CYBER SECURITY] Unsafe setpoint rejected.\n");
      safeSerialPrintf("Requested setpoint: %.1f C\n", requestedSetPoint);
      safeSerialPrintf("Allowed range: 16.0 C to 30.0 C\n");
      return;
    }

    if (xSemaphoreTake(dataMutex, pdMS_TO_TICKS(200)) == pdTRUE) {
      gSensorData.setPointC = requestedSetPoint;
      gControlState.acceptedCommands++;
      gControlState.cyberLocked = false;

      snprintf(gControlState.faultMessage, sizeof(gControlState.faultMessage), "Cloud setpoint accepted");

      xSemaphoreGive(dataMutex);
    }

    safeSerialPrintf("\n[CLOUD COMMAND] Authenticated setpoint update accepted.\n");
    safeSerialPrintf("New target setpoint: %.1f C\n", requestedSetPoint);
  }
  else if (command.indexOf("RESET") >= 0) {
    if (xSemaphoreTake(dataMutex, pdMS_TO_TICKS(200)) == pdTRUE) {
      gControlState.emergencyOverride = false;
      gControlState.faultActive = false;
      gControlState.sensorAnomaly = false;
      gControlState.cyberLocked = false;
      gControlState.acceptedCommands++;

      snprintf(gControlState.faultMessage, sizeof(gControlState.faultMessage), "Cloud reset accepted");

      xSemaphoreGive(dataMutex);
    }

    safeSerialPrintf("\n[CLOUD COMMAND] Reset command processed.\n");
  }
  else {
    if (xSemaphoreTake(dataMutex, pdMS_TO_TICKS(200)) == pdTRUE) {
      gControlState.rejectedCommands++;
      snprintf(gControlState.faultMessage, sizeof(gControlState.faultMessage), "Unknown command rejected");
      xSemaphoreGive(dataMutex);
    }

    safeSerialPrintf("\n[CYBER SECURITY] Unknown authenticated command rejected.\n");
    safeSerialPrintf("Command: %s\n", command.c_str());
  }
}

void taskCyberCloud(void *pvParameters) {
  Serial.setTimeout(20);

  for (;;) {
    if (Serial.available() > 0) {
      String command = Serial.readStringUntil('\n');
      handleCloudCommand(command);
    }

    vTaskDelay(pdMS_TO_TICKS(200));
  }
}

// -------------------- Task 5: LCD Display --------------------
void taskDisplay(void *pvParameters) {
  TickType_t lastWakeTime = xTaskGetTickCount();

  for (;;) {
    SensorData sensorCopy;
    ControlState stateCopy;

    if (xSemaphoreTake(dataMutex, pdMS_TO_TICKS(200)) == pdTRUE) {
      sensorCopy = gSensorData;
      stateCopy = gControlState;
      xSemaphoreGive(dataMutex);
    }

    lcd.clear();

    lcd.setCursor(0, 0);
    lcd.print("Mode:");
    lcd.print(getDisplayModeText(stateCopy.mode));

    lcd.setCursor(0, 1);

    if (stateCopy.emergencyOverride) {
      lcd.print("Emergency ON");
    }
    else if (stateCopy.faultActive) {
      lcd.print("FAULT ACTIVE");
    }
    else {
      lcd.print("System Normal");
    }

    vTaskDelayUntil(&lastWakeTime, pdMS_TO_TICKS(1000));
  }
}

// -------------------- Task 6: Readable serial diagnostic logging --------------------
void taskDiagnostics(void *pvParameters) {
  TickType_t lastWakeTime = xTaskGetTickCount();

  for (;;) {
    SensorData sensorCopy;
    ControlState stateCopy;

    if (xSemaphoreTake(dataMutex, pdMS_TO_TICKS(200)) == pdTRUE) {
      sensorCopy = gSensorData;
      stateCopy = gControlState;
      xSemaphoreGive(dataMutex);
    }

    safeSerialPrintf("\n========================================\n");
    safeSerialPrintf("        SMART HVAC SYSTEM STATUS\n");
    safeSerialPrintf("========================================\n");

    safeSerialPrintf("Temperature:        %.1f C\n", sensorCopy.temperatureC);
    safeSerialPrintf("Humidity:           %.1f %%\n", sensorCopy.humidityPct);
    safeSerialPrintf("Airflow/Pressure:   %.1f %%\n", sensorCopy.pressureSmoothPct);
    safeSerialPrintf("Raw ADC Reading:    %u / 4095\n", sensorCopy.adcRaw);
    safeSerialPrintf("Target Setpoint:    %.1f C\n", sensorCopy.setPointC);

    safeSerialPrintf("----------------------------------------\n");
    safeSerialPrintf("Current Mode:       %s\n", modeToString(stateCopy.mode));

    safeSerialPrintf("Fan:                %s\n", stateCopy.fanOn ? "ON" : "OFF");
    safeSerialPrintf("Heating:            %s\n", stateCopy.heaterOn ? "ON" : "OFF");
    safeSerialPrintf("Cooling:            %s\n", stateCopy.coolerOn ? "ON" : "OFF");

    safeSerialPrintf("----------------------------------------\n");

    safeSerialPrintf("Fault Status:       %s\n", stateCopy.faultActive ? "ACTIVE" : "NORMAL");
    safeSerialPrintf("Emergency Button:   %s\n", stateCopy.emergencyOverride ? "ACTIVE" : "NOT ACTIVE");
    safeSerialPrintf("System Message:     %s\n", stateCopy.faultMessage);

    safeSerialPrintf("----------------------------------------\n");
    safeSerialPrintf("Accepted Commands:  %lu\n", (unsigned long)stateCopy.acceptedCommands);
    safeSerialPrintf("Rejected Commands:  %lu\n", (unsigned long)stateCopy.rejectedCommands);
    safeSerialPrintf("========================================\n");

    vTaskDelayUntil(&lastWakeTime, pdMS_TO_TICKS(3000));
  }
}

// -------------------- Setup --------------------
void setup() {
  Serial.begin(115200);
  delay(500);

  pinMode(PIN_COOL_RELAY, OUTPUT);
  pinMode(PIN_HEAT_RELAY, OUTPUT);
  pinMode(PIN_FAN_RELAY, OUTPUT);
  pinMode(PIN_FAULT_LED, OUTPUT);
  pinMode(PIN_BUZZER, OUTPUT);

  pinMode(PIN_EMERGENCY, INPUT_PULLUP);
  pinMode(PIN_RESET, INPUT_PULLUP);

  setOutputs(false, false, false, false, false);

  analogReadResolution(12);
  analogSetPinAttenuation(PIN_PRESSURE, ADC_11db);

  dhtSensor.setup(PIN_DHT22, DHTesp::DHT22);

  Wire.begin(21, 22);
  lcd.init();
  lcd.backlight();
  lcd.clear();
  lcd.setCursor(0, 0);
  lcd.print("Smart HVAC");
  lcd.setCursor(0, 1);
  lcd.print("Starting...");

  memset(&gSensorData, 0, sizeof(gSensorData));
  gSensorData.setPointC = DEFAULT_SETPOINT_C;
  gSensorData.pressureSmoothPct = 50.0f;

  memset(&gControlState, 0, sizeof(gControlState));
  gControlState.mode = MODE_IDLE;

  snprintf(gControlState.faultMessage, sizeof(gControlState.faultMessage), "System booting");

  dataMutex = xSemaphoreCreateMutex();
  serialMutex = xSemaphoreCreateMutex();

  if (dataMutex == NULL || serialMutex == NULL) {
    Serial.println("[BOOT ERROR] Mutex creation failed. System stopped.");

    while (true) {
      delay(1000);
    }
  }

  xTaskCreatePinnedToCore(taskInputs,      "InputIRQ",    4096, NULL, 5, &inputTaskHandle, 1);
  xTaskCreatePinnedToCore(taskControl,     "Control",     4096, NULL, 4, NULL, 1);
  xTaskCreatePinnedToCore(taskSensorRead,  "SensorADC",   4096, NULL, 3, NULL, 1);
  xTaskCreatePinnedToCore(taskCyberCloud,  "CyberCloud",  4096, NULL, 2, NULL, 1);
  xTaskCreatePinnedToCore(taskDisplay,     "LCDDisplay",  4096, NULL, 2, NULL, 1);
  xTaskCreatePinnedToCore(taskDiagnostics, "Diagnostics", 4096, NULL, 1, NULL, 1);

  attachInterrupt(digitalPinToInterrupt(PIN_EMERGENCY), emergencyISR, FALLING);
  attachInterrupt(digitalPinToInterrupt(PIN_RESET), resetISR, FALLING);

  safeSerialPrintf("\n[BOOT] Industrial IoT Smart HVAC Controller started.\n");
  safeSerialPrintf("[BOOT] Valid command example: AUTH=HVAC2026;SET=23.5\n");
  safeSerialPrintf("[BOOT] Reset command example: AUTH=HVAC2026;RESET\n");
}

// -------------------- Main loop --------------------
void loop() {
  vTaskDelay(pdMS_TO_TICKS(1000));
}