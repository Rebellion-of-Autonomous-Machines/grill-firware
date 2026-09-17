#include <WiFi.h>
#include <SPI.h>
#include <Wire.h>
#include <WebServer.h>
#include <DNSServer.h>
#include <Preferences.h>
#include <ModbusRTU.h>
#include <Adafruit_MAX31865.h>
#include <math.h>

// --- SPI MAX31865 ---
static const uint8_t MAX31865_CS_PIN = 5;
static const uint8_t MAX31865_SCK_PIN = 18;
static const uint8_t MAX31865_MISO_PIN = 19;
static const uint8_t MAX31865_MOSI_PIN = 23;
static const float MAX31865_RTD_NOMINAL = 100.0f;  // PT100
static const float MAX31865_REF_RESISTOR = 430.0f; // Typical MAX31865 PT100 board
static const max31865_numwires_t MAX31865_WIRES = MAX31865_2WIRE;
Adafruit_MAX31865 rtd = Adafruit_MAX31865(MAX31865_CS_PIN);

// --- Analog thermocouple amplifier AD8495 ---
static const uint8_t ANALOG_A1 = 36;
static const uint8_t AD8495_SAMPLE_COUNT = 16;
static const float AD8495_ZERO_C_MV = 790.0f; // Calibrated for this 3.3 V AD8495 module.
static const float AD8495_MV_PER_C = 5.0f;

// --- KC868-A6 / PCF8574 relays ---
static const uint8_t I2C_SDA_PIN = 4;
static const uint8_t I2C_SCL_PIN = 15;
static const uint8_t PCF8574_ADDRESS = 0x24;
static const uint8_t PCF8574_INPUT_ADDRESS = 0x22;
static const uint8_t HEATER_RELAY_PHASE_PIN = 0;   // Relay 1: heater phase.
static const uint8_t HEATER_RELAY_NEUTRAL_PIN = 1; // Relay 2: heater neutral.
static const uint8_t RELAY_MOTOR_A_PIN = 4;        // Relay 5.
static const uint8_t RELAY_MOTOR_B_PIN = 5;        // Relay 6.
static const uint8_t DI6_BIT = 5;                  // P5 on input PCF8574.
static const bool DI6_ACTIVE_LOW = true;
static const uint16_t RELAY_DEADTIME_MS = 120;
static const bool STOP_USE_NO_STATE = false;       // false = NC/NC stop, true = NO/NO stop.
static const uint32_t OPEN_TRAVEL_MS = 15000;

uint8_t relayOutputMask = 0xFF;

// --- WiFi AP ---
const char* ap_ssid = "grill";
const char* ap_password = "clickclick";

WebServer server(80);
DNSServer dnsServer;
Preferences preferences;
const uint16_t DNS_PORT = 53;
HardwareSerial RS485Serial(2);
ModbusRTU mb;

static const uint8_t RS485_RX_PIN = 14;
static const uint8_t RS485_TX_PIN = 27;
static const uint32_t RS485_BAUDRATE = 9600;
static const uint8_t MODBUS_SLAVE_ID = 1;

enum ModbusReg : uint16_t {
  REG_STATUS_BITS = 0,        // RO bitmask: 0 heating_enabled,1 heater_on,2 autotune_active,3 fault,4 thermo_open
  REG_TEMPERATURE_X10 = 1,    // RO int16
  REG_TARGET_X10 = 2,         // RW int16
  REG_POWER_X10 = 3,          // RO uint16
  REG_KP_X100 = 4,            // RW uint16
  REG_KI_X10000 = 5,          // RW uint16
  REG_KD_X100 = 6,            // RW uint16
  REG_COMMAND = 7,            // RW write command, firmware resets to 0 after processing
  REG_COMMAND_RESULT = 8,     // RO command result code
  REG_AUTOTUNE_CYCLES = 9,    // RO
  REG_FAULT_CODE = 10,        // RO 0 none, 1 RTD open/fault, 2 emergency
  REG_HEATING_ENABLE = 11,    // RW 0/1
  REG_AUTOTUNE_READY = 12,    // RO 0/1
  REG_MOTOR_DIRECTION = 13,   // RO 0 stop, 1 open, 2 close
  REG_MOTOR_OPEN_REMAINING_SEC = 14, // RO
  REG_REG_COUNT = 32
};

enum ModbusCommand : uint16_t {
  CMD_NONE = 0,
  CMD_HEAT_ON = 1,
  CMD_HEAT_OFF = 2,
  CMD_AUTOTUNE_START = 3,
  CMD_AUTOTUNE_STOP = 4,
  CMD_CLEAR_FAULT = 5,
  CMD_SAVE_PID = 6,
  CMD_MOTOR_OPEN = 7,
  CMD_MOTOR_CLOSE = 8,
  CMD_MOTOR_STOP = 9
};

enum ModbusCommandResult : uint16_t {
  CMD_RES_OK = 0,
  CMD_RES_UNKNOWN_CMD = 1,
  CMD_RES_BUSY = 2,
  CMD_RES_INVALID_ARG = 3
};

// --- Temperature / control state ---
float lastTemperatureC = NAN;
float ad8495TemperatureC = NAN;
float ad8495VoltageMv = NAN;
bool thermoOpenCircuit = false;
bool heaterRelaysOn = false;
bool heatingEnabled = false;
bool faultActive = false;
String faultText = "";
uint32_t lastSensorReadMs = 0;
bool di6Active = false;

enum MotorDirection : uint8_t {
  MOTOR_STOP = 0,
  MOTOR_OPEN = 1,
  MOTOR_CLOSE = 2
};

MotorDirection currentDirection = MOTOR_STOP;
uint32_t motorRunStartMs = 0;
uint32_t openRemainingMs = OPEN_TRAVEL_MS;
uint8_t max31865LastFault = 0;
uint16_t max31865LastRaw = 0;
float max31865LastResistanceOhm = NAN;

float targetTemperatureC = 180.0f;
float heaterPowerPercent = 0.0f;

// Conservative starting coefficients for a slow heater. Autotune can replace them.
float pidKp = 12.0f;
float pidKi = 0.08f;
float pidKd = 8.0f;
float pidIntegral = 0.0f;
float pidPreviousError = 0.0f;
bool pidHasPreviousError = false;
uint32_t lastPidComputeMs = 0;

static const uint32_t SENSOR_READ_INTERVAL_MS = 500;
static const uint32_t PID_INTERVAL_MS = 500;
static const float MIN_TARGET_C = 0.0f;
static const float MAX_TARGET_C = 300.0f;
static const float EMERGENCY_STOP_C = 330.0f;
static const uint8_t SSR_PWM_PIN = 33;
static const uint8_t SSR_PWM_CHANNEL = 0;
static const uint16_t SSR_PWM_FREQ_HZ = 1000;
static const uint8_t SSR_PWM_RESOLUTION_BITS = 10;
static const uint16_t SSR_PWM_MAX_DUTY = (1U << SSR_PWM_RESOLUTION_BITS) - 1;

// --- Fast PID autotune by one heat-up curve ---
bool autotuneActive = false;
bool autotuneReady = false;
bool autotuneHeating = false;
uint16_t autotuneCycles = 0;
uint32_t autotuneLastSampleMs = 0;
uint32_t autotuneRiseDelayMs = 0;
float autotuneStartTemperatureC = NAN;
float autotunePreviousTemperatureC = NAN;
float autotuneMaxSlopeCPerSec = 0.0f;
String autotuneStatus = "Остановлен";

static const float AUTOTUNE_MIN_DELTA_C = 10.0f;
static const float AUTOTUNE_RISE_DETECT_C = 0.5f;
static const uint32_t AUTOTUNE_TIMEOUT_MS = 25UL * 60UL * 1000UL;
static const float AUTOTUNE_FINISH_RATIO = 0.95f;
uint32_t autotuneStartMs = 0;
static const float TEMP_FILTER_ALPHA = 0.25f;

void resetPid();
void setTargetTemperature(float targetC);
float clampFloat(float value, float low, float high);
void syncModbusRegistersFromState();
void applyModbusWritableRegisters();
void handleModbusCommand();

uint16_t floatToU16(float value, float scale) {
  int32_t scaled = static_cast<int32_t>(lroundf(value * scale));
  if (scaled < 0) {
    scaled = 0;
  }
  if (scaled > 65535) {
    scaled = 65535;
  }
  return static_cast<uint16_t>(scaled);
}

uint16_t floatToI16Reg(float value, float scale) {
  int32_t scaled = static_cast<int32_t>(lroundf(value * scale));
  if (scaled < -32768) {
    scaled = -32768;
  }
  if (scaled > 32767) {
    scaled = 32767;
  }
  return static_cast<uint16_t>(static_cast<int16_t>(scaled));
}

float i16RegToFloat(uint16_t regValue, float scale) {
  return static_cast<float>(static_cast<int16_t>(regValue)) / scale;
}

String decodeMax31865Fault(uint8_t fault) {
  if (fault == 0) {
    return "NONE";
  }
  String s = "";
  if (fault & MAX31865_FAULT_HIGHTHRESH) {
    s += "HIGH_THRESH;";
  }
  if (fault & MAX31865_FAULT_LOWTHRESH) {
    s += "LOW_THRESH;";
  }
  if (fault & MAX31865_FAULT_REFINLOW) {
    s += "REFIN_LOW;";
  }
  if (fault & MAX31865_FAULT_REFINHIGH) {
    s += "REFIN_HIGH;";
  }
  if (fault & MAX31865_FAULT_RTDINLOW) {
    s += "RTDIN_LOW;";
  }
  if (fault & MAX31865_FAULT_OVUV) {
    s += "OVUV;";
  }
  return s;
}

void loadPidSettings() {
  preferences.begin("pid", true);
  pidKp = preferences.getFloat("kp", pidKp);
  pidKi = preferences.getFloat("ki", pidKi);
  pidKd = preferences.getFloat("kd", pidKd);
  preferences.end();
}

void savePidSettings() {
  preferences.begin("pid", false);
  preferences.putFloat("kp", pidKp);
  preferences.putFloat("ki", pidKi);
  preferences.putFloat("kd", pidKd);
  preferences.end();
}

void initInputExpander() {
  // For PCF8574, writing 1 keeps pins in quasi-bidirectional input mode.
  Wire.beginTransmission(PCF8574_INPUT_ADDRESS);
  Wire.write(0xFF);
  Wire.endTransmission();
}

void updateDI6State() {
  Wire.requestFrom((uint8_t)PCF8574_INPUT_ADDRESS, (uint8_t)1);
  if (Wire.available() < 1) {
    return;
  }

  uint8_t inputByte = Wire.read();
  bool bitState = bitRead(inputByte, DI6_BIT);
  di6Active = DI6_ACTIVE_LOW ? !bitState : bitState;
}

float readMAX31865C(bool& openCircuit) {
  // Clear stale latched faults before a fresh conversion.
  rtd.clearFault();
  float temp = rtd.temperature(MAX31865_RTD_NOMINAL, MAX31865_REF_RESISTOR);
  max31865LastRaw = rtd.readRTD();
  max31865LastResistanceOhm = (static_cast<float>(max31865LastRaw) / 32768.0f) * MAX31865_REF_RESISTOR;
  uint8_t fault = rtd.readFault();
  max31865LastFault = fault;
  if (fault) {
    // Treat any RTD wiring/fault state as sensor-open/fault for control safety.
    openCircuit = true;
    faultText = String("MAX31865 fault: ") + decodeMax31865Fault(fault);
    rtd.clearFault();
    return NAN;
  }

  openCircuit = false;
  if (!isfinite(temp)) {
    openCircuit = true;
    faultText = "MAX31865 fault: NON_FINITE_TEMP";
    return NAN;
  }
  return temp;
}

float readStableMAX31865C(bool& openCircuit) {
  // Reduce EMI-related spikes by median of several reads.
  float samples[5];
  uint8_t validCount = 0;
  bool gotOpenCircuit = false;

  for (uint8_t i = 0; i < 5; i++) {
    bool sampleOpen = false;
    float t = readMAX31865C(sampleOpen);
    if (sampleOpen) {
      gotOpenCircuit = true;
    } else if (!isnan(t)) {
      samples[validCount++] = t;
    }
    delay(5);
  }

  if (validCount == 0) {
    openCircuit = gotOpenCircuit;
    return NAN;
  }

  for (uint8_t i = 0; i < validCount; i++) {
    for (uint8_t j = i + 1; j < validCount; j++) {
      if (samples[j] < samples[i]) {
        float tmp = samples[i];
        samples[i] = samples[j];
        samples[j] = tmp;
      }
    }
  }

  openCircuit = false;
  return samples[validCount / 2];
}

float readAD8495C() {
  uint32_t totalMv = 0;
  for (uint8_t i = 0; i < AD8495_SAMPLE_COUNT; i++) {
    totalMv += analogReadMilliVolts(ANALOG_A1);
    delay(1);
  }

  ad8495VoltageMv = static_cast<float>(totalMv) / AD8495_SAMPLE_COUNT;
  return (ad8495VoltageMv - AD8495_ZERO_C_MV) / AD8495_MV_PER_C;
}

void applyRelayOutputMask() {
  Wire.beginTransmission(PCF8574_ADDRESS);
  Wire.write(relayOutputMask);
  Wire.endTransmission();
}

void setRelayPin(uint8_t pin, bool enabled) {
  if (enabled) {
    bitClear(relayOutputMask, pin);
  } else {
    bitSet(relayOutputMask, pin);
  }

  applyRelayOutputMask();
}

void setAllRelaysOff() {
  relayOutputMask = 0xFF;
  applyRelayOutputMask();
  heaterRelaysOn = false;
  currentDirection = MOTOR_STOP;
  motorRunStartMs = 0;
}

void setHeaterRelays(bool enabled) {
  heaterRelaysOn = enabled;
  setRelayPin(HEATER_RELAY_PHASE_PIN, enabled);
  setRelayPin(HEATER_RELAY_NEUTRAL_PIN, enabled);
}

void setMotorRelaysRaw(bool relayA_no, bool relayB_no) {
  setRelayPin(RELAY_MOTOR_A_PIN, relayA_no);
  setRelayPin(RELAY_MOTOR_B_PIN, relayB_no);
}

void stopMotorRaw() {
  setMotorRelaysRaw(STOP_USE_NO_STATE, STOP_USE_NO_STATE);
  currentDirection = MOTOR_STOP;
  motorRunStartMs = 0;
}

void accountOpenElapsed() {
  if (currentDirection != MOTOR_OPEN || motorRunStartMs == 0) {
    return;
  }

  uint32_t elapsedMs = millis() - motorRunStartMs;
  if (elapsedMs >= openRemainingMs) {
    openRemainingMs = 0;
  } else {
    openRemainingMs -= elapsedMs;
  }
}

void stopMotor() {
  accountOpenElapsed();
  stopMotorRaw();
}

void startOpenMotor() {
  if (currentDirection == MOTOR_OPEN || openRemainingMs == 0) {
    return;
  }

  stopMotor();
  delay(RELAY_DEADTIME_MS);
  setMotorRelaysRaw(true, false);
  currentDirection = MOTOR_OPEN;
  motorRunStartMs = millis();
}

void startCloseMotor() {
  if (currentDirection == MOTOR_CLOSE) {
    return;
  }

  stopMotor();
  delay(RELAY_DEADTIME_MS);

  if (di6Active) {
    openRemainingMs = OPEN_TRAVEL_MS;
    stopMotorRaw();
    return;
  }

  setMotorRelaysRaw(false, true);
  currentDirection = MOTOR_CLOSE;
  motorRunStartMs = millis();
}

void completeOpenMotor() {
  openRemainingMs = 0;
  stopMotorRaw();
}

void completeCloseMotor() {
  openRemainingMs = OPEN_TRAVEL_MS;
  stopMotorRaw();
}

void updateMotorControl() {
  if (currentDirection == MOTOR_OPEN && motorRunStartMs != 0) {
    if ((millis() - motorRunStartMs) >= openRemainingMs) {
      completeOpenMotor();
    }
    return;
  }

  if (currentDirection == MOTOR_CLOSE && di6Active) {
    completeCloseMotor();
  }
}

uint16_t openRemainingSeconds() {
  if (currentDirection == MOTOR_OPEN && motorRunStartMs != 0) {
    uint32_t elapsedMs = millis() - motorRunStartMs;
    if (elapsedMs >= openRemainingMs) {
      return 0;
    }
    return (uint16_t)((openRemainingMs - elapsedMs + 999) / 1000);
  }

  return (uint16_t)((openRemainingMs + 999) / 1000);
}

void runStartupOpenCycle() {
  openRemainingMs = OPEN_TRAVEL_MS;
  startOpenMotor();
  delay(OPEN_TRAVEL_MS);
  completeOpenMotor();
}

const char* motorDirectionText() {
  if (currentDirection == MOTOR_OPEN) {
    return "Открытие";
  }
  if (currentDirection == MOTOR_CLOSE) {
    return "Закрытие";
  }
  return "Стоп";
}

void setSsrPwmPercent(float powerPercent) {
  float clampedPercent = clampFloat(powerPercent, 0.0f, 100.0f);
  uint16_t duty = static_cast<uint16_t>((clampedPercent * SSR_PWM_MAX_DUTY) / 100.0f);
  ledcWrite(SSR_PWM_CHANNEL, duty);
}

void stopHeater(const String& reason) {
  heaterPowerPercent = 0.0f;
  setSsrPwmPercent(0.0f);
  setHeaterRelays(false);

  if (reason.length() > 0) {
    faultActive = true;
    faultText = reason;
  }
}

void setHeatingEnabled(bool enabled) {
  heatingEnabled = enabled;
  mb.Hreg(REG_HEATING_ENABLE, heatingEnabled ? 1 : 0);

  if (!heatingEnabled) {
    stopHeater("");
  }

  resetPid();
}

void resetPid() {
  pidIntegral = 0.0f;
  pidPreviousError = 0.0f;
  pidHasPreviousError = false;
  lastPidComputeMs = 0;
}

void setTargetTemperature(float targetC) {
  targetTemperatureC = clampFloat(targetC, MIN_TARGET_C, MAX_TARGET_C);
  mb.Hreg(REG_TARGET_X10, floatToI16Reg(targetTemperatureC, 10.0f));
  resetPid();
}

float clampFloat(float value, float low, float high) {
  if (value < low) {
    return low;
  }
  if (value > high) {
    return high;
  }
  return value;
}

void stopAutotune(const String& statusText, bool keepReady) {
  autotuneActive = false;
  autotuneReady = keepReady;
  autotuneHeating = false;
  autotuneStatus = statusText;
  heatingEnabled = false;
  stopHeater("");
  resetPid();
}

void startAutotune() {
  if (faultActive || thermoOpenCircuit || isnan(lastTemperatureC)) {
    autotuneStatus = "Нет валидной температуры";
    return;
  }

  if (targetTemperatureC - lastTemperatureC < AUTOTUNE_MIN_DELTA_C) {
    autotuneStatus = "Уставка должна быть минимум на 10°C выше текущей";
    return;
  }

  autotuneActive = true;
  heatingEnabled = false;
  autotuneReady = false;
  autotuneHeating = true;
  autotuneCycles = 0;
  autotuneLastSampleMs = millis();
  autotuneRiseDelayMs = 0;
  autotuneStartTemperatureC = lastTemperatureC;
  autotunePreviousTemperatureC = lastTemperatureC;
  autotuneMaxSlopeCPerSec = 0.0f;
  autotuneStartMs = millis();
  autotuneStatus = "Быстрый автотюнинг: нагрев до уставки";
  heaterPowerPercent = 100.0f;
  setHeaterRelays(true);
}

void finishAutotune() {
  if (autotuneMaxSlopeCPerSec <= 0.001f || autotuneRiseDelayMs == 0) {
    stopAutotune("Не удалось рассчитать коэффициенты", false);
    return;
  }

  float temperatureDeltaC = targetTemperatureC - autotuneStartTemperatureC;
  float timeToTargetSec = temperatureDeltaC / autotuneMaxSlopeCPerSec;
  float slopeCPerMin = autotuneMaxSlopeCPerSec * 60.0f;
  float deadTimeSec = max(1.0f, autotuneRiseDelayMs / 1000.0f);

  pidKp = clampFloat(80.0f / (slopeCPerMin + 0.5f), 8.0f, 30.0f);
  pidKi = clampFloat(pidKp / max(60.0f, timeToTargetSec * 2.5f), 0.01f, 0.15f);
  pidKd = clampFloat(pidKp * (deadTimeSec / max(30.0f, timeToTargetSec)), 2.0f, 20.0f);
  savePidSettings();

  stopAutotune("Готово, коэффициенты применены", true);
}

void updateAutotune(uint32_t nowMs) {
  if (!autotuneActive) {
    return;
  }

  if (faultActive || thermoOpenCircuit || isnan(lastTemperatureC)) {
    stopAutotune("Остановлен из-за ошибки датчика", false);
    return;
  }

  if (lastTemperatureC >= EMERGENCY_STOP_C) {
    stopAutotune("Аварийная температура", false);
    faultActive = true;
    faultText = "Аварийная температура";
    return;
  }

  if (nowMs - autotuneStartMs > AUTOTUNE_TIMEOUT_MS) {
    stopAutotune("Таймаут автотюнинга", false);
    return;
  }

  if (nowMs == autotuneLastSampleMs) {
    return;
  }

  float dtSec = (nowMs - autotuneLastSampleMs) / 1000.0f;
  if (dtSec > 0.0f) {
    float slopeCPerSec = (lastTemperatureC - autotunePreviousTemperatureC) / dtSec;
    if (slopeCPerSec > autotuneMaxSlopeCPerSec) {
      autotuneMaxSlopeCPerSec = slopeCPerSec;
    }
  }

  if (autotuneRiseDelayMs == 0 && lastTemperatureC >= autotuneStartTemperatureC + AUTOTUNE_RISE_DETECT_C) {
    autotuneRiseDelayMs = nowMs - autotuneStartMs;
  }

  autotunePreviousTemperatureC = lastTemperatureC;
  autotuneLastSampleMs = nowMs;
  autotuneCycles++;

  float finishTemperature = autotuneStartTemperatureC + ((targetTemperatureC - autotuneStartTemperatureC) * AUTOTUNE_FINISH_RATIO);
  if (lastTemperatureC >= finishTemperature) {
    heaterPowerPercent = 0.0f;
    finishAutotune();
  }
}

void computePid(uint32_t nowMs) {
  if (autotuneActive) {
    return;
  }

  if (!heatingEnabled) {
    heaterPowerPercent = 0.0f;
    setHeaterRelays(false);
    return;
  }

  if (faultActive || thermoOpenCircuit || isnan(lastTemperatureC)) {
    stopHeater("");
    return;
  }

  if (lastTemperatureC >= EMERGENCY_STOP_C) {
    stopHeater("Аварийная температура");
    return;
  }

  if (lastPidComputeMs != 0 && nowMs - lastPidComputeMs < PID_INTERVAL_MS) {
    return;
  }

  float dt = lastPidComputeMs == 0 ? (PID_INTERVAL_MS / 1000.0f) : ((nowMs - lastPidComputeMs) / 1000.0f);
  lastPidComputeMs = nowMs;

  float error = targetTemperatureC - lastTemperatureC;
  pidIntegral += error * dt;
  pidIntegral = clampFloat(pidIntegral, -1000.0f, 1000.0f);

  float derivative = 0.0f;
  if (pidHasPreviousError && dt > 0.0f) {
    derivative = (error - pidPreviousError) / dt;
  }
  pidPreviousError = error;
  pidHasPreviousError = true;

  float output = (pidKp * error) + (pidKi * pidIntegral) + (pidKd * derivative);
  heaterPowerPercent = clampFloat(output, 0.0f, 100.0f);
}

void applyHeaterOutput() {
  if (faultActive || thermoOpenCircuit || isnan(lastTemperatureC)) {
    setSsrPwmPercent(0.0f);
    setHeaterRelays(false);
    return;
  }

  if (autotuneActive || heatingEnabled) {
    setHeaterRelays(true);
    setSsrPwmPercent(heaterPowerPercent);
    return;
  }

  setSsrPwmPercent(0.0f);
  setHeaterRelays(false);
}

String formatTemperatureValue() {
  if (thermoOpenCircuit) {
    return "Ошибка";
  }

  if (isnan(lastTemperatureC)) {
    return "Нет данных";
  }

  char buff[24];
  snprintf(buff, sizeof(buff), "%.2f", lastTemperatureC);
  return String(buff);
}

String formatAD8495TemperatureValue() {
  if (isnan(ad8495TemperatureC)) {
    return "Нет данных";
  }

  char buff[12];
  snprintf(buff, sizeof(buff), "%.2f", ad8495TemperatureC);
  return String(buff);
}

String boolToJson(bool value) {
  return value ? "true" : "false";
}

uint8_t rtdWireCount() {
  if (MAX31865_WIRES == MAX31865_2WIRE) {
    return 2;
  }
  if (MAX31865_WIRES == MAX31865_3WIRE) {
    return 3;
  }
  return 4;
}

String statusJson() {
  String json = "{";
  json += "\"temperature\":";
  json += isnan(lastTemperatureC) ? "null" : String(lastTemperatureC, 2);
  json += ",\"ad8495_temperature\":";
  json += isnan(ad8495TemperatureC) ? "null" : String(ad8495TemperatureC, 2);
  json += ",\"ad8495_mv\":";
  json += isnan(ad8495VoltageMv) ? "null" : String(ad8495VoltageMv, 1);
  json += ",\"target\":" + String(targetTemperatureC, 1);
  json += ",\"power\":" + String(heaterPowerPercent, 1);
  json += ",\"heater_on\":" + boolToJson(heaterRelaysOn);
  json += ",\"heating_enabled\":" + boolToJson(heatingEnabled);
  json += ",\"fault\":" + boolToJson(faultActive || thermoOpenCircuit);
  json += ",\"fault_text\":\"";
  if (faultText.length() > 0) {
    json += faultText;
  } else if (thermoOpenCircuit) {
    json += "RTD датчик не подключен/ошибка MAX31865";
  }
  json += "\",\"autotune_active\":" + boolToJson(autotuneActive);
  json += ",\"autotune_ready\":" + boolToJson(autotuneReady);
  json += ",\"autotune_cycles\":" + String(autotuneCycles);
  json += ",\"autotune_status\":\"" + autotuneStatus + "\"";
  json += ",\"kp\":" + String(pidKp, 4);
  json += ",\"ki\":" + String(pidKi, 6);
  json += ",\"kd\":" + String(pidKd, 4);
  json += ",\"fault_hex\":\"0x";
  if (max31865LastFault < 16) {
    json += "0";
  }
  json += String(max31865LastFault, HEX);
  json += "\"";
  json += ",\"rtd_raw\":" + String(max31865LastRaw);
  json += ",\"rtd_ohm\":";
  json += isnan(max31865LastResistanceOhm) ? "null" : String(max31865LastResistanceOhm, 3);
  json += ",\"rtd_wires\":" + String(rtdWireCount());
  json += ",\"di6_active\":" + boolToJson(di6Active);
  json += ",\"motor_direction\":" + String((uint8_t)currentDirection);
  json += ",\"motor_direction_text\":\"" + String(motorDirectionText()) + "\"";
  json += ",\"motor_open_remaining_sec\":" + String(openRemainingSeconds());
  json += "}";
  return json;
}

void syncModbusRegistersFromState() {
  uint16_t statusBits = 0;
  if (heatingEnabled) {
    statusBits |= (1U << 0);
  }
  if (heaterRelaysOn) {
    statusBits |= (1U << 1);
  }
  if (autotuneActive) {
    statusBits |= (1U << 2);
  }
  if (faultActive) {
    statusBits |= (1U << 3);
  }
  if (thermoOpenCircuit) {
    statusBits |= (1U << 4);
  }
  if (di6Active) {
    statusBits |= (1U << 5);
  }

  uint16_t faultCode = 0;
  if (thermoOpenCircuit) {
    faultCode = 1;
  } else if (faultActive) {
    faultCode = 2;
  }

  mb.Hreg(REG_STATUS_BITS, statusBits);
  mb.Hreg(REG_TEMPERATURE_X10, isnan(lastTemperatureC) ? static_cast<uint16_t>(0x8000) : floatToI16Reg(lastTemperatureC, 10.0f));
  mb.Hreg(REG_TARGET_X10, floatToI16Reg(targetTemperatureC, 10.0f));
  mb.Hreg(REG_POWER_X10, floatToU16(heaterPowerPercent, 10.0f));
  mb.Hreg(REG_KP_X100, floatToU16(pidKp, 100.0f));
  mb.Hreg(REG_KI_X10000, floatToU16(pidKi, 10000.0f));
  mb.Hreg(REG_KD_X100, floatToU16(pidKd, 100.0f));
  mb.Hreg(REG_AUTOTUNE_CYCLES, autotuneCycles);
  mb.Hreg(REG_FAULT_CODE, faultCode);
  mb.Hreg(REG_HEATING_ENABLE, heatingEnabled ? 1 : 0);
  mb.Hreg(REG_AUTOTUNE_READY, autotuneReady ? 1 : 0);
  mb.Hreg(REG_MOTOR_DIRECTION, (uint16_t)currentDirection);
  mb.Hreg(REG_MOTOR_OPEN_REMAINING_SEC, openRemainingSeconds());
}

void applyModbusWritableRegisters() {
  uint16_t regTarget = mb.Hreg(REG_TARGET_X10);
  uint16_t regKp = mb.Hreg(REG_KP_X100);
  uint16_t regKi = mb.Hreg(REG_KI_X10000);
  uint16_t regKd = mb.Hreg(REG_KD_X100);
  uint16_t regHeatingEnable = mb.Hreg(REG_HEATING_ENABLE);
  float requestedTarget = clampFloat(i16RegToFloat(regTarget, 10.0f), MIN_TARGET_C, MAX_TARGET_C);
  if (fabsf(requestedTarget - targetTemperatureC) > 0.01f) {
    if (!autotuneActive) {
      setTargetTemperature(requestedTarget);
      faultActive = false;
      faultText = "";
      mb.Hreg(REG_COMMAND_RESULT, CMD_RES_OK);
    } else {
      mb.Hreg(REG_COMMAND_RESULT, CMD_RES_BUSY);
    }
  }

  float requestedKp = clampFloat(static_cast<float>(regKp) / 100.0f, 0.0f, 300.0f);
  float requestedKi = clampFloat(static_cast<float>(regKi) / 10000.0f, 0.0f, 10.0f);
  float requestedKd = clampFloat(static_cast<float>(regKd) / 100.0f, 0.0f, 300.0f);
  if (fabsf(requestedKp - pidKp) > 0.001f || fabsf(requestedKi - pidKi) > 0.0001f || fabsf(requestedKd - pidKd) > 0.001f) {
    pidKp = requestedKp;
    pidKi = requestedKi;
    pidKd = requestedKd;
    resetPid();
    savePidSettings();
    mb.Hreg(REG_COMMAND_RESULT, CMD_RES_OK);
  }

  bool requestedHeatingEnable = regHeatingEnable != 0;
  if (requestedHeatingEnable != heatingEnabled) {
    if (!autotuneActive) {
      setHeatingEnabled(requestedHeatingEnable);
      mb.Hreg(REG_COMMAND_RESULT, CMD_RES_OK);
    } else {
      mb.Hreg(REG_COMMAND_RESULT, CMD_RES_BUSY);
    }
  }
}

void handleModbusCommand() {
  uint16_t cmd = mb.Hreg(REG_COMMAND);
  if (cmd == CMD_NONE) {
    return;
  }

  uint16_t result = CMD_RES_OK;
  switch (cmd) {
    case CMD_HEAT_ON:
      if (autotuneActive) {
        result = CMD_RES_BUSY;
      } else {
        faultActive = false;
        faultText = "";
        setHeatingEnabled(true);
      }
      break;
    case CMD_HEAT_OFF:
      setHeatingEnabled(false);
      break;
    case CMD_AUTOTUNE_START:
      faultActive = false;
      faultText = "";
      startAutotune();
      if (!autotuneActive) {
        result = CMD_RES_INVALID_ARG;
      }
      break;
    case CMD_AUTOTUNE_STOP:
      stopAutotune("Остановлен вручную", false);
      break;
    case CMD_CLEAR_FAULT:
      faultActive = false;
      faultText = "";
      result = CMD_RES_OK;
      break;
    case CMD_SAVE_PID:
      savePidSettings();
      break;
    case CMD_MOTOR_OPEN:
      startOpenMotor();
      break;
    case CMD_MOTOR_CLOSE:
      startCloseMotor();
      break;
    case CMD_MOTOR_STOP:
      stopMotor();
      break;
    default:
      result = CMD_RES_UNKNOWN_CMD;
      break;
  }

  mb.Hreg(REG_COMMAND_RESULT, result);
  mb.Hreg(REG_COMMAND, CMD_NONE);
}

String generateHTML() {
  String html = R"rawliteral(
<!doctype html>
<html lang="ru">
<head>
  <meta charset="utf-8">
  <meta name="viewport" content="width=device-width,initial-scale=1">
  <title>ПИД нагреватель</title>
  <style>
    :root {
      --bg: #f1f5f2;
      --panel: #ffffff;
      --text: #172016;
      --muted: #657065;
      --line: #c9d5c8;
      --accent: #0f8a5f;
      --warn: #b42318;
    }
    * { box-sizing: border-box; }
    body {
      margin: 0;
      min-height: 100vh;
      display: grid;
      place-items: center;
      padding: 18px;
      font-family: "Segoe UI", Tahoma, sans-serif;
      color: var(--text);
      background:
        linear-gradient(135deg, rgba(15, 138, 95, .12), rgba(255, 255, 255, 0) 38%),
        linear-gradient(315deg, rgba(20, 83, 45, .12), rgba(255, 255, 255, 0) 42%),
        var(--bg);
    }
    .panel {
      width: min(560px, 100%);
      background: var(--panel);
      border: 1px solid var(--line);
      border-radius: 8px;
      box-shadow: 0 18px 42px rgba(23, 32, 22, .14);
      padding: 22px;
    }
    h1 { margin: 0 0 4px; font-size: 25px; }
    .sub { margin: 0 0 18px; color: var(--muted); font-size: 14px; }
    .grid { display: grid; grid-template-columns: 1fr 1fr; gap: 12px; }
    .cell {
      border: 1px solid var(--line);
      border-radius: 8px;
      padding: 14px;
      background: #fbfdfb;
    }
    .label { color: var(--muted); font-size: 13px; margin-bottom: 8px; }
    .value { font-size: 34px; line-height: 1; font-weight: 700; color: var(--accent); }
    .unit { font-size: 16px; color: var(--muted); }
    form, .actions { display: grid; grid-template-columns: 1fr auto; gap: 10px; margin-top: 14px; }
    input {
      min-width: 0;
      border: 1px solid var(--line);
      border-radius: 8px;
      padding: 12px;
      font-size: 18px;
    }
    button {
      border: 0;
      border-radius: 8px;
      padding: 0 18px;
      min-height: 46px;
      font-size: 16px;
      font-weight: 700;
      color: white;
      background: var(--accent);
    }
    button.stop { background: var(--warn); }
    button:disabled, input:disabled {
      opacity: .5;
      cursor: not-allowed;
    }
    .status {
      display: flex;
      justify-content: space-between;
      gap: 12px;
      margin-top: 14px;
      padding-top: 14px;
      border-top: 1px solid var(--line);
      color: var(--muted);
      font-size: 14px;
      flex-wrap: wrap;
    }
    .fault { color: var(--warn); font-weight: 700; }
    @media (max-width: 460px) {
      .grid, form, .actions { grid-template-columns: 1fr; }
    }
  </style>
</head>
<body>
  <main class="panel">
    <h1>Управление ТЭНом</h1>
    <p class="sub">Реле 1 и 2 включаются вместе: фаза и ноль нагревателя.</p>
    <section class="grid">
      <div class="cell">
        <div class="label">Текущая температура</div>
        <div class="value"><span id="temperature">)rawliteral";

  html += formatTemperatureValue();

  html += R"rawliteral(</span> <span class="unit">°C</span></div>
      </div>
      <div class="cell">
        <div class="label">Термопара AD8495</div>
        <div class="value"><span id="ad8495Temperature">)rawliteral";

  html += formatAD8495TemperatureValue();

  html += R"rawliteral(</span> <span class="unit">°C</span></div>
      </div>
      <div class="cell">
        <div class="label">Уставка</div>
        <div class="value"><span id="target">)rawliteral";

  html += String(targetTemperatureC, 1);

  html += R"rawliteral(</span> <span class="unit">°C</span></div>
      </div>
    </section>
    <form action="/set" method="get">
      <input id="targetInput" name="target" type="number" step="0.5" min="0" max="300" value=")rawliteral";

  html += String(targetTemperatureC, 1);

  html += R"rawliteral(">
      <button id="setButton" type="submit">Задать</button>
    </form>
    <div class="actions">
      <form action="/heat/on" method="get">
        <button id="heatOnButton" type="submit">Включить нагрев</button>
      </form>
      <form action="/heat/off" method="get">
        <button id="heatOffButton" class="stop" type="submit">Выключить нагрев</button>
      </form>
    </div>
    <div class="actions">
      <form id="autotuneStartForm" action="/autotune/start" method="get">
        <button id="autotuneStartButton" type="submit">Быстрый автотюнинг</button>
      </form>
      <form id="autotuneStopForm" action="/autotune/stop" method="get">
        <button id="autotuneStopButton" class="stop" type="submit">Остановить</button>
      </form>
    </div>
    <div class="actions">
      <form action="/motor/open" method="get">
        <button id="motorOpenButton" type="submit">Открыть</button>
      </form>
      <form action="/motor/close" method="get">
        <button id="motorCloseButton" type="submit">Закрыть</button>
      </form>
      <form action="/motor/stop" method="get">
        <button id="motorStopButton" class="stop" type="submit">Стоп мотор</button>
      </form>
    </div>
    <div class="status">
      <span>Мощность: <b id="power">)rawliteral";

  html += String(heaterPowerPercent, 1);

  html += R"rawliteral(</b>%</span>
      <span>ТЭН: <b id="heater">)rawliteral";

  html += heaterRelaysOn ? "вкл" : "выкл";

  html += R"rawliteral(</b></span>
      <span>Нагрев: <b id="enabled">)rawliteral";

  html += heatingEnabled ? "включен" : "выключен";

  html += R"rawliteral(</b></span>
      <span>Мотор: <b id="motorDirection">)rawliteral";

  html += motorDirectionText();

  html += R"rawliteral(</b></span>
      <span>Автотюнинг: <b id="autotune">)rawliteral";

  html += autotuneStatus;

  html += R"rawliteral(</b></span>
    </div>
    <div class="status">
      <span>Kp: <b id="kp">)rawliteral";

  html += String(pidKp, 3);

  html += R"rawliteral(</b></span>
      <span>Ki: <b id="ki">)rawliteral";

  html += String(pidKi, 5);

  html += R"rawliteral(</b></span>
      <span>Kd: <b id="kd">)rawliteral";

  html += String(pidKd, 3);

  html += R"rawliteral(</b></span>
      <span>Точки: <b id="cycles">)rawliteral";

  html += String(autotuneCycles);

  html += R"rawliteral(</b></span>
      <span>RTD wires: <b id="rtdWires">3</b></span>
      <span>Fault HEX: <b id="faultHex">0x00</b></span>
      <span>RTD RAW: <b id="rtdRaw">0</b></span>
      <span>RTD Ω: <b id="rtdOhm">0.000</b></span>
      <span>AD8495: <b id="ad8495Mv">0.0</b> mV</span>
      <span>Открытие осталось: <b id="motorOpenRemaining">0</b> с</span>
    </div>
    <div id="fault" class="status fault"></div>
  </main>
  <script>
    async function refreshStatus() {
      const response = await fetch('/api/status', { cache: 'no-store' });
      const data = await response.json();
      const busy = data.autotune_active;
      document.getElementById('temperature').textContent = data.temperature === null ? 'Нет данных' : data.temperature.toFixed(2);
      document.getElementById('ad8495Temperature').textContent = data.ad8495_temperature === null ? 'Нет данных' : data.ad8495_temperature.toFixed(2);
      const targetInput = document.getElementById('targetInput');
      document.getElementById('target').textContent = data.target.toFixed(1);
      if (document.activeElement !== targetInput) {
        targetInput.value = data.target.toFixed(1);
      }
      document.getElementById('power').textContent = data.power.toFixed(1);
      document.getElementById('heater').textContent = data.heater_on ? 'вкл' : 'выкл';
      document.getElementById('enabled').textContent = data.heating_enabled ? 'включен' : 'выключен';
      document.getElementById('motorDirection').textContent = data.motor_direction_text || 'Стоп';
      document.getElementById('autotune').textContent = data.autotune_status;
      document.getElementById('cycles').textContent = data.autotune_cycles;
      document.getElementById('kp').textContent = data.kp.toFixed(3);
      document.getElementById('ki').textContent = data.ki.toFixed(5);
      document.getElementById('kd').textContent = data.kd.toFixed(3);
      document.getElementById('faultHex').textContent = data.fault_hex || '0x00';
      document.getElementById('rtdRaw').textContent = (data.rtd_raw ?? 0).toString();
      document.getElementById('rtdOhm').textContent = (data.rtd_ohm == null ? 'N/A' : Number(data.rtd_ohm).toFixed(3));
      document.getElementById('ad8495Mv').textContent = data.ad8495_mv == null ? 'N/A' : Number(data.ad8495_mv).toFixed(1);
      document.getElementById('rtdWires').textContent = (data.rtd_wires ?? 3).toString();
      document.getElementById('motorOpenRemaining').textContent = (data.motor_open_remaining_sec ?? 0).toString();
      document.getElementById('fault').textContent = data.fault ? data.fault_text : '';
      targetInput.disabled = busy;
      document.getElementById('setButton').disabled = busy;
      document.getElementById('heatOnButton').disabled = busy || data.heating_enabled;
      document.getElementById('heatOffButton').disabled = busy || !data.heating_enabled;
      document.getElementById('autotuneStartButton').disabled = busy;
      document.getElementById('autotuneStopButton').disabled = !busy;
      document.getElementById('motorOpenButton').disabled = data.motor_direction === 1 || (data.motor_open_remaining_sec ?? 0) === 0;
      document.getElementById('motorCloseButton').disabled = data.motor_direction === 2 || data.di6_active;
      document.getElementById('motorStopButton').disabled = data.motor_direction === 0;
    }

    document.querySelectorAll('form').forEach(function (form) {
      form.addEventListener('submit', async function (event) {
        event.preventDefault();
        const url = new URL(form.action, window.location.origin);
        new FormData(form).forEach(function (value, key) {
          url.searchParams.set(key, value);
        });
        url.searchParams.set('ajax', '1');

        const buttons = form.querySelectorAll('button');
        buttons.forEach(function (button) { button.disabled = true; });
        try {
          await fetch(url.toString(), { cache: 'no-store' });
        } finally {
          await refreshStatus();
        }
      });
    });

    setInterval(refreshStatus, 1000);
    refreshStatus();
  </script>
</body>
</html>
)rawliteral";

  return html;
}

void handleRoot() {
  server.send(200, "text/html; charset=UTF-8", generateHTML());
}

void sendActionDone() {
  if (server.hasArg("ajax")) {
    server.send(204, "text/plain", "");
    return;
  }

  server.sendHeader("Location", "/", true);
  server.send(302, "text/plain", "");
}

void handleSetTarget() {
  if (autotuneActive) {
    server.send(409, "text/plain; charset=UTF-8", "Автотюнинг активен");
    return;
  }

  if (!server.hasArg("target")) {
    server.send(400, "text/plain; charset=UTF-8", "Нет уставки");
    return;
  }

  float requestedTarget = server.arg("target").toFloat();
  if (requestedTarget < MIN_TARGET_C || requestedTarget > MAX_TARGET_C) {
    server.send(400, "text/plain; charset=UTF-8", "Уставка вне диапазона");
    return;
  }

  setTargetTemperature(requestedTarget);
  faultActive = false;
  faultText = "";

  sendActionDone();
}

void handleAutotuneStart() {
  faultActive = false;
  faultText = "";
  startAutotune();
  sendActionDone();
}

void handleHeatOn() {
  if (autotuneActive) {
    server.send(409, "text/plain; charset=UTF-8", "Автотюнинг активен");
    return;
  }

  faultActive = false;
  faultText = "";
  setHeatingEnabled(true);
  sendActionDone();
}

void handleHeatOff() {
  setHeatingEnabled(false);
  sendActionDone();
}

void handleAutotuneStop() {
  stopAutotune("Остановлен вручную", false);
  sendActionDone();
}

void handleMotorOpen() {
  startOpenMotor();
  sendActionDone();
}

void handleMotorClose() {
  startCloseMotor();
  sendActionDone();
}

void handleMotorStop() {
  stopMotor();
  sendActionDone();
}

void handleStatus() {
  server.send(200, "application/json; charset=UTF-8", statusJson());
}

void handleRedirectToRoot() {
  server.sendHeader("Location", String("http://") + WiFi.softAPIP().toString() + "/", true);
  server.send(302, "text/plain", "");
}

void setup() {
  Serial.begin(115200);
  loadPidSettings();

  pinMode(ANALOG_A1, INPUT);
  analogReadResolution(12);
  analogSetPinAttenuation(ANALOG_A1, ADC_11db);

  Wire.begin(I2C_SDA_PIN, I2C_SCL_PIN);
  initInputExpander();
  setAllRelaysOff();
  updateDI6State();
  runStartupOpenCycle();

  pinMode(MAX31865_CS_PIN, OUTPUT);
  digitalWrite(MAX31865_CS_PIN, HIGH);
  SPI.begin(MAX31865_SCK_PIN, MAX31865_MISO_PIN, MAX31865_MOSI_PIN, MAX31865_CS_PIN);
  rtd.begin(MAX31865_WIRES);
  ledcSetup(SSR_PWM_CHANNEL, SSR_PWM_FREQ_HZ, SSR_PWM_RESOLUTION_BITS);
  ledcAttachPin(SSR_PWM_PIN, SSR_PWM_CHANNEL);
  setSsrPwmPercent(0.0f);

  RS485Serial.begin(RS485_BAUDRATE, SERIAL_8N1, RS485_RX_PIN, RS485_TX_PIN);
  mb.begin(&RS485Serial);
  mb.slave(MODBUS_SLAVE_ID);
  mb.addHreg(REG_STATUS_BITS, 0, REG_REG_COUNT);
  mb.Hreg(REG_COMMAND_RESULT, CMD_RES_OK);
  syncModbusRegistersFromState();

  delay(250);

  WiFi.softAP(ap_ssid, ap_password);
  delay(200);
  dnsServer.start(DNS_PORT, "*", WiFi.softAPIP());

  server.on("/", HTTP_GET, handleRoot);
  server.on("/set", HTTP_GET, handleSetTarget);
  server.on("/heat/on", HTTP_GET, handleHeatOn);
  server.on("/heat/off", HTTP_GET, handleHeatOff);
  server.on("/autotune/start", HTTP_GET, handleAutotuneStart);
  server.on("/autotune/stop", HTTP_GET, handleAutotuneStop);
  server.on("/motor/open", HTTP_GET, handleMotorOpen);
  server.on("/motor/close", HTTP_GET, handleMotorClose);
  server.on("/motor/stop", HTTP_GET, handleMotorStop);
  server.on("/api/status", HTTP_GET, handleStatus);
    
  server.on("/generate_204", HTTP_GET, handleRedirectToRoot);
  server.on("/hotspot-detect.html", HTTP_GET, handleRedirectToRoot);
  server.on("/fwlink", HTTP_GET, handleRedirectToRoot);
  server.onNotFound(handleRedirectToRoot);

  server.begin();
  resetPid();

  Serial.print("AP IP: ");
  Serial.println(WiFi.softAPIP());
  Serial.printf("PID: Kp = %.3f, Ki = %.5f, Kd = %.3f\n", pidKp, pidKi, pidKd);
}

void loop() {
  dnsServer.processNextRequest();
  server.handleClient();
  updateDI6State();
  updateMotorControl();
  mb.task();
  handleModbusCommand();
  applyModbusWritableRegisters();

  uint32_t now = millis();
  if ((now - lastSensorReadMs) >= SENSOR_READ_INTERVAL_MS || lastSensorReadMs == 0) {
    lastSensorReadMs = now;
    ad8495TemperatureC = readAD8495C();
    float rawTemperatureC = readStableMAX31865C(thermoOpenCircuit);
    if (!thermoOpenCircuit && !isnan(rawTemperatureC)) {
      if (isnan(lastTemperatureC)) {
        lastTemperatureC = rawTemperatureC;
      } else {
        lastTemperatureC = (TEMP_FILTER_ALPHA * rawTemperatureC) + ((1.0f - TEMP_FILTER_ALPHA) * lastTemperatureC);
      }
    } else {
      lastTemperatureC = NAN;
    }

    if (thermoOpenCircuit) {
      stopAutotune("Остановлен из-за ошибки датчика", false);
      if (faultText.length() == 0) {
        stopHeater("RTD датчик не подключен/ошибка MAX31865");
      } else {
        stopHeater(faultText);
      }
      Serial.print("MAX31865: ");
      Serial.println(faultText);
    } else if (!isnan(lastTemperatureC)) {
      Serial.printf(
        "T = %.2f C, target = %.1f C, power = %.1f%%, Kp = %.3f, Ki = %.5f, Kd = %.3f\n",
        lastTemperatureC,
        targetTemperatureC,
        heaterPowerPercent,
        pidKp,
        pidKi,
        pidKd
      );
    }
  }

  updateAutotune(now);
  computePid(now);
  applyHeaterOutput();
  syncModbusRegistersFromState();
  server.handleClient();
}



