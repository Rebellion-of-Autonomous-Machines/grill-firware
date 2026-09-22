#include <WiFi.h>
#include <SPI.h>
#include <Ethernet.h>
#include <Wire.h>
#include <WebServer.h>
#include <DNSServer.h>
#include <Preferences.h>
#include <math.h>
#include "ThermostatAnticipation.h"

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
static const uint8_t HEATER_CONTACTOR_RELAY_PIN = 3; // Physical relay 4, active-low.
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

// --- Ethernet / Modbus TCP (W5500) ---
static const uint8_t W5500_CS_PIN = 5;
static const uint16_t MODBUS_TCP_PORT = 502;
static const uint8_t MODBUS_UNIT_ID = 1;
byte ethernetMac[] = {0x02, 0xA0, 0xC9, 0x00, 0x00, 0x62};
IPAddress ethernetIp(192, 168, 1, 51);
IPAddress ethernetDns(192, 168, 1, 1);
IPAddress ethernetGateway(192, 168, 1, 1);
IPAddress ethernetSubnet(255, 255, 255, 0);
bool ethernetUseDhcp = false;

class Esp32EthernetServer : public EthernetServer {
 public:
  explicit Esp32EthernetServer(uint16_t port) : EthernetServer(port) {}

  void begin(uint16_t port = 0) override {
    (void)port;
    EthernetServer::begin();
  }
};

Esp32EthernetServer modbusServer(MODBUS_TCP_PORT);
EthernetClient modbusClient;
String networkSettingsMessage;
bool networkSettingsSaved = false;

enum ModbusReg : uint16_t {
  REG_STATUS_BITS = 0,        // RO bits: 0 enabled, 1 contactor on, 3 fault, 4 sensor fault, 5 lid closed
  REG_TEMPERATURE_X10 = 1,    // RO int16
  REG_TARGET_X10 = 2,         // RW int16
  REG_HYSTERESIS_X10 = 3,     // RW uint16
  REG_MIN_ON_SECONDS = 4,     // RW uint16
  REG_MIN_OFF_SECONDS = 5,    // RW uint16
  REG_EMERGENCY_X10 = 6,      // RW uint16
  REG_COMMAND = 7,            // RW write command, firmware resets to 0 after processing
  REG_COMMAND_RESULT = 8,     // RO command result code
  REG_RELAY_LOCK_SECONDS = 9, // RO seconds until contactor may switch
  REG_FAULT_CODE = 10,        // RO 0 none, 1 sensor range, 2 emergency
  REG_HEATING_ENABLE = 11,    // RW 0/1
  REG_SENSOR_MV = 12,         // RO AD8495 output in mV
  REG_MOTOR_DIRECTION = 13,   // RO 0 stop, 1 open, 2 close
  REG_MOTOR_OPEN_REMAINING_SEC = 14, // RO
  REG_ANTICIPATION_ENABLE = 15, // RW 0/1
  REG_RATE_C_MIN_X100 = 16,    // RO int16
  REG_ON_THRESHOLD_X10 = 17,  // RO int16
  REG_OFF_THRESHOLD_X10 = 18, // RO int16
  REG_OFF_LOOKAHEAD_X10 = 19, // RO seconds x10
  REG_ON_LOOKAHEAD_X10 = 20,  // RO seconds x10
  REG_LEARNED_CYCLES = 21,    // RO completed extrema
  REG_REG_COUNT = 32
};

enum ModbusCommand : uint16_t {
  CMD_NONE = 0,
  CMD_HEAT_ON = 1,
  CMD_HEAT_OFF = 2,
  CMD_CLEAR_FAULT = 5,
  CMD_SAVE_THERMOSTAT = 6,
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

uint16_t modbusRegs[REG_REG_COUNT] = {0};
uint16_t commandResult = CMD_RES_OK;

// --- Temperature / control state ---
float lastTemperatureC = NAN;
float lastRawTemperatureC = NAN;
float ad8495VoltageMv = NAN;
bool sensorFault = true;
bool heaterContactorOn = false;
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
float targetTemperatureC = ThermostatDefaults::targetC;
float thermostatHysteresisC = ThermostatDefaults::hysteresisC;
uint16_t thermostatMinOnSeconds = ThermostatDefaults::minOnSeconds;
uint16_t thermostatMinOffSeconds = ThermostatDefaults::minOffSeconds;
bool anticipationEnabled = true;
ThermostatAnticipation anticipation;
float emergencyTemperatureC = 300.0f;
uint32_t contactorLastChangeMs = 0;

static const uint32_t SENSOR_READ_INTERVAL_MS = 500;
static const float MIN_TARGET_C = 10.0f;
static const float MAX_TARGET_C = 300.0f;
static const float SENSOR_MIN_C = 10.0f;
static const float SENSOR_MAX_C = 300.0f;
static const float MIN_HYSTERESIS_C = 0.5f;
static const float MAX_HYSTERESIS_C = 50.0f;
static const uint16_t MAX_MIN_SWITCH_SECONDS = 3600;
static const float TEMP_FILTER_ALPHA = 0.25f;

void setTargetTemperature(float targetC);
void saveThermostatSettings();
float clampFloat(float value, float low, float high);
void syncModbusRegistersFromState();
void handleModbusTcp();

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

bool parseIPv4(const String& text, IPAddress& address) {
  String value = text;
  value.trim();
  return address.fromString(value);
}

void loadNetworkSettings() {
  Preferences networkPreferences;
  if (!networkPreferences.begin("ethernet", true)) {
    return;
  }

  IPAddress address;
  if (parseIPv4(networkPreferences.getString("ip", ethernetIp.toString()), address)) ethernetIp = address;
  if (parseIPv4(networkPreferences.getString("subnet", ethernetSubnet.toString()), address)) ethernetSubnet = address;
  if (parseIPv4(networkPreferences.getString("gateway", ethernetGateway.toString()), address)) ethernetGateway = address;
  if (parseIPv4(networkPreferences.getString("dns", ethernetDns.toString()), address)) ethernetDns = address;
  ethernetUseDhcp = networkPreferences.getBool("dhcp", false);
  networkPreferences.end();
}

bool saveNetworkSettings() {
  Preferences networkPreferences;
  if (!networkPreferences.begin("ethernet", false)) {
    return false;
  }
  bool saved = networkPreferences.putString("ip", ethernetIp.toString()) > 0;
  saved &= networkPreferences.putString("subnet", ethernetSubnet.toString()) > 0;
  saved &= networkPreferences.putString("gateway", ethernetGateway.toString()) > 0;
  saved &= networkPreferences.putString("dns", ethernetDns.toString()) > 0;
  saved &= networkPreferences.putBool("dhcp", ethernetUseDhcp) > 0;
  networkPreferences.end();
  return saved;
}

void applyEthernetSettings() {
  if (modbusClient) modbusClient.stop();
  if (ethernetUseDhcp) {
    Ethernet.begin(ethernetMac, 10000, 2000);
  } else {
    Ethernet.begin(ethernetMac, ethernetIp, ethernetDns, ethernetGateway, ethernetSubnet);
  }
  delay(200);
  modbusServer.begin();
}

const char* ethernetConnectionText() {
  if (Ethernet.hardwareStatus() == EthernetNoHardware) return "W5500 не обнаружен";
  if (Ethernet.linkStatus() == LinkOFF) return "Сетевой кабель отключен";
  if (Ethernet.linkStatus() == Unknown) return "Состояние линии неизвестно";
  if (Ethernet.localIP() == IPAddress(0, 0, 0, 0)) {
    return ethernetUseDhcp ? "Адрес DHCP не получен" : "IP не назначен";
  }
  return "Подключено";
}

String modbusClientText() {
  if (!modbusClient || !modbusClient.connected()) return "Нет подключения";
  return modbusClient.remoteIP().toString() + ":" + String(modbusClient.remotePort());
}

void loadThermostatSettings() {
  if (!preferences.begin("thermostat", true)) {
    saveThermostatSettings();
    return;
  }
  bool migrateDefaults = preferences.getUChar("version", 1) < 2;
  targetTemperatureC = preferences.getFloat("target", targetTemperatureC);
  thermostatHysteresisC = preferences.getFloat("hyst", thermostatHysteresisC);
  thermostatMinOnSeconds = preferences.getUShort("min_on", thermostatMinOnSeconds);
  thermostatMinOffSeconds = preferences.getUShort("min_off", thermostatMinOffSeconds);
  emergencyTemperatureC = preferences.getFloat("emergency", emergencyTemperatureC);
  anticipationEnabled = preferences.getBool("anticipation", true);
  preferences.end();

  // Migrate only the previous factory hysteresis; preserve custom settings.
  if (migrateDefaults && fabsf(thermostatHysteresisC - 5.0f) < 0.01f) {
    thermostatHysteresisC = ThermostatDefaults::hysteresisC;
  }
  if (!isfinite(targetTemperatureC)) targetTemperatureC = ThermostatDefaults::targetC;
  if (!isfinite(thermostatHysteresisC)) thermostatHysteresisC = ThermostatDefaults::hysteresisC;
  if (!isfinite(emergencyTemperatureC)) emergencyTemperatureC = SENSOR_MAX_C;
  targetTemperatureC = clampFloat(targetTemperatureC, MIN_TARGET_C, MAX_TARGET_C);
  thermostatHysteresisC = clampFloat(thermostatHysteresisC, MIN_HYSTERESIS_C, MAX_HYSTERESIS_C);
  thermostatMinOnSeconds = min(thermostatMinOnSeconds, MAX_MIN_SWITCH_SECONDS);
  thermostatMinOffSeconds = min(thermostatMinOffSeconds, MAX_MIN_SWITCH_SECONDS);
  emergencyTemperatureC = clampFloat(emergencyTemperatureC, targetTemperatureC, SENSOR_MAX_C);
  if (migrateDefaults) saveThermostatSettings();
}

void saveThermostatSettings() {
  if (!preferences.begin("thermostat", false)) return;
  preferences.putFloat("target", targetTemperatureC);
  preferences.putFloat("hyst", thermostatHysteresisC);
  preferences.putUShort("min_on", thermostatMinOnSeconds);
  preferences.putUShort("min_off", thermostatMinOffSeconds);
  preferences.putFloat("emergency", emergencyTemperatureC);
  preferences.putBool("anticipation", anticipationEnabled);
  preferences.putUChar("version", 2);
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
  heaterContactorOn = false;
  currentDirection = MOTOR_STOP;
  motorRunStartMs = 0;
}

void setHeaterContactor(bool enabled) {
  if (heaterContactorOn == enabled) {
    return;
  }
  heaterContactorOn = enabled;
  contactorLastChangeMs = millis();
  setRelayPin(HEATER_CONTACTOR_RELAY_PIN, enabled);
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

void stopHeater(const String& reason) {
  anticipation.resetSamples();
  setHeaterContactor(false);

  if (reason.length() > 0) {
    faultActive = true;
    faultText = reason;
  }
}

void setHeatingEnabled(bool enabled) {
  if (heatingEnabled != enabled) anticipation.reset();
  heatingEnabled = enabled;
  modbusRegs[REG_HEATING_ENABLE] = heatingEnabled ? 1 : 0;

  if (!heatingEnabled) {
    stopHeater("");
  }
}

void setTargetTemperature(float targetC) {
  if (fabsf(targetC - targetTemperatureC) > 0.01f) anticipation.reset();
  targetTemperatureC = clampFloat(targetC, MIN_TARGET_C, MAX_TARGET_C);
  modbusRegs[REG_TARGET_X10] = floatToI16Reg(targetTemperatureC, 10.0f);
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

uint16_t contactorLockSeconds(uint32_t nowMs) {
  uint32_t minimumMs = (heaterContactorOn ? thermostatMinOnSeconds : thermostatMinOffSeconds) * 1000UL;
  uint32_t elapsedMs = nowMs - contactorLastChangeMs;
  if (elapsedMs >= minimumMs) {
    return 0;
  }
  return static_cast<uint16_t>((minimumMs - elapsedMs + 999UL) / 1000UL);
}

void updateThermostat(uint32_t nowMs) {
  if (!heatingEnabled || faultActive || sensorFault ||
      !isfinite(lastTemperatureC) || !isfinite(lastRawTemperatureC) ||
      nowMs - lastSensorReadMs > 3000UL) {
    anticipation.resetSamples();
    setHeaterContactor(false);
    return;
  }

  // The emergency limit must not wait for the display filter or relay lock.
  if (lastRawTemperatureC >= emergencyTemperatureC || lastTemperatureC >= emergencyTemperatureC) {
    stopHeater("Аварийная температура");
    return;
  }

  if (contactorLockSeconds(nowMs) != 0) {
    return;
  }

  bool requestedOn = heaterContactorOn;
  if (anticipationEnabled) {
    requestedOn = anticipation.demand(heaterContactorOn, lastTemperatureC,
                                      targetTemperatureC, thermostatHysteresisC);
  } else if (heaterContactorOn && lastTemperatureC >= targetTemperatureC) {
    requestedOn = false;
  } else if (!heaterContactorOn && lastTemperatureC <= targetTemperatureC - thermostatHysteresisC) {
    requestedOn = true;
  }
  if (requestedOn != heaterContactorOn) {
    setHeaterContactor(requestedOn);
    if (anticipationEnabled && currentDirection == MOTOR_STOP) {
      anticipation.switched(requestedOn, lastTemperatureC, nowMs);
    }
  }
}

String formatTemperatureValue() {
  if (sensorFault) {
    return "Ошибка";
  }

  if (isnan(lastTemperatureC)) {
    return "Нет данных";
  }

  char buff[24];
  snprintf(buff, sizeof(buff), "%.2f", lastTemperatureC);
  return String(buff);
}

String boolToJson(bool value) {
  return value ? "true" : "false";
}

float thermostatOnThreshold() {
  return anticipationEnabled ? anticipation.onThreshold(targetTemperatureC, thermostatHysteresisC)
                             : targetTemperatureC - thermostatHysteresisC;
}

float thermostatOffThreshold() {
  return anticipationEnabled ? anticipation.offThreshold(targetTemperatureC) : targetTemperatureC;
}

String statusJson() {
  String json = "{";
  json += "\"temperature\":";
  json += isnan(lastTemperatureC) ? "null" : String(lastTemperatureC, 2);
  json += ",\"ad8495_mv\":";
  json += isnan(ad8495VoltageMv) ? "null" : String(ad8495VoltageMv, 1);
  json += ",\"target\":" + String(targetTemperatureC, 1);
  json += ",\"hysteresis\":" + String(thermostatHysteresisC, 1);
  json += ",\"anticipation_enabled\":" + boolToJson(anticipationEnabled);
  json += ",\"temperature_rate\":" + String(anticipation.rateCPerSecond() * 60.0f, 2);
  json += ",\"on_threshold\":" + String(thermostatOnThreshold(), 1);
  json += ",\"off_threshold\":" + String(thermostatOffThreshold(), 1);
  json += ",\"off_lookahead_seconds\":" + String(anticipation.offSeconds(), 1);
  json += ",\"on_lookahead_seconds\":" + String(anticipation.onSeconds(), 1);
  json += ",\"learned_cycles\":" + String(anticipation.learnedCycles());
  json += ",\"min_on_seconds\":" + String(thermostatMinOnSeconds);
  json += ",\"min_off_seconds\":" + String(thermostatMinOffSeconds);
  json += ",\"emergency_temperature\":" + String(emergencyTemperatureC, 1);
  json += ",\"relay_lock_seconds\":" + String(contactorLockSeconds(millis()));
  json += ",\"heater_on\":" + boolToJson(heaterContactorOn);
  json += ",\"heating_enabled\":" + boolToJson(heatingEnabled);
  json += ",\"sensor_fault\":" + boolToJson(sensorFault);
  json += ",\"fault\":" + boolToJson(faultActive || sensorFault);
  json += ",\"fault_text\":\"" + faultText + "\"";
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
  if (heaterContactorOn) {
    statusBits |= (1U << 1);
  }
  if (faultActive) {
    statusBits |= (1U << 3);
  }
  if (sensorFault) {
    statusBits |= (1U << 4);
  }
  if (di6Active) {
    statusBits |= (1U << 5);
  }

  uint16_t faultCode = 0;
  if (sensorFault) {
    faultCode = 1;
  } else if (faultActive) {
    faultCode = 2;
  }

  modbusRegs[REG_STATUS_BITS] = statusBits;
  modbusRegs[REG_TEMPERATURE_X10] = isnan(lastTemperatureC) ? static_cast<uint16_t>(0x8000) : floatToI16Reg(lastTemperatureC, 10.0f);
  modbusRegs[REG_TARGET_X10] = floatToI16Reg(targetTemperatureC, 10.0f);
  modbusRegs[REG_HYSTERESIS_X10] = floatToU16(thermostatHysteresisC, 10.0f);
  modbusRegs[REG_MIN_ON_SECONDS] = thermostatMinOnSeconds;
  modbusRegs[REG_MIN_OFF_SECONDS] = thermostatMinOffSeconds;
  modbusRegs[REG_EMERGENCY_X10] = floatToU16(emergencyTemperatureC, 10.0f);
  modbusRegs[REG_COMMAND] = CMD_NONE;
  modbusRegs[REG_COMMAND_RESULT] = commandResult;
  modbusRegs[REG_RELAY_LOCK_SECONDS] = contactorLockSeconds(millis());
  modbusRegs[REG_FAULT_CODE] = faultCode;
  modbusRegs[REG_HEATING_ENABLE] = heatingEnabled ? 1 : 0;
  modbusRegs[REG_SENSOR_MV] = isnan(ad8495VoltageMv) ? 0 : floatToU16(ad8495VoltageMv, 1.0f);
  modbusRegs[REG_MOTOR_DIRECTION] = static_cast<uint16_t>(currentDirection);
  modbusRegs[REG_MOTOR_OPEN_REMAINING_SEC] = openRemainingSeconds();
  modbusRegs[REG_ANTICIPATION_ENABLE] = anticipationEnabled ? 1 : 0;
  modbusRegs[REG_RATE_C_MIN_X100] = floatToI16Reg(anticipation.rateCPerSecond() * 60.0f, 100.0f);
  modbusRegs[REG_ON_THRESHOLD_X10] = floatToI16Reg(thermostatOnThreshold(), 10.0f);
  modbusRegs[REG_OFF_THRESHOLD_X10] = floatToI16Reg(thermostatOffThreshold(), 10.0f);
  modbusRegs[REG_OFF_LOOKAHEAD_X10] = floatToU16(anticipation.offSeconds(), 10.0f);
  modbusRegs[REG_ON_LOOKAHEAD_X10] = floatToU16(anticipation.onSeconds(), 10.0f);
  modbusRegs[REG_LEARNED_CYCLES] = anticipation.learnedCycles();
}

void executeModbusCommand(uint16_t cmd) {
  commandResult = CMD_RES_OK;
  switch (cmd) {
    case CMD_NONE:
      break;
    case CMD_HEAT_ON:
      faultActive = false;
      faultText = sensorFault ? "Температура AD8495 вне диапазона 10...300 °C" : "";
      setHeatingEnabled(true);
      break;
    case CMD_HEAT_OFF:
      setHeatingEnabled(false);
      break;
    case CMD_CLEAR_FAULT:
      faultActive = false;
      faultText = sensorFault ? "Температура AD8495 вне диапазона 10...300 °C" : "";
      break;
    case CMD_SAVE_THERMOSTAT:
      saveThermostatSettings();
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
      commandResult = CMD_RES_UNKNOWN_CMD;
      break;
  }
  syncModbusRegistersFromState();
}

bool writeModbusRegister(uint16_t address, uint16_t value) {
  if (address >= REG_REG_COUNT) return false;
  commandResult = CMD_RES_OK;

  switch (address) {
    case REG_COMMAND:
      executeModbusCommand(value);
      return true;
    case REG_TARGET_X10:
      setTargetTemperature(clampFloat(i16RegToFloat(value, 10.0f), MIN_TARGET_C, MAX_TARGET_C));
      if (emergencyTemperatureC < targetTemperatureC) emergencyTemperatureC = targetTemperatureC;
      saveThermostatSettings();
      break;
    case REG_HYSTERESIS_X10:
      thermostatHysteresisC = clampFloat(value / 10.0f, MIN_HYSTERESIS_C, MAX_HYSTERESIS_C);
      anticipation.cancelCycle();
      saveThermostatSettings();
      break;
    case REG_MIN_ON_SECONDS:
      thermostatMinOnSeconds = min(value, MAX_MIN_SWITCH_SECONDS);
      anticipation.cancelCycle();
      saveThermostatSettings();
      break;
    case REG_MIN_OFF_SECONDS:
      thermostatMinOffSeconds = min(value, MAX_MIN_SWITCH_SECONDS);
      anticipation.cancelCycle();
      saveThermostatSettings();
      break;
    case REG_EMERGENCY_X10:
      emergencyTemperatureC = clampFloat(value / 10.0f, targetTemperatureC, SENSOR_MAX_C);
      saveThermostatSettings();
      break;
    case REG_HEATING_ENABLE:
      setHeatingEnabled(value != 0);
      break;
    case REG_ANTICIPATION_ENABLE:
      if (value > 1) {
        commandResult = CMD_RES_INVALID_ARG;
        break;
      }
      if (anticipationEnabled != (value != 0)) anticipation.reset();
      anticipationEnabled = value != 0;
      saveThermostatSettings();
      break;
    default:
      return false;
  }
  syncModbusRegistersFromState();
  return true;
}

void sendModbusTcpResponse(EthernetClient& client, const uint8_t* requestHeader, const uint8_t* pdu, uint16_t pduLen) {
  uint8_t header[7] = {requestHeader[0], requestHeader[1], 0, 0, highByte(pduLen + 1), lowByte(pduLen + 1), requestHeader[6]};
  client.write(header, sizeof(header));
  client.write(pdu, pduLen);
}

void sendModbusException(EthernetClient& client, const uint8_t* header, uint8_t functionCode, uint8_t exceptionCode) {
  uint8_t pdu[2] = {static_cast<uint8_t>(functionCode | 0x80), exceptionCode};
  sendModbusTcpResponse(client, header, pdu, sizeof(pdu));
}

bool waitForClientBytes(EthernetClient& client, int count, uint16_t timeoutMs = 50) {
  uint32_t started = millis();
  while (client.connected() && client.available() < count && millis() - started < timeoutMs) delay(1);
  return client.available() >= count;
}

void processModbusTcpRequest(EthernetClient& client) {
  if (!waitForClientBytes(client, 7)) return;
  uint8_t header[7];
  client.read(header, sizeof(header));
  uint16_t protocolId = word(header[2], header[3]);
  uint16_t length = word(header[4], header[5]);
  if (protocolId != 0 || length < 2 || length > 253) { client.stop(); return; }

  uint16_t pduLen = length - 1;
  if (!waitForClientBytes(client, pduLen)) { client.stop(); return; }
  uint8_t pdu[253];
  client.read(pdu, pduLen);
  uint8_t functionCode = pdu[0];
  if (header[6] != MODBUS_UNIT_ID && header[6] != 0) return;
  syncModbusRegistersFromState();

  if (functionCode == 0x03) {
    if (pduLen < 5) { sendModbusException(client, header, functionCode, 0x03); return; }
    uint16_t startAddress = word(pdu[1], pdu[2]);
    uint16_t quantity = word(pdu[3], pdu[4]);
    if (quantity == 0 || quantity > 60 || startAddress + quantity > REG_REG_COUNT) {
      sendModbusException(client, header, functionCode, 0x02); return;
    }
    uint8_t response[125];
    response[0] = functionCode;
    response[1] = quantity * 2;
    for (uint16_t i = 0; i < quantity; i++) {
      uint16_t registerValue = modbusRegs[startAddress + i];
      response[2 + i * 2] = highByte(registerValue);
      response[3 + i * 2] = lowByte(registerValue);
    }
    sendModbusTcpResponse(client, header, response, 2 + quantity * 2);
    return;
  }

  if (functionCode == 0x06) {
    if (pduLen < 5) { sendModbusException(client, header, functionCode, 0x03); return; }
    uint16_t address = word(pdu[1], pdu[2]);
    uint16_t value = word(pdu[3], pdu[4]);
    if (!writeModbusRegister(address, value)) { sendModbusException(client, header, functionCode, 0x02); return; }
    sendModbusTcpResponse(client, header, pdu, 5);
    return;
  }

  if (functionCode == 0x10) {
    if (pduLen < 6) { sendModbusException(client, header, functionCode, 0x03); return; }
    uint16_t startAddress = word(pdu[1], pdu[2]);
    uint16_t quantity = word(pdu[3], pdu[4]);
    uint8_t byteCount = pdu[5];
    if (quantity == 0 || byteCount != quantity * 2 || startAddress + quantity > REG_REG_COUNT || pduLen < 6 + byteCount) {
      sendModbusException(client, header, functionCode, 0x02); return;
    }
    for (uint16_t i = 0; i < quantity; i++) {
      uint16_t value = word(pdu[6 + i * 2], pdu[7 + i * 2]);
      if (!writeModbusRegister(startAddress + i, value)) { sendModbusException(client, header, functionCode, 0x02); return; }
    }
    uint8_t response[5] = {functionCode, pdu[1], pdu[2], pdu[3], pdu[4]};
    sendModbusTcpResponse(client, header, response, sizeof(response));
    return;
  }

  sendModbusException(client, header, functionCode, 0x01);
}

void handleModbusTcp() {
  if (!modbusClient || !modbusClient.connected()) {
    EthernetClient newClient = modbusServer.available();
    if (newClient) modbusClient = newClient;
  }
  if (modbusClient && modbusClient.connected() && modbusClient.available()) processModbusTcpRequest(modbusClient);
}

String generateHTML() {
  String html = R"rawliteral(
<!doctype html>
<html lang="ru">
<head>
  <meta charset="utf-8">
  <meta name="viewport" content="width=device-width,initial-scale=1">
  <title>Термостат гриля</title>
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
    .settings-form {
      grid-template-columns: 1fr 1fr;
      padding: 14px;
      border: 1px solid var(--line);
      border-radius: 8px;
      background: #fbfdfb;
    }
    .field { display: grid; gap: 6px; }
    .field label { color: var(--muted); font-size: 13px; font-weight: 700; }
    .settings-form button { grid-column: 1 / -1; }
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
    .network { margin-top: 18px; padding-top: 14px; border-top: 1px solid var(--line); }
    .network h2 { margin: 0 0 10px; font-size: 18px; }
    .network-state { margin: 5px 0; color: var(--muted); font-size: 14px; }
    .notice { margin: 10px 0; padding: 10px; border-radius: 8px; }
    .notice-ok { background: #e7f7ef; color: #08734d; }
    .notice-error { background: #fdecea; color: var(--warn); }
    select { border: 1px solid var(--line); border-radius: 8px; padding: 12px; font-size: 16px; }
    @media (max-width: 460px) {
      .grid, form, .actions, .settings-form { grid-template-columns: 1fr; }
      .settings-form button { grid-column: 1; }
    }
  </style>
</head>
<body>
  <main class="panel">
    <h1>Управление ТЭНом</h1>
    <p class="sub">Термостат управляет контактором ТЭНа через реле 4.</p>
    <section class="grid">
      <div class="cell">
        <div class="label">Температура AD8495</div>
        <div class="value"><span id="temperature">)rawliteral";

  html += formatTemperatureValue();

  html += R"rawliteral(</span> <span class="unit">°C</span></div>
      </div>
      <div class="cell">
        <div class="label">Уставка</div>
        <div class="value"><span id="target">)rawliteral";

  html += String(targetTemperatureC, 1);

  html += R"rawliteral(</span> <span class="unit">°C</span></div>
      </div>
    </section>
    <form class="settings-form" action="/thermostat/set" method="get">
      <div class="field">
        <label for="targetInput">Уставка, °C</label>
        <input id="targetInput" name="target" type="number" step="0.5" min="10" max="300" value=")rawliteral";

  html += String(targetTemperatureC, 1);

  html += R"rawliteral(">
      </div>
      <div class="field">
        <label for="hysteresisInput">Гистерезис, °C</label>
        <input id="hysteresisInput" name="hysteresis" type="number" step="0.1" min="0.5" max="50" value=")rawliteral";

  html += String(thermostatHysteresisC, 1);

  html += R"rawliteral(">
      </div>
      <div class="field">
        <label for="minOnInput">Минимальное время включения, с</label>
        <input id="minOnInput" name="min_on" type="number" min="0" max="3600" value=")rawliteral";

  html += String(thermostatMinOnSeconds);

  html += R"rawliteral(">
      </div>
      <div class="field">
        <label for="minOffInput">Минимальное время выключения, с</label>
        <input id="minOffInput" name="min_off" type="number" min="0" max="3600" value=")rawliteral";

  html += String(thermostatMinOffSeconds);

  html += R"rawliteral(">
      </div>
      <div class="field">
        <label for="emergencyInput">Аварийная температура, °C</label>
        <input id="emergencyInput" name="emergency" type="number" step="0.5" min="10" max="300" value=")rawliteral";

  html += String(emergencyTemperatureC, 1);

  html += R"rawliteral(">
      </div>
      <div class="field">
        <label for="anticipationInput">Упреждение и автоподстройка</label>
        <select id="anticipationInput" name="anticipation">
          <option value="1")rawliteral";
  if (anticipationEnabled) html += " selected";
  html += R"rawliteral(>Включены</option>
          <option value="0")rawliteral";
  if (!anticipationEnabled) html += " selected";
  html += R"rawliteral(>Выключены</option>
        </select>
      </div>
      <button id="setButton" type="submit">Сохранить параметры</button>
    </form>
    <div id="actionMessage" class="fault" role="alert"></div>
    <div class="actions">
      <form action="/heat/on" method="get">
        <button id="heatOnButton" type="submit">Включить нагрев</button>
      </form>
      <form action="/heat/off" method="get">
        <button id="heatOffButton" class="stop" type="submit">Выключить нагрев</button>
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
      <span>Контактор ТЭНа: <b id="heater">)rawliteral";

  html += heaterContactorOn ? "вкл" : "выкл";

  html += R"rawliteral(</b></span>
      <span>Нагрев: <b id="enabled">)rawliteral";

  html += heatingEnabled ? "включен" : "выключен";

  html += R"rawliteral(</b></span>
      <span>Блокировка: <b id="relayLock">)rawliteral";

  html += String(contactorLockSeconds(millis()));

  html += R"rawliteral(</b> с</span>
      <span>Мотор: <b id="motorDirection">)rawliteral";

  html += motorDirectionText();

  html += R"rawliteral(</b></span>
    </div>
    <div class="status">
      <span>Гистерезис: <b id="hysteresis">)rawliteral";

  html += String(thermostatHysteresisC, 1);

  html += R"rawliteral(</b> °C</span>
      <span>Мин. ON: <b id="minOn">)rawliteral";

  html += String(thermostatMinOnSeconds);

  html += R"rawliteral(</b> с</span>
      <span>Мин. OFF: <b id="minOff">)rawliteral";

  html += String(thermostatMinOffSeconds);

  html += R"rawliteral(</b> с</span>
      <span>Авария: <b id="emergency">)rawliteral";

  html += String(emergencyTemperatureC, 1);

  html += R"rawliteral(</b> °C</span>
      <span>AD8495: <b id="ad8495Mv">0.0</b> mV</span>
      <span>Открытие осталось: <b id="motorOpenRemaining">0</b> с</span>
    </div>
    <div class="status">
      <span>Расчетный порог включения: <b id="onThreshold">-</b> °C</span>
      <span>Расчетный порог выключения: <b id="offThreshold">-</b> °C</span>
      <span>Скорость температуры: <b id="temperatureRate">-</b> °C/мин</span>
      <span>Завершено этапов подстройки: <b id="learnedCycles">0</b></span>
    </div>
    <section class="network">
      <h2>Ethernet / Modbus TCP</h2>
)rawliteral";

  if (networkSettingsMessage.length() > 0) {
    html += "<div class=\"notice ";
    html += networkSettingsSaved ? "notice-ok" : "notice-error";
    html += "\">" + networkSettingsMessage + "</div>";
  }

  html += R"rawliteral(
      <div class="network-state">Состояние W5500: <b>)rawliteral";
  html += ethernetConnectionText();
  html += R"rawliteral(</b></div>
      <div class="network-state">Текущий IP: <b>)rawliteral";
  html += Ethernet.localIP().toString();
  html += R"rawliteral(</b></div>
      <div class="network-state">Порт Modbus TCP: <b>502</b>, Unit ID: <b>1</b></div>
      <div class="network-state">Клиент: <b>)rawliteral";
  html += modbusClientText();
  html += R"rawliteral(</b></div>
      <form id="networkForm" class="settings-form" action="/network" method="post">
        <div class="field">
          <label for="networkMode">Получение адреса</label>
          <select id="networkMode" name="mode" onchange="updateNetworkMode()">
            <option value="static")rawliteral";
  if (!ethernetUseDhcp) html += " selected";
  html += R"rawliteral(>Статический IP</option>
            <option value="dhcp")rawliteral";
  if (ethernetUseDhcp) html += " selected";
  html += R"rawliteral(>DHCP</option>
          </select>
        </div>
        <div class="field"><label for="networkIp">IP-адрес</label><input class="static-network-field" id="networkIp" name="ip" value=")rawliteral";
  html += ethernetIp.toString();
  html += R"rawliteral("></div>
        <div class="field"><label for="networkSubnet">Маска подсети</label><input class="static-network-field" id="networkSubnet" name="subnet" value=")rawliteral";
  html += ethernetSubnet.toString();
  html += R"rawliteral("></div>
        <div class="field"><label for="networkGateway">Шлюз</label><input class="static-network-field" id="networkGateway" name="gateway" value=")rawliteral";
  html += ethernetGateway.toString();
  html += R"rawliteral("></div>
        <div class="field"><label for="networkDns">DNS</label><input class="static-network-field" id="networkDns" name="dns" value=")rawliteral";
  html += ethernetDns.toString();
  html += R"rawliteral("></div>
        <button type="submit">Сохранить и применить</button>
      </form>
    </section>
    <div id="fault" class="status fault"></div>
  </main>
  <script>
    function updateNetworkMode() {
      const disabled = document.getElementById('networkMode').value === 'dhcp';
      document.querySelectorAll('.static-network-field').forEach(function (field) { field.disabled = disabled; });
    }

    async function refreshStatus() {
      const response = await fetch('/api/status', { cache: 'no-store' });
      const data = await response.json();
      document.getElementById('temperature').textContent = data.temperature === null ? 'Нет данных' : data.temperature.toFixed(2);
      document.getElementById('target').textContent = data.target.toFixed(1);
      const fields = {
        targetInput: data.target.toFixed(1),
        hysteresisInput: data.hysteresis.toFixed(1),
        minOnInput: data.min_on_seconds,
        minOffInput: data.min_off_seconds,
        emergencyInput: data.emergency_temperature.toFixed(1),
        anticipationInput: data.anticipation_enabled ? '1' : '0'
      };
      Object.entries(fields).forEach(function ([id, value]) {
        const field = document.getElementById(id);
        if (document.activeElement !== field && field.dataset.dirty !== '1') field.value = value;
      });
      document.getElementById('onThreshold').textContent = data.on_threshold.toFixed(1);
      document.getElementById('offThreshold').textContent = data.off_threshold.toFixed(1);
      document.getElementById('temperatureRate').textContent = data.temperature_rate.toFixed(2);
      document.getElementById('learnedCycles').textContent = data.learned_cycles;
      document.getElementById('hysteresis').textContent = data.hysteresis.toFixed(1);
      document.getElementById('minOn').textContent = data.min_on_seconds;
      document.getElementById('minOff').textContent = data.min_off_seconds;
      document.getElementById('emergency').textContent = data.emergency_temperature.toFixed(1);
      document.getElementById('relayLock').textContent = data.relay_lock_seconds;
      document.getElementById('heater').textContent = data.heater_on ? 'вкл' : 'выкл';
      document.getElementById('enabled').textContent = data.heating_enabled ? 'включен' : 'выключен';
      document.getElementById('motorDirection').textContent = data.motor_direction_text || 'Стоп';
      document.getElementById('ad8495Mv').textContent = data.ad8495_mv == null ? 'N/A' : Number(data.ad8495_mv).toFixed(1);
      document.getElementById('motorOpenRemaining').textContent = (data.motor_open_remaining_sec ?? 0).toString();
      document.getElementById('fault').textContent = data.fault ? data.fault_text : '';
      document.getElementById('heatOnButton').disabled = data.heating_enabled;
      document.getElementById('heatOffButton').disabled = !data.heating_enabled;
      document.getElementById('motorOpenButton').disabled = data.motor_direction === 1 || (data.motor_open_remaining_sec ?? 0) === 0;
      document.getElementById('motorCloseButton').disabled = data.motor_direction === 2 || data.di6_active;
      document.getElementById('motorStopButton').disabled = data.motor_direction === 0;
    }

    document.querySelectorAll('form:not(#networkForm)').forEach(function (form) {
      form.querySelectorAll('input,select').forEach(function (field) {
        field.addEventListener('input', function () { field.dataset.dirty = '1'; });
      });
      form.addEventListener('submit', async function (event) {
        event.preventDefault();
        const url = new URL(form.action, window.location.origin);
        new FormData(form).forEach(function (value, key) {
          url.searchParams.set(key, value);
        });
        url.searchParams.set('ajax', '1');

        const buttons = form.querySelectorAll('button');
        const sentFields = Array.from(form.querySelectorAll('input,select')).map(field => [field, field.value]);
        buttons.forEach(function (button) { button.disabled = true; });
        const message = document.getElementById('actionMessage');
        message.textContent = '';
        try {
          const response = await fetch(url.toString(), { cache: 'no-store' });
          if (!response.ok) throw new Error(await response.text());
          sentFields.forEach(function ([field, value]) {
            if (field.value === value) delete field.dataset.dirty;
          });
        } catch (error) {
          message.textContent = error.message;
        } finally {
          buttons.forEach(function (button) { button.disabled = false; });
          try { await refreshStatus(); } catch (error) { message.textContent = 'Нет связи с контроллером'; }
        }
      });
    });

    setInterval(refreshStatus, 1000);
    updateNetworkMode();
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

void handleSetThermostat() {
  if (!server.hasArg("target") || !server.hasArg("hysteresis") ||
      !server.hasArg("min_on") || !server.hasArg("min_off") || !server.hasArg("emergency")) {
    server.send(400, "text/plain; charset=UTF-8", "Не все параметры переданы");
    return;
  }

  float requestedTarget = server.arg("target").toFloat();
  float requestedHysteresis = server.arg("hysteresis").toFloat();
  long requestedMinOn = server.arg("min_on").toInt();
  long requestedMinOff = server.arg("min_off").toInt();
  float requestedEmergency = server.arg("emergency").toFloat();
  bool requestedAnticipation = anticipationEnabled;
  if (server.hasArg("anticipation")) {
    if (server.arg("anticipation") != "0" && server.arg("anticipation") != "1") {
      server.send(400, "text/plain; charset=UTF-8", "Упреждение должно быть 0 или 1");
      return;
    }
    requestedAnticipation = server.arg("anticipation") == "1";
  }
  if (!isfinite(requestedTarget) || !isfinite(requestedHysteresis) || !isfinite(requestedEmergency)) {
    server.send(400, "text/plain; charset=UTF-8", "Некорректное числовое значение");
    return;
  }
  if (requestedTarget < MIN_TARGET_C || requestedTarget > MAX_TARGET_C) {
    server.send(400, "text/plain; charset=UTF-8", "Уставка вне диапазона");
    return;
  }
  if (requestedHysteresis < MIN_HYSTERESIS_C || requestedHysteresis > MAX_HYSTERESIS_C ||
      requestedMinOn < 0 || requestedMinOn > MAX_MIN_SWITCH_SECONDS ||
      requestedMinOff < 0 || requestedMinOff > MAX_MIN_SWITCH_SECONDS ||
      requestedEmergency < requestedTarget || requestedEmergency > SENSOR_MAX_C) {
    server.send(400, "text/plain; charset=UTF-8", "Параметры термостата вне диапазона");
    return;
  }

  setTargetTemperature(requestedTarget);
  thermostatHysteresisC = requestedHysteresis;
  thermostatMinOnSeconds = static_cast<uint16_t>(requestedMinOn);
  thermostatMinOffSeconds = static_cast<uint16_t>(requestedMinOff);
  emergencyTemperatureC = requestedEmergency;
  anticipation.cancelCycle();
  if (anticipationEnabled != requestedAnticipation) anticipation.reset();
  anticipationEnabled = requestedAnticipation;
  saveThermostatSettings();

  sendActionDone();
}

void handleHeatOn() {
  faultActive = false;
  faultText = sensorFault ? "Температура AD8495 вне диапазона 10...300 °C" : "";
  setHeatingEnabled(true);
  sendActionDone();
}

void handleHeatOff() {
  setHeatingEnabled(false);
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

void handleNetworkSettings() {
  if (heatingEnabled || currentDirection != MOTOR_STOP) {
    networkSettingsSaved = false;
    networkSettingsMessage = "Настройки сети можно менять только при выключенном нагреве и остановленном моторе.";
    server.send(409, "text/html; charset=UTF-8", generateHTML());
    return;
  }
  if (!server.hasArg("mode") || (server.arg("mode") != "static" && server.arg("mode") != "dhcp")) {
    networkSettingsSaved = false;
    networkSettingsMessage = "Выберите режим получения IP-адреса.";
    server.send(400, "text/html; charset=UTF-8", generateHTML());
    return;
  }

  bool newUseDhcp = server.arg("mode") == "dhcp";
  IPAddress newIp, newSubnet, newGateway, newDns;
  if (!newUseDhcp &&
      (!server.hasArg("ip") || !server.hasArg("subnet") || !server.hasArg("gateway") || !server.hasArg("dns") ||
       !parseIPv4(server.arg("ip"), newIp) || !parseIPv4(server.arg("subnet"), newSubnet) ||
       !parseIPv4(server.arg("gateway"), newGateway) || !parseIPv4(server.arg("dns"), newDns))) {
    networkSettingsSaved = false;
    networkSettingsMessage = "Ошибка: проверьте формат IPv4-адресов.";
    server.send(400, "text/html; charset=UTF-8", generateHTML());
    return;
  }

  bool oldDhcp = ethernetUseDhcp;
  IPAddress oldIp = ethernetIp, oldSubnet = ethernetSubnet, oldGateway = ethernetGateway, oldDns = ethernetDns;
  ethernetUseDhcp = newUseDhcp;
  if (!newUseDhcp) {
    ethernetIp = newIp;
    ethernetSubnet = newSubnet;
    ethernetGateway = newGateway;
    ethernetDns = newDns;
  }
  if (!saveNetworkSettings()) {
    ethernetUseDhcp = oldDhcp;
    ethernetIp = oldIp;
    ethernetSubnet = oldSubnet;
    ethernetGateway = oldGateway;
    ethernetDns = oldDns;
    networkSettingsSaved = false;
    networkSettingsMessage = "Не удалось сохранить настройки сети.";
    server.send(500, "text/html; charset=UTF-8", generateHTML());
    return;
  }

  applyEthernetSettings();
  networkSettingsSaved = true;
  networkSettingsMessage = "Настройки сохранены. Текущий IP W5500: " + Ethernet.localIP().toString();
  handleRoot();
}

void handleRedirectToRoot() {
  server.sendHeader("Location", String("http://") + WiFi.softAPIP().toString() + "/", true);
  server.send(302, "text/plain", "");
}

void setup() {
  Serial.begin(115200);
  loadThermostatSettings();

  pinMode(ANALOG_A1, INPUT);
  analogReadResolution(12);
  analogSetPinAttenuation(ANALOG_A1, ADC_11db);

  Wire.begin(I2C_SDA_PIN, I2C_SCL_PIN);
  initInputExpander();
  setAllRelaysOff();
  updateDI6State();
  runStartupOpenCycle();

  SPI.begin();
  Ethernet.init(W5500_CS_PIN);
  loadNetworkSettings();
  applyEthernetSettings();
  syncModbusRegistersFromState();

  delay(250);

  WiFi.softAP(ap_ssid, ap_password);
  delay(200);
  dnsServer.start(DNS_PORT, "*", WiFi.softAPIP());

  server.on("/", HTTP_GET, handleRoot);
  server.on("/thermostat/set", HTTP_GET, handleSetThermostat);
  server.on("/heat/on", HTTP_GET, handleHeatOn);
  server.on("/heat/off", HTTP_GET, handleHeatOff);
  server.on("/motor/open", HTTP_GET, handleMotorOpen);
  server.on("/motor/close", HTTP_GET, handleMotorClose);
  server.on("/motor/stop", HTTP_GET, handleMotorStop);
  server.on("/api/status", HTTP_GET, handleStatus);
  server.on("/network", HTTP_POST, handleNetworkSettings);
    
  server.on("/generate_204", HTTP_GET, handleRedirectToRoot);
  server.on("/hotspot-detect.html", HTTP_GET, handleRedirectToRoot);
  server.on("/fwlink", HTTP_GET, handleRedirectToRoot);
  server.onNotFound(handleRedirectToRoot);

  server.begin();

  Serial.print("AP IP: ");
  Serial.println(WiFi.softAPIP());
  Serial.print("Ethernet IP: ");
  Serial.println(Ethernet.localIP());
  Serial.printf("Thermostat: target=%.1f C, hysteresis=%.1f C, min ON=%u s, min OFF=%u s\n",
                targetTemperatureC, thermostatHysteresisC, thermostatMinOnSeconds, thermostatMinOffSeconds);
}

void loop() {
  dnsServer.processNextRequest();
  server.handleClient();
  updateDI6State();
  updateMotorControl();
  if (ethernetUseDhcp) Ethernet.maintain();
  handleModbusTcp();

  uint32_t now = millis();
  if ((now - lastSensorReadMs) >= SENSOR_READ_INTERVAL_MS || lastSensorReadMs == 0) {
    lastSensorReadMs = now;
    float rawTemperatureC = readAD8495C();
    lastRawTemperatureC = rawTemperatureC;
    sensorFault = !isfinite(rawTemperatureC) || rawTemperatureC < SENSOR_MIN_C || rawTemperatureC > SENSOR_MAX_C;
    if (!sensorFault) {
      if (isnan(lastTemperatureC)) {
        lastTemperatureC = rawTemperatureC;
      } else {
        lastTemperatureC = (TEMP_FILTER_ALPHA * rawTemperatureC) + ((1.0f - TEMP_FILTER_ALPHA) * lastTemperatureC);
      }
      if (!faultActive) {
        faultText = "";
      }
      if (anticipationEnabled && heatingEnabled && !faultActive && currentDirection == MOTOR_STOP) {
        anticipation.sample(lastTemperatureC, millis());
      } else {
        anticipation.resetSamples();
      }
    } else {
      lastTemperatureC = NAN;
      faultText = "Температура AD8495 вне диапазона 10...300 °C";
      stopHeater("");
    }

    if (!sensorFault && !isnan(lastTemperatureC)) {
      Serial.printf("T=%.2f C, target=%.1f C, contactor=%s\n",
                    lastTemperatureC, targetTemperatureC, heaterContactorOn ? "ON" : "OFF");
    }
  }

  updateThermostat(millis());
  syncModbusRegistersFromState();
  handleModbusTcp();
  server.handleClient();
}



