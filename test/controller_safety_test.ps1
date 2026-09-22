param([string]$Compiler = "g++")
$ErrorActionPreference = "Stop"
$root = Split-Path -Parent $PSScriptRoot
$source = Get-Content -Raw -Encoding UTF8 (Join-Path $root "src/main.cpp")

# Compile the actual controller functions, not a second implementation.
function Get-ControllerFunction([string]$name) {
    $pattern = "(?ms)^(?:void|float|uint16_t) " + [regex]::Escape($name) + "\([^;{]*\) \{.*?^\}"
    $match = [regex]::Match($source, $pattern)
    if (-not $match.Success) { throw "Function not found: $name" }
    return $match.Value
}

$fixture = @'
#include "ThermostatAnticipation.h"
#include <assert.h>
#include <stdio.h>
#include <string>
using String = std::string;
enum { REG_TARGET_X10 = 2, REG_HEATING_ENABLE = 11, MOTOR_STOP = 0 };
const uint8_t HEATER_CONTACTOR_RELAY_PIN = 3;
const float MIN_TARGET_C = 10, MAX_TARGET_C = 300;
uint16_t modbusRegs[32] = {};
bool heatingEnabled = false, heaterContactorOn = false;
bool sensorFault = false, faultActive = false, anticipationEnabled = true;
String faultText;
int currentDirection = MOTOR_STOP;
float lastTemperatureC = 20, lastRawTemperatureC = 20;
float targetTemperatureC = 100, thermostatHysteresisC = 2, emergencyTemperatureC = 300;
uint16_t thermostatMinOnSeconds = 10, thermostatMinOffSeconds = 10;
uint32_t contactorLastChangeMs = 0, lastSensorReadMs = 0, clockMs = 0;
bool relayEnabled = false;
int relayWrites = 0;
ThermostatAnticipation anticipation;
uint32_t millis() { return clockMs; }
void setRelayPin(uint8_t pin, bool enabled) {
  assert(pin == HEATER_CONTACTOR_RELAY_PIN);
  relayEnabled = enabled;
  ++relayWrites;
}
'@

$functions = @(
    "floatToI16Reg", "clampFloat", "setHeaterContactor", "stopHeater",
    "setHeatingEnabled", "setTargetTemperature", "contactorLockSeconds", "updateThermostat"
)
foreach ($name in $functions) {
    $fixture += [Environment]::NewLine + (Get-ControllerFunction $name) + [Environment]::NewLine
}

$fixture += @'
void resetFixture() {
  anticipation.reset();
  heatingEnabled = heaterContactorOn = relayEnabled = faultActive = sensorFault = false;
  anticipationEnabled = true;
  targetTemperatureC = 100;
  thermostatHysteresisC = 2;
  emergencyTemperatureC = 300;
  lastTemperatureC = lastRawTemperatureC = 20;
  clockMs = contactorLastChangeMs = lastSensorReadMs = 0;
  relayWrites = 0;
  faultText.clear();
}
void tick(uint32_t now, float temperature) {
  clockMs = lastSensorReadMs = now;
  lastTemperatureC = lastRawTemperatureC = temperature;
  updateThermostat(now);
}
void startHeating() {
  resetFixture();
  setHeatingEnabled(true);
  tick(10000, 20);
  assert(heaterContactorOn && relayEnabled);
}
int main() {
  resetFixture();
  setHeatingEnabled(true);
  tick(9999, 20);
  assert(!heaterContactorOn);
  assert(contactorLockSeconds(clockMs) == 1);
  tick(10000, 20);
  assert(heaterContactorOn);
  setHeatingEnabled(true); // Repeated ON must not restart the lock.
  assert(contactorLastChangeMs == 10000);
  tick(19999, 101);
  assert(heaterContactorOn);
  tick(20000, 101);
  assert(!heaterContactorOn);
  tick(29999, 90);
  assert(!heaterContactorOn);
  tick(30000, 90);
  assert(heaterContactorOn);
  assert(relayWrites == 3);

  startHeating();
  clockMs = 10001;
  setHeatingEnabled(false); // Manual OFF bypasses minimum ON.
  assert(!heatingEnabled && !heaterContactorOn && !relayEnabled);
  setHeatingEnabled(false);
  assert(contactorLastChangeMs == 10001);
  assert(relayWrites == 2);
  setHeatingEnabled(true);
  tick(10002, 20);
  assert(!heaterContactorOn); // Rapid restart still respects minimum OFF.
  tick(20001, 20);
  assert(heaterContactorOn);

  startHeating();
  clockMs = lastSensorReadMs = 10001;
  lastTemperatureC = 95;
  lastRawTemperatureC = 121;
  emergencyTemperatureC = 120;
  updateThermostat(clockMs);
  assert(faultActive && !heaterContactorOn && !relayEnabled);
  tick(30000, 20);
  assert(!heaterContactorOn); // Latched emergency cannot restart by itself.

  startHeating();
  sensorFault = true;
  tick(10001, 20);
  assert(!heaterContactorOn && !relayEnabled);
  startHeating();
  tick(10001, NAN);
  assert(!heaterContactorOn);
  startHeating();
  clockMs = 13001; // No new sample.
  updateThermostat(clockMs);
  assert(!heaterContactorOn);

  resetFixture();
  anticipationEnabled = false;
  setHeatingEnabled(true);
  tick(10000, 99);
  assert(!heaterContactorOn);
  tick(10001, 98);
  assert(heaterContactorOn);
  tick(20001, 99.9f);
  assert(heaterContactorOn);
  tick(20002, 100);
  assert(!heaterContactorOn);

  startHeating();
  for (uint32_t t = 10000; t <= 30000; t += 500)
    anticipation.sample(70 + (t - 10000) * 0.0005f, t);
  assert(anticipation.rateCPerSecond() > 0);
  setTargetTemperature(110);
  assert(anticipation.rateCPerSecond() == 0);
  assert(modbusRegs[REG_TARGET_X10] == 1100);

  resetFixture();
  contactorLastChangeMs = UINT32_MAX - 5000;
  assert(contactorLockSeconds(4998) == 1);
  assert(contactorLockSeconds(4999) == 0);
  puts("Controller safety and relay-lock tests passed");
}
'@

$build = Join-Path $root ".pio/host-tests"
New-Item -ItemType Directory -Force -Path $build | Out-Null
$cpp = Join-Path $build "controller_safety_test.cpp"
$exe = Join-Path $build "controller_safety_test.exe"
[IO.File]::WriteAllText($cpp, $fixture, (New-Object Text.UTF8Encoding($false)))
& $Compiler "-std=c++11" "-Wall" "-Wextra" "-pedantic" "-I" (Join-Path $root "include") $cpp "-o" $exe
if ($LASTEXITCODE -ne 0) { throw "Host compilation failed" }
& $exe
if ($LASTEXITCODE -ne 0) { throw "Controller safety test failed" }
