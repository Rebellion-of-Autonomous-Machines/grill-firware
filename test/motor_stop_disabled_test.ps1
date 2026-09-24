param([string]$Compiler = "g++")
$ErrorActionPreference = "Stop"
$root = Split-Path -Parent $PSScriptRoot
$source = Get-Content -Raw -Encoding UTF8 (Join-Path $root "src/main.cpp")

function Extract([string]$pattern) {
    $match = [regex]::Match($source, $pattern)
    if (-not $match.Success) { throw "Source pattern not found: $pattern" }
    return $match.Value + [Environment]::NewLine
}

$fixture = @'
#include <stdint.h>
#include <assert.h>
#include <stdio.h>
#include <string>
using String = std::string;
'@
foreach ($name in @("ModbusCommand", "ModbusCommandResult", "MotorDirection")) {
    $fixture += [Environment]::NewLine + (Extract ("(?ms)^enum " + $name + " : \w+ \{.*?^\};"))
}
$fixture += @'
const uint32_t OPEN_TRAVEL_MS = 6000;
const uint16_t RELAY_DEADTIME_MS = 120;
const bool STOP_USE_NO_STATE = false;
MotorDirection currentDirection = MOTOR_STOP;
uint32_t motorRunStartMs = 0, openRemainingMs = OPEN_TRAVEL_MS, clockMs = 1000;
uint32_t closeRunDurationMs = 0;
uint16_t closingHeightMm = 60;
bool nextMotorCommandIsOpen = true;
bool startupOpeningInProgress = false;
bool di6Active = false, di6WasActive = false, relayA = false, relayB = false;
bool faultActive = false, sensorFault = false, heatingEnabled = false;
String faultText;
uint16_t commandResult = CMD_RES_OK;
int relayWrites = 0;
uint32_t millis() { return clockMs; }
void delay(uint32_t ms) { clockMs += ms; }
void setMotorRelaysRaw(bool a, bool b) {
  relayA = a; relayB = b; ++relayWrites;
}
void setHeatingEnabled(bool on) { heatingEnabled = on; }
void saveThermostatSettings() {}
void syncModbusRegistersFromState() {}
uint32_t closingTimeMsForHeight(uint16_t) { return 1300; }
struct Server {
  int code = 0;
  void send(int status, const char*, const char*) { code = status; }
} server;
'@
$fixture += "void stopMotor();" + [Environment]::NewLine
foreach ($name in @(
    "stopMotorRaw", "accountOpenElapsed", "stopMotor", "startOpenMotor",
    "processDI6Emergency",
    "startCloseMotor", "completeOpenMotor", "completeCloseMotor",
    "updateMotorControl", "executeModbusCommand", "handleMotorStopDisabled"
)) {
    $fixture += [Environment]::NewLine + (Extract ("(?ms)^void " + $name + "\([^;{]*\) \{.*?^\}"))
}
$fixture += @'
void assertManualStopRejected() {
  const auto direction = currentDirection;
  const auto started = motorRunStartMs;
  const auto remaining = openRemainingMs;
  const int writes = relayWrites;
  const bool a = relayA, b = relayB;
  executeModbusCommand(9); // Old clients still sending the retired command.
  assert(commandResult == CMD_RES_UNKNOWN_CMD);
  assert(currentDirection == direction && motorRunStartMs == started);
  assert(openRemainingMs == remaining && relayWrites == writes);
  handleMotorStopDisabled(); // A direct HTTP call must not bypass the UI.
  assert(server.code == 403);
  assert(currentDirection == direction && motorRunStartMs == started);
  assert(openRemainingMs == remaining && relayWrites == writes);
  assert(relayA == a && relayB == b);
}
int main() {
  assertManualStopRejected();
  executeModbusCommand(CMD_MOTOR_OPEN);
  assert(commandResult == CMD_RES_OK && currentDirection == MOTOR_OPEN);
  assert(relayA && !relayB);
  clockMs += 5000;
  assertManualStopRejected();
  clockMs = motorRunStartMs + OPEN_TRAVEL_MS - 1;
  updateMotorControl();
  assert(currentDirection == MOTOR_OPEN);
  ++clockMs;
  updateMotorControl();
  assert(currentDirection == MOTOR_STOP && openRemainingMs == 0);
  assert(!relayA && !relayB);

  executeModbusCommand(CMD_MOTOR_OPEN);
  assert(commandResult == CMD_RES_BUSY && currentDirection == MOTOR_STOP);

  executeModbusCommand(CMD_MOTOR_CLOSE);
  assert(commandResult == CMD_RES_OK && currentDirection == MOTOR_CLOSE);
  assert(!relayA && relayB);
  clockMs = motorRunStartMs + closeRunDurationMs - 1;
  assertManualStopRejected();
  updateMotorControl();
  assert(currentDirection == MOTOR_CLOSE);
  ++clockMs;
  updateMotorControl();
  assert(currentDirection == MOTOR_STOP);
  di6Active = true;
  processDI6Emergency();
  assert(currentDirection == MOTOR_OPEN && !nextMotorCommandIsOpen);
  const uint32_t emergencyOpenStart = motorRunStartMs;
  processDI6Emergency();
  assert(motorRunStartMs == emergencyOpenStart); // Held DI6 triggers once.
  clockMs = motorRunStartMs + 1000;
  di6Active = false;
  processDI6Emergency();
  clockMs += 100;
  di6Active = true;
  processDI6Emergency();
  assert(currentDirection == MOTOR_OPEN && openRemainingMs == OPEN_TRAVEL_MS);
  clockMs = motorRunStartMs + OPEN_TRAVEL_MS;
  updateMotorControl();
  assert(currentDirection == MOTOR_STOP && !nextMotorCommandIsOpen);
  executeModbusCommand(CMD_MOTOR_CLOSE);
  assert(commandResult == CMD_RES_BUSY && currentDirection == MOTOR_STOP);
  executeModbusCommand(CMD_MOTOR_OPEN);
  assert(commandResult == CMD_RES_OK && currentDirection == MOTOR_OPEN);
  clockMs = motorRunStartMs + OPEN_TRAVEL_MS;
  updateMotorControl();
  di6Active = false;
  processDI6Emergency();
  executeModbusCommand(CMD_MOTOR_CLOSE);
  assert(commandResult == CMD_RES_OK && currentDirection == MOTOR_CLOSE);
  updateMotorControl();
  assert(currentDirection == MOTOR_CLOSE);
  clockMs = motorRunStartMs + closeRunDurationMs;
  updateMotorControl();
  assert(currentDirection == MOTOR_STOP && openRemainingMs == OPEN_TRAVEL_MS);
  assert(!relayA && !relayB);
  puts("Motor STOP rejection, alternation and DI6 emergency tests passed");
}
'@

$build = Join-Path $root ".pio/host-tests"
New-Item -ItemType Directory -Force -Path $build | Out-Null
$cpp = Join-Path $build "motor_stop_disabled_test.cpp"
$exe = Join-Path $build "motor_stop_disabled_test.exe"
[IO.File]::WriteAllText($cpp, $fixture, (New-Object Text.UTF8Encoding($false)))
& $Compiler "-std=c++11" "-Wall" "-Wextra" "-pedantic" $cpp "-o" $exe
if ($LASTEXITCODE -ne 0) { throw "Host compilation failed" }
& $exe
if ($LASTEXITCODE -ne 0) { throw "Motor STOP rejection test failed" }
