param([string]$Compiler = "g++")
$ErrorActionPreference = "Stop"
$root = Split-Path -Parent $PSScriptRoot
$source = Get-Content -Raw -Encoding UTF8 (Join-Path $root "src/main.cpp")
function Extract([string]$pattern) {
    $match = [regex]::Match($source, $pattern)
    if (-not $match.Success) { throw "Missing source: $pattern" }
    return $match.Value + [Environment]::NewLine
}
$fixture = @'
#include <stdint.h>
#include <cassert>
#include <vector>
#include <cstdio>
#include <cstring>
#include <algorithm>
'@
foreach ($name in @("ModbusReg", "ModbusCommand", "ModbusCommandResult", "MotorDirection")) {
    $fixture += [Environment]::NewLine + (Extract ("(?ms)^enum " + $name + " : \w+ \{.*?^\};"))
}
$fixture += @'
uint16_t modbusRegs[REG_REG_COUNT] = {};
uint16_t commandResult = 0, lastMotorEvent = 0;
uint64_t totalRuntimeMs = 7200000, heaterRuntimeMs = 3600000;
bool di6Active = false, sensorFault = false, faultActive = false;
bool startupOpeningInProgress = false, nextMotorCommandIsOpen = false;
MotorDirection currentDirection = MOTOR_STOP;
uint8_t relayOutputMask = 0xf7;
const uint8_t MODBUS_UNIT_ID = 1;
int applied = 0;
void accountRuntime() {}
void syncModbusRegistersFromState() {}
void updateDI6State() {}
void processDI6Emergency() {}
bool writeModbusRegister(uint16_t address, uint16_t value) {
  ++applied; modbusRegs[address] = value; return true;
}
void executeModbusCommand(uint16_t cmd) {
  ++applied;
  currentDirection = cmd == CMD_MOTOR_OPEN ? MOTOR_OPEN : MOTOR_CLOSE;
  commandResult = CMD_RES_OK;
}
uint8_t highByte(uint16_t v) { return v >> 8; }
uint8_t lowByte(uint16_t v) { return v & 255; }
uint16_t word(uint8_t hi, uint8_t lo) { return (uint16_t(hi) << 8) | lo; }
struct EthernetClient {
  std::vector<uint8_t> rx, tx;
  size_t cursor = 0;
  bool stopped = false;
  void write(const uint8_t* p, size_t n) { tx.insert(tx.end(), p, p+n); }
  int read(uint8_t* p, size_t n) {
    assert(cursor+n <= rx.size());
    memcpy(p, rx.data()+cursor, n); cursor += n; return n;
  }
  void stop() { stopped = true; }
};
bool waitForClientBytes(EthernetClient& c, int n) { return c.rx.size()-c.cursor >= size_t(n); }
'@
foreach ($name in @("holdingIndex", "scenarioState", "readWireRegister", "validateWireWrite", "applyWireWrite", "sendModbusTcpResponse", "sendModbusException", "processModbusTcpRequest")) {
    $fixture += [Environment]::NewLine + (Extract ("(?ms)^(?:int|uint16_t|uint8_t|bool|void) " + $name + "\([^;{]*\) \{.*?^\}"))
}
$fixture += @'
EthernetClient request(std::vector<uint8_t> pdu) {
  EthernetClient c;
  c.rx = {0, 1, 0, 0, 0, uint8_t(pdu.size()+1), 1};
  c.rx.insert(c.rx.end(), pdu.begin(), pdu.end());
  processModbusTcpRequest(c);
  assert(!c.tx.empty());
  return c;
}
void exception(const EthernetClient& c, int function, int error) {
  assert(c.tx.size() == 9 && c.tx[7] == (function|0x80) && c.tx[8] == error);
}
int main() {
  for (uint16_t a = 2; a <= 6; ++a) {
    exception(request({3, 0, uint8_t(a), 0, 1}), 3, 3);
    exception(request({6, 0, uint8_t(a), 0, 1}), 6, 3);
    exception(request({16, 0, uint8_t(a), 0, 1, 2, 0, 1}), 16, 3);
  }
  assert(applied == 0);
  // Forbidden/RO registers anywhere in a packet cause zero partial writes.
  exception(request({16, 0, 0, 0, 3, 6, 0, 12, 0, 0, 0, 0}), 16, 2);
  exception(request({16, 0, 14, 0, 2, 4, 0, 0, 0, 99}), 16, 3);
  assert(applied == 0);
  auto version = request({3, 0, 17, 0, 1});
  assert(version.tx[7] == 3 && version.tx[10] == 3);
  auto counters = request({4, 0, 0, 0, 4});
  assert(counters.tx[7] == 4 && counters.tx[8] == 8);
  assert(word(counters.tx[11], counters.tx[12]) == 2);
  assert(word(counters.tx[15], counters.tx[16]) == 1);
  totalRuntimeMs = 3599999;
  auto partial = request({4, 0, 0, 0, 2});
  assert(word(partial.tx[11], partial.tx[12]) == 0);
  totalRuntimeMs = 3600000;
  auto hour = request({4, 0, 0, 0, 2});
  assert(word(hour.tx[11], hour.tx[12]) == 1);
  modbusRegs[REG_TEMPERATURE_X10] = 1000;
  auto temp = request({4, 0, 70, 0, 1});
  assert(word(temp.tx[9], temp.tx[10]) == 1000);
  auto snapshot = request({4, 0, 80, 0, 28});
  assert(snapshot.tx[7] == 4 && snapshot.tx[8] == 56);
  exception(request({4, 0, 19, 0, 2}), 4, 2);
  exception(request({6, 0, 17, 0, 2}), 6, 2);
  exception(request({6, 0, 9, 0, 2}), 6, 3);
  exception(request({6, 0, 0, 0, 7}), 6, 3);
  auto setpoint = request({6, 0, 7, 3, 232});
  assert(setpoint.tx[7] == 6 && modbusRegs[REG_TARGET_X10] == 1000);
  request({6, 0, 8, 0, 0});
  assert(modbusRegs[REG_CLOSING_HEIGHT_MM] == 0);
  auto close = request({6, 0, 0, 0, 12});
  assert(close.tx[7] == 6 && currentDirection == MOTOR_CLOSE);
  exception(request({6, 0, 0, 0, 11}), 6, 6);
  auto heat = request({6, 0, 9, 0, 1});
  assert(heat.tx[7] == 6 && modbusRegs[REG_HEATING_ENABLE] == 1);
  currentDirection = MOTOR_STOP; di6Active = true;
  exception(request({6, 0, 0, 0, 12}), 6, 6);
  nextMotorCommandIsOpen = true;
  assert(request({6, 0, 0, 0, 11}).tx[7] == 6);
  puts("modbusTCP wire map, exceptions, FC04, packet validation and interlocks passed");
}
'@
$build = Join-Path $root ".pio/host-tests"
New-Item -ItemType Directory -Force -Path $build | Out-Null
$cpp = Join-Path $build "modbusTCP_protocol_test.cpp"
$exe = Join-Path $build "modbusTCP_protocol_test.exe"
[IO.File]::WriteAllText($cpp, $fixture, (New-Object Text.UTF8Encoding($false)))
& $Compiler "-std=c++11" "-Wall" "-Wextra" "-pedantic" $cpp "-o" $exe
if ($LASTEXITCODE -ne 0) { throw "Compilation failed" }
& $exe
if ($LASTEXITCODE -ne 0) { throw "Protocol test failed" }
