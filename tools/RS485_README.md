# RS-485 / Modbus карта регистров

Параметры slave:
- UART: `Serial2`
- RXD: `GPIO14`
- TXD: `GPIO27`
- Скорость: `9600 8N1`
- Slave ID: `1`

Holding registers:
- `0` `RO` `STATUS_BITS` (битовая маска)
  - bit0: heating_enabled
  - bit1: heater_relay_on
  - bit2: autotune_active
  - bit3: fault_active
  - bit4: thermo_open
  - bit5: di6_closed
- `1` `RO` `TEMPERATURE_X10` (int16, `0x8000` = нет данных)
- `2` `RW` `TARGET_X10` (int16)
- `3` `RO` `POWER_X10` (uint16)
- `4` `RW` `KP_X100` (uint16)
- `5` `RW` `KI_X10000` (uint16)
- `6` `RW` `KD_X100` (uint16)
- `7` `RW` `COMMAND` (после выполнения сбрасывается в `0`)
- `8` `RO` `COMMAND_RESULT` (`0` ok, `1` unknown, `2` busy, `3` invalid arg)
- `9` `RO` `AUTOTUNE_CYCLES`
- `10` `RO` `FAULT_CODE` (`0` none, `1` thermocouple open, `2` emergency)
- `11` `RW` `HEATING_ENABLE` (`0/1`)
- `12` `RO` `AUTOTUNE_READY` (`0/1`)
- `13` `RO` `MOTOR_DIRECTION` (`0` stop, `1` open, `2` close)
- `14` `RO` `MOTOR_OPEN_REMAINING_SEC`

Команды (`reg 7`):
- `1` heat on
- `2` heat off
- `3` autotune start
- `4` autotune stop
- `5` clear fault
- `6` save pid
- `7` motor open
- `8` motor close
- `9` motor stop

## Tkinter тестер

Файл: `tools/rs485_tester.py`

Установка зависимостей:
```bash
pip install pymodbus pyserial
```

Запуск:
```bash
python tools/rs485_tester.py
```

