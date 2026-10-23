"""Simple Modbus TCP lid cycle tester for the grill controller.

The tester intentionally uses only the OPEN (11) and CLOSE (12) commands.
It never sends a motor STOP command.  The stop button only stops scheduling
the next command and leaves the controller in charge of the current motion.
"""

import inspect
import queue
import threading
import time
import tkinter as tk
from tkinter import messagebox, ttk

from pymodbus.client import ModbusTcpClient


HOLDING_COMMAND = 0                 # 40001 on the wire
HOLDING_PROTOCOL_VERSION = 17       # 40018 on the wire
INPUT_SNAPSHOT = 80                  # 30081 on the wire
INPUT_SNAPSHOT_COUNT = 30

CMD_OPEN = 11
CMD_CLOSE = 12
PROTOCOL_VERSION = 4

DIR_STOP = 0
DIR_OPEN = 1
DIR_CLOSE = 2

STATUS_HEATING = 1 << 0
STATUS_DI6 = 1 << 5
STATUS_ENERGY_SAVING = 1 << 6


def call_with_unit(fn, unit_id, *args, **kwargs):
    """Call pymodbus with the keyword supported by the installed version."""
    parameters = inspect.signature(fn).parameters
    if "slave" in parameters:
        kwargs["slave"] = unit_id
    elif "unit" in parameters:
        kwargs["unit"] = unit_id
    elif "device_id" in parameters:
        kwargs["device_id"] = unit_id
    return fn(*args, **kwargs)


class CycleTester:
    def __init__(self, root):
        self.root = root
        self.root.title("Тест циклов крышки Modbus TCP")
        self.root.geometry("720x560")
        self.root.minsize(650, 480)

        self.client = None
        self.unit_id = 1
        self.stop_event = threading.Event()
        self.worker = None
        self.messages = queue.Queue()

        self.ip_var = tk.StringVar(value="192.168.1.51")
        self.port_var = tk.StringVar(value="502")
        self.unit_var = tk.StringVar(value="1")
        self.cycles_var = tk.StringVar(value="10")
        self.open_hold_var = tk.StringVar(value="10")
        self.close_hold_var = tk.StringVar(value="10")
        self.connection_var = tk.StringVar(value="Отключено")
        self.status_var = tk.StringVar(value="Готов")

        self._build_ui()
        self.root.after(100, self._drain_messages)
        self.root.protocol("WM_DELETE_WINDOW", self.close)

    def _build_ui(self):
        connection = ttk.LabelFrame(self.root, text="Подключение")
        connection.pack(fill="x", padx=10, pady=(10, 5))

        self._field(connection, "IP:", self.ip_var, 0, 0, 16)
        self._field(connection, "Порт:", self.port_var, 0, 2, 7)
        self._field(connection, "Slave ID:", self.unit_var, 0, 4, 6)

        self.connect_button = ttk.Button(connection, text="Подключить", command=self.connect)
        self.connect_button.grid(row=0, column=6, padx=(12, 4), pady=7)
        self.disconnect_button = ttk.Button(
            connection, text="Отключить", command=self.disconnect, state="disabled"
        )
        self.disconnect_button.grid(row=0, column=7, padx=4, pady=7)
        ttk.Label(connection, textvariable=self.connection_var).grid(
            row=1, column=0, columnspan=8, sticky="w", padx=7, pady=(0, 7)
        )

        settings = ttk.LabelFrame(self.root, text="Параметры теста")
        settings.pack(fill="x", padx=10, pady=5)
        self._field(settings, "Количество циклов:", self.cycles_var, 0, 0, 8)
        self._field(settings, "Пауза в открытом состоянии, с:", self.open_hold_var, 1, 0, 8)
        self._field(settings, "Пауза в закрытом состоянии, с:", self.close_hold_var, 2, 0, 8)

        actions = ttk.Frame(self.root)
        actions.pack(fill="x", padx=10, pady=5)
        self.start_button = ttk.Button(actions, text="Запустить тест", command=self.start)
        self.start_button.pack(side="left", padx=(0, 6))
        self.stop_button = ttk.Button(actions, text="Остановить тест", command=self.request_stop, state="disabled")
        self.stop_button.pack(side="left")
        ttk.Label(actions, textvariable=self.status_var).pack(side="left", padx=16)

        log_frame = ttk.LabelFrame(self.root, text="Журнал")
        log_frame.pack(fill="both", expand=True, padx=10, pady=(5, 10))
        self.log = tk.Text(log_frame, height=15, state="disabled", wrap="word")
        scrollbar = ttk.Scrollbar(log_frame, orient="vertical", command=self.log.yview)
        self.log.configure(yscrollcommand=scrollbar.set)
        self.log.pack(side="left", fill="both", expand=True)
        scrollbar.pack(side="right", fill="y")

    @staticmethod
    def _field(parent, label, variable, row, column, width):
        ttk.Label(parent, text=label).grid(row=row, column=column, sticky="w", padx=(7, 4), pady=5)
        ttk.Entry(parent, textvariable=variable, width=width).grid(
            row=row, column=column + 1, sticky="w", padx=(0, 10), pady=5
        )

    def _write_log(self, text):
        self.log.configure(state="normal")
        self.log.insert("end", time.strftime("%H:%M:%S ") + text + "\n")
        self.log.see("end")
        self.log.configure(state="disabled")

    def _drain_messages(self):
        while True:
            try:
                message = self.messages.get_nowait()
            except queue.Empty:
                break
            self._write_log(message)
        self.root.after(100, self._drain_messages)

    def _post(self, message):
        self.messages.put(message)

    def _read_snapshot(self):
        if self.client is None:
            raise RuntimeError("Нет подключения")
        response = call_with_unit(
            self.client.read_input_registers,
            self.unit_id,
            INPUT_SNAPSHOT,
            count=INPUT_SNAPSHOT_COUNT,
        )
        if response.isError():
            raise RuntimeError(str(response))
        values = response.registers
        if len(values) < INPUT_SNAPSHOT_COUNT:
            raise RuntimeError("Контроллер вернул неполную телеметрию")
        status = values[0]
        return {
            "status": status,
            "direction": values[13],
            "next_direction": values[24],
            "scenario": values[27],
            "command_result": values[8],
            "fault": values[10],
            "temperature": values[1] / 10.0,
            "heating": bool(status & STATUS_HEATING),
            "di6": bool(status & STATUS_DI6),
            "energy_saving": bool(status & STATUS_ENERGY_SAVING),
        }

    def _send_motor_command(self, command):
        response = call_with_unit(
            self.client.write_register,
            self.unit_id,
            HOLDING_COMMAND,
            command,
        )
        if response.isError():
            try:
                snapshot = self._read_snapshot()
                if snapshot["command_result"] == 4:
                    raise RuntimeError("Команда заблокирована: температура еще не достигла уставки")
            except RuntimeError as error:
                raise error
            raise RuntimeError(str(response))
        snapshot = self._read_snapshot()
        if snapshot["command_result"] == 4:
            raise RuntimeError("Команда заблокирована: температура еще не достигла уставки")
        if snapshot["command_result"] == 2:
            raise RuntimeError("Команда отклонена: крышка еще движется")

    def _wait_until_stopped(self, expected_direction):
        deadline = time.monotonic() + 25.0
        started = False
        while time.monotonic() < deadline:
            if self.stop_event.is_set():
                return False
            snapshot = self._read_snapshot()
            if snapshot["fault"]:
                raise RuntimeError("Контроллер сообщил аварийный код %s" % snapshot["fault"])
            if snapshot["di6"]:
                raise RuntimeError("Сработал аварийный концевик DI6")
            direction = snapshot["direction"]
            if direction == expected_direction:
                started = True
            if started and direction == DIR_STOP:
                return True
            time.sleep(0.2)
        raise RuntimeError("Тайм-аут ожидания завершения движения")

    def _hold(self, seconds, label):
        end = time.monotonic() + seconds
        while True:
            remaining = end - time.monotonic()
            if remaining <= 0:
                return True
            if self.stop_event.wait(min(0.25, remaining)):
                return False
        
    def _run(self, cycles, open_hold, close_hold):
        try:
            snapshot = self._read_snapshot()
            next_direction = snapshot["next_direction"]
            if next_direction not in (DIR_OPEN, DIR_CLOSE):
                raise RuntimeError("Контроллер не сообщил допустимое следующее направление")
            self._post("Начало теста: %d циклов, стартовое направление: %s" % (
                cycles, "открытие" if next_direction == DIR_OPEN else "закрытие"
            ))

            for cycle in range(1, cycles + 1):
                for command, hold, name in self._cycle_steps(next_direction, open_hold, close_hold):
                    if self.stop_event.is_set():
                        self._post("Тест остановлен до следующей команды")
                        return
                    self._post("Цикл %d/%d: команда %s" % (cycle, cycles, name))
                    self._send_motor_command(command)
                    if not self._wait_until_stopped(DIR_OPEN if command == CMD_OPEN else DIR_CLOSE):
                        self._post("Тест остановлен после завершения текущей команды")
                        return
                    self._post("Движение завершено, выдержка %.1f с" % hold)
                    if not self._hold(hold, name):
                        self._post("Тест остановлен во время выдержки")
                        return
                    next_direction = DIR_CLOSE if command == CMD_OPEN else DIR_OPEN
            self._post("Тест завершен успешно")
        except Exception as error:
            self._post("ОШИБКА: %s" % error)
        finally:
            self.root.after(0, self._worker_finished)

    @staticmethod
    def _cycle_steps(start_direction, open_hold, close_hold):
        if start_direction == DIR_OPEN:
            return ((CMD_OPEN, open_hold, "ОТКРЫТЬ"), (CMD_CLOSE, close_hold, "ЗАКРЫТЬ"))
        return ((CMD_CLOSE, close_hold, "ЗАКРЫТЬ"), (CMD_OPEN, open_hold, "ОТКРЫТЬ"))

    def _worker_finished(self):
        self.worker = None
        self.start_button.configure(state="normal")
        self.stop_button.configure(state="disabled")
        self.status_var.set("Готов")

    def connect(self):
        try:
            host = self.ip_var.get().strip()
            port = int(self.port_var.get())
            self.unit_id = int(self.unit_var.get())
            if not host or not 1 <= port <= 65535 or not 1 <= self.unit_id <= 247:
                raise ValueError("Проверьте IP, порт и Slave ID")
            client = ModbusTcpClient(host, port=port, timeout=2)
            if not client.connect():
                raise RuntimeError("Не удалось подключиться")
            version = call_with_unit(
                client.read_holding_registers,
                self.unit_id,
                HOLDING_PROTOCOL_VERSION,
                count=1,
            )
            if version.isError() or version.registers[0] != PROTOCOL_VERSION:
                client.close()
                raise RuntimeError("Нужна прошивка Modbus TCP v4")
            self.client = client
            self.connection_var.set("Подключено: %s:%d, Slave ID %d" % (host, port, self.unit_id))
            self.connect_button.configure(state="disabled")
            self.disconnect_button.configure(state="normal")
            self._write_log("Подключение установлено, протокол v4")
        except Exception as error:
            self.client = None
            messagebox.showerror("Ошибка подключения", str(error))

    def disconnect(self):
        if self.worker is not None:
            messagebox.showwarning("Тест выполняется", "Сначала остановите тест и дождитесь его завершения")
            return
        if self.client is not None:
            self.client.close()
        self.client = None
        self.connection_var.set("Отключено")
        self.connect_button.configure(state="normal")
        self.disconnect_button.configure(state="disabled")

    def start(self):
        if self.client is None:
            messagebox.showwarning("Нет подключения", "Сначала подключитесь к контроллеру")
            return
        if self.worker is not None:
            return
        try:
            cycles = int(self.cycles_var.get())
            open_hold = float(self.open_hold_var.get().replace(",", "."))
            close_hold = float(self.close_hold_var.get().replace(",", "."))
            if cycles < 1 or open_hold < 0 or close_hold < 0:
                raise ValueError
        except ValueError:
            messagebox.showerror("Ошибка параметров", "Циклы должны быть >= 1, время выдержки не может быть отрицательным")
            return
        self.stop_event.clear()
        self.worker = threading.Thread(
            target=self._run,
            args=(cycles, open_hold, close_hold),
            daemon=True,
        )
        self.start_button.configure(state="disabled")
        self.stop_button.configure(state="normal")
        self.status_var.set("Выполняется")
        self.worker.start()

    def request_stop(self):
        if self.worker is not None:
            self.stop_event.set()
            self.status_var.set("Остановка после текущего шага")
            self._write_log("Запрошена остановка: команда STOP контроллеру не отправляется")

    def close(self):
        if self.worker is not None:
            self.stop_event.set()
            messagebox.showinfo("Тест выполняется", "Дождитесь завершения текущей команды движения")
            return
        if self.client is not None:
            self.client.close()
        self.root.destroy()


def main():
    root = tk.Tk()
    CycleTester(root)
    root.mainloop()


if __name__ == "__main__":
    main()
