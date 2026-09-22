import inspect
from collections import deque
from dataclasses import dataclass
import math
import time
from typing import Optional
import tkinter as tk
from tkinter import ttk, messagebox

from pymodbus.client import ModbusTcpClient

REG_STATUS_BITS = 0
REG_TEMPERATURE_X10 = 1
REG_TARGET_X10 = 2
REG_HYSTERESIS_X10 = 3
REG_MIN_ON_SECONDS = 4
REG_MIN_OFF_SECONDS = 5
REG_EMERGENCY_X10 = 6
REG_COMMAND = 7
REG_COMMAND_RESULT = 8
REG_RELAY_LOCK_SECONDS = 9
REG_FAULT_CODE = 10
REG_HEATING_ENABLE = 11
REG_SENSOR_MV = 12
REG_MOTOR_DIRECTION = 13
REG_MOTOR_OPEN_REMAINING_SEC = 14
REG_ANTICIPATION_ENABLE = 15
REG_RATE_C_MIN_X100 = 16
REG_ON_THRESHOLD_X10 = 17
REG_OFF_THRESHOLD_X10 = 18
REG_OFF_LOOKAHEAD_X10 = 19
REG_ON_LOOKAHEAD_X10 = 20
REG_LEARNED_CYCLES = 21
REGISTER_COUNT = 22

CMD_HEAT_ON = 1
CMD_HEAT_OFF = 2
CMD_CLEAR_FAULT = 5
CMD_SAVE_THERMOSTAT = 6
CMD_MOTOR_OPEN = 7
CMD_MOTOR_CLOSE = 8
CMD_MOTOR_STOP = 9
NOISE_FILTER_ALPHA = 0.12
HISTORY_SECONDS = 3600
MAX_SAMPLE_GAP_SECONDS = 3.0
GRAPH_SERIES = (
    ("temperature", "Температура", "#0f8a5f", ()),
    ("filtered", "Сглаженная (только график)", "#d97706", ()),
    ("target", "Уставка", "#dc2626", ()),
    ("on_threshold", "Порог включения", "#2563eb", (6, 3)),
    ("off_threshold", "Порог выключения", "#64748b", (2, 3)),
)

CMD_RESULT_TEXT = {
    0: "Выполнено",
    1: "Неизвестная команда",
    2: "Занят",
    3: "Некорректный параметр",
}

FAULT_TEXT = {
    0: "Нет аварии",
    1: "Датчик вне диапазона",
    2: "Аварийная температура",
}

MOTOR_DIRECTION_TEXT = {
    0: "Стоп",
    1: "Открытие",
    2: "Закрытие",
}


@dataclass
class GraphSample:
    timestamp: float
    temperature: Optional[float]
    filtered: Optional[float]
    target: Optional[float]
    on_threshold: Optional[float]
    off_threshold: Optional[float]
    contactor: Optional[bool]


class App:
    def __init__(self, root: tk.Tk):
        self.root = root
        self.root.title("Modbus TCP тест гриля")
        width = min(1120, max(1000, self.root.winfo_screenwidth() - 80))
        height = min(860, max(660, self.root.winfo_screenheight() - 100))
        self.root.geometry(f"{width}x{height}")
        self.root.minsize(1000, 660)
        self.client = None
        self.connected = False
        self._poll_job = None
        self._syncing_fields = False
        self._dirty_fields = set()
        self._last_anticipation = False
        self._history_device = None
        self._latest_target = None
        self._active_unit = None

        self.host_var = tk.StringVar(value="192.168.1.51")
        self.port_var = tk.StringVar(value="502")
        self.slave_var = tk.StringVar(value="1")

        self.target_var = tk.StringVar(value="100.0")
        self.hysteresis_var = tk.StringVar(value="2.0")
        self.min_on_var = tk.StringVar(value="10")
        self.min_off_var = tk.StringVar(value="10")
        self.emergency_var = tk.StringVar(value="300.0")
        self.anticipation_var = tk.BooleanVar(value=False)

        self.status_var = tk.StringVar(value="Не подключено")
        self.values_var = tk.StringVar(value="-")
        self.contactor_var = tk.StringVar(value="Контактор: нет данных")
        self.anticipation_status_var = tk.StringVar(value="Упреждение: нет данных")
        self.learning_var = tk.StringVar(value="Автоподстройка: нет данных")

        self.target_entry = None
        self.hysteresis_entry = None
        self.min_on_entry = None
        self.min_off_entry = None
        self.emergency_entry = None
        self.heat_on_button = None
        self.heat_off_button = None
        self.motor_open_button = None
        self.motor_close_button = None
        self.motor_stop_button = None

        self.history = deque(maxlen=7200)
        self._filtered_temperature = None
        self.graph_canvas = None

        self._build_ui()
        self._settings_fields = {
            "target": (self.target_var, self.target_entry),
            "hysteresis": (self.hysteresis_var, self.hysteresis_entry),
            "min_on": (self.min_on_var, self.min_on_entry),
            "min_off": (self.min_off_var, self.min_off_entry),
            "emergency": (self.emergency_var, self.emergency_entry),
        }
        for name, (variable, _) in self._settings_fields.items():
            variable.trace_add("write", lambda *_args, key=name: self._mark_edited(key))
        self._disable_live_controls()
        self.root.protocol("WM_DELETE_WINDOW", self.close)
        self._tick()

    def _build_ui(self):
        frm = ttk.Frame(self.root, padding=10)
        frm.grid(sticky="nsew")
        self.root.columnconfigure(0, weight=1)
        self.root.rowconfigure(0, weight=1)
        frm.columnconfigure(0, weight=1)
        frm.rowconfigure(3, weight=1)

        conn = ttk.LabelFrame(frm, text="Подключение", padding=8)
        conn.grid(row=0, column=0, sticky="ew")

        ttk.Label(conn, text="IP:").grid(row=0, column=0, sticky="w")
        ttk.Entry(conn, textvariable=self.host_var, width=16).grid(row=0, column=1, sticky="w")
        ttk.Label(conn, text="Порт:").grid(row=0, column=2, sticky="w", padx=(8, 0))
        ttk.Entry(conn, textvariable=self.port_var, width=10).grid(row=0, column=3, sticky="w")
        ttk.Label(conn, text="Unit ID:").grid(row=0, column=4, sticky="w", padx=(8, 0))
        ttk.Entry(conn, textvariable=self.slave_var, width=8).grid(row=0, column=5, sticky="w")

        ttk.Button(conn, text="Подключить", command=self.connect).grid(row=0, column=6, padx=(8, 0))
        ttk.Button(conn, text="Отключить", command=self.disconnect).grid(row=0, column=7, padx=(4, 0))

        ctrl = ttk.LabelFrame(frm, text="Управление", padding=8)
        ctrl.grid(row=1, column=0, sticky="ew", pady=(8, 0))

        ttk.Label(ctrl, text="Уставка °C:").grid(row=0, column=0, sticky="w")
        self.target_entry = ttk.Entry(ctrl, textvariable=self.target_var, width=10)
        self.target_entry.grid(row=0, column=1, sticky="w")
        ttk.Button(ctrl, text="Записать уставку", command=self.write_target).grid(row=0, column=2, padx=6)

        ttk.Label(ctrl, text="Гистерезис °C:").grid(row=1, column=0, sticky="w")
        self.hysteresis_entry = ttk.Entry(ctrl, textvariable=self.hysteresis_var, width=10)
        self.hysteresis_entry.grid(row=1, column=1, sticky="w")
        ttk.Label(ctrl, text="Мин. ON, с:").grid(row=1, column=2, sticky="w")
        self.min_on_entry = ttk.Entry(ctrl, textvariable=self.min_on_var, width=10)
        self.min_on_entry.grid(row=1, column=3, sticky="w")
        ttk.Label(ctrl, text="Мин. OFF, с:").grid(row=1, column=4, sticky="w")
        self.min_off_entry = ttk.Entry(ctrl, textvariable=self.min_off_var, width=10)
        self.min_off_entry.grid(row=1, column=5, sticky="w")
        ttk.Label(ctrl, text="Авария °C:").grid(row=2, column=0, sticky="w")
        self.emergency_entry = ttk.Entry(ctrl, textvariable=self.emergency_var, width=10)
        self.emergency_entry.grid(row=2, column=1, sticky="w")
        ttk.Button(ctrl, text="Записать термостат", command=self.write_thermostat).grid(row=2, column=2, columnspan=2, padx=6)
        self.anticipation_check = ttk.Checkbutton(
            ctrl, text="Упреждение и автоподстройка",
            variable=self.anticipation_var, command=self.write_anticipation,
        )
        self.anticipation_check.grid(row=2, column=4, columnspan=2, sticky="w")

        self.heat_on_button = ttk.Button(ctrl, text="Нагрев ВКЛ", command=lambda: self.set_heating(True))
        self.heat_on_button.grid(row=3, column=0, pady=4)
        self.heat_off_button = ttk.Button(ctrl, text="Нагрев ВЫКЛ", command=lambda: self.set_heating(False))
        self.heat_off_button.grid(row=3, column=1, pady=4)
        ttk.Button(ctrl, text="Сброс аварии", command=lambda: self.send_command(CMD_CLEAR_FAULT)).grid(row=3, column=2, pady=4)
        self.motor_open_button = ttk.Button(ctrl, text="Открыть", command=lambda: self.send_command(CMD_MOTOR_OPEN))
        self.motor_open_button.grid(row=4, column=0, pady=4)
        self.motor_close_button = ttk.Button(ctrl, text="Закрыть", command=lambda: self.send_command(CMD_MOTOR_CLOSE))
        self.motor_close_button.grid(row=4, column=1, pady=4)
        self.motor_stop_button = ttk.Button(ctrl, text="Стоп мотор", command=lambda: self.send_command(CMD_MOTOR_STOP))
        self.motor_stop_button.grid(row=4, column=2, pady=4)

        stat = ttk.LabelFrame(frm, text="Статус", padding=8)
        stat.grid(row=2, column=0, sticky="ew", pady=(8, 0))
        stat.columnconfigure(0, weight=1)
        ttk.Label(stat, textvariable=self.status_var).grid(row=0, column=0, sticky="w")
        self.contactor_label = ttk.Label(stat, textvariable=self.contactor_var, font=("Segoe UI", 10, "bold"))
        self.contactor_label.grid(row=1, column=0, sticky="w")
        values_label = ttk.Label(stat, textvariable=self.values_var, justify="left", anchor="w")
        values_label.grid(row=2, column=0, sticky="ew")
        ttk.Label(stat, textvariable=self.anticipation_status_var).grid(row=3, column=0, sticky="w")
        ttk.Label(stat, textvariable=self.learning_var).grid(row=4, column=0, sticky="w")
        stat.bind("<Configure>", lambda event: values_label.configure(wraplength=max(200, event.width - 24)))

        graph = ttk.LabelFrame(frm, text="Температура и контактор: последние 60 минут", padding=8)
        graph.grid(row=3, column=0, sticky="nsew", pady=(8, 0))
        graph.columnconfigure(0, weight=1)
        graph.rowconfigure(1, weight=1)
        legend = ttk.Frame(graph)
        legend.grid(row=0, column=0, sticky="ew")
        for i, (_key, label, color, dash) in enumerate(GRAPH_SERIES):
            ttk.Label(legend, text=label + (" (пунктир)" if dash else ""), foreground=color).grid(
                row=i // 3, column=i % 3, sticky="w", padx=(0, 18),
            )
        ttk.Label(legend, text="Полоса контактора: ВКЛ / ВЫКЛ", foreground="#a16207").grid(
            row=1, column=2, sticky="w",
        )

        self.graph_canvas = tk.Canvas(
            graph,
            width=960,
            height=260,
            bg="#ffffff",
            highlightthickness=1,
            highlightbackground="#cccccc",
        )
        self.graph_canvas.grid(row=1, column=0, sticky="nsew")
        self.graph_canvas.bind("<Configure>", lambda _event: self._draw_graph())

    def connect(self):
        self.disconnect()
        try:
            host = self.host_var.get().strip()
            port = int(self.port_var.get())
            unit = self._slave()
            if not host or not 1 <= port <= 65535 or not 0 <= unit <= 255:
                raise ValueError("Проверьте IP, порт (1...65535) и Unit ID (0...255)")
            self.client = ModbusTcpClient(
                host=host,
                port=port,
                timeout=1,
            )
            self.connected = bool(self.client.connect())
            self.status_var.set("Подключено" if self.connected else "Ошибка подключения")
            if self.connected:
                self._active_unit = unit
                device = (host, port, unit)
                if device != self._history_device:
                    self.history.clear()
                    self._filtered_temperature = None
                    self._dirty_fields.clear()
                    self._history_device = device
                self.poll()
        except Exception as exc:
            self._mark_unavailable(str(exc))

    def disconnect(self):
        if self.client:
            try:
                self.client.close()
            except Exception:
                pass
        self.client = None
        self.connected = False
        self._active_unit = None
        self._mark_unavailable("Не подключено", record=bool(self.history))

    def close(self):
        if self._poll_job is not None:
            self.root.after_cancel(self._poll_job)
            self._poll_job = None
        self.disconnect()
        self.root.destroy()

    def _disable_live_controls(self):
        for widget in (
            self.heat_on_button, self.motor_open_button,
            self.motor_close_button, self.anticipation_check,
        ):
            widget.state(["disabled"])
        # Keep manual OFF/STOP available during a failed status read.
        for widget in (self.heat_off_button, self.motor_stop_button):
            widget.state(["!disabled"] if self.connected else ["disabled"])

    def _mark_unavailable(self, reason, record=True):
        self.status_var.set(reason)
        self.contactor_var.set("Контактор: нет актуальных данных")
        self.contactor_label.configure(foreground="#64748b")
        self.anticipation_status_var.set("Упреждение: нет актуальных данных")
        self.learning_var.set("Автоподстройка: нет актуальных данных")
        self.values_var.set("-")
        self._latest_target = None
        self._disable_live_controls()
        if record:
            self._append_history(None)
        self._draw_graph()

    def _mark_edited(self, name):
        if not self._syncing_fields:
            self._dirty_fields.add(name)

    def _sync_settings(self, values):
        self._syncing_fields = True
        try:
            for name, value in values.items():
                variable, entry = self._settings_fields[name]
                if name not in self._dirty_fields and not self._entry_focused(entry):
                    variable.set(value)
        finally:
            self._syncing_fields = False

    def _slave(self):
        return self._active_unit if self._active_unit is not None else int(self.slave_var.get())

    def _call_with_slave(self, fn, *args, **kwargs):
        sid = self._slave()
        params = inspect.signature(fn).parameters
        if "slave" in params:
            kwargs["slave"] = sid
        elif "unit" in params:
            kwargs["unit"] = sid
        elif "device_id" in params:
            kwargs["device_id"] = sid
        return fn(*args, **kwargs)

    def write_reg(self, addr: int, value: int):
        if not self.connected or not self.client:
            raise RuntimeError("Нет подключения")
        rr = self._call_with_slave(self.client.write_register, addr, int(value) & 0xFFFF)
        if rr.isError():
            raise RuntimeError(str(rr))

    def send_command(self, cmd: int):
        try:
            self.write_reg(REG_COMMAND, cmd)
            self.poll(record=False)
        except Exception as exc:
            messagebox.showerror("Modbus", str(exc))

    def set_heating(self, enabled: bool):
        try:
            # Heating has its own RW register. Writing it directly avoids ambiguity
            # with the one-shot command register and mirrors the web UI state.
            self.write_reg(REG_HEATING_ENABLE, 1 if enabled else 0)
            self.poll(record=False)
        except Exception as exc:
            messagebox.showerror("Modbus", str(exc))

    def write_target(self):
        try:
            val = float(self.target_var.get().replace(",", "."))
            if not math.isfinite(val) or not 10 <= val <= 300:
                raise ValueError("Уставка должна быть в пределах 10...300 °C")
            self.write_reg(REG_TARGET_X10, int(round(val * 10.0)) & 0xFFFF)
            self._dirty_fields.discard("target")
            self.poll(record=False)
        except Exception as exc:
            messagebox.showerror("Modbus", str(exc))

    def write_thermostat(self):
        try:
            hysteresis = float(self.hysteresis_var.get().replace(",", "."))
            min_on = int(self.min_on_var.get())
            min_off = int(self.min_off_var.get())
            emergency = float(self.emergency_var.get().replace(",", "."))
            if not math.isfinite(hysteresis) or not 0.5 <= hysteresis <= 50:
                raise ValueError("Гистерезис должен быть в пределах 0,5...50 °C")
            if not 0 <= min_on <= 3600 or not 0 <= min_off <= 3600:
                raise ValueError("Минимальное время ON/OFF должно быть в пределах 0...3600 с")
            if not math.isfinite(emergency) or not 10 <= emergency <= 300:
                raise ValueError("Аварийная температура должна быть в пределах 10...300 °C")
            if self._latest_target is None:
                raise ValueError("Сначала получите актуальное состояние контроллера")
            if emergency < self._latest_target:
                raise ValueError("Аварийная температура не может быть ниже уставки контроллера")
            self.write_reg(REG_HYSTERESIS_X10, int(round(hysteresis * 10.0)))
            self.write_reg(REG_MIN_ON_SECONDS, min_on)
            self.write_reg(REG_MIN_OFF_SECONDS, min_off)
            self.write_reg(REG_EMERGENCY_X10, int(round(emergency * 10.0)))
            self.write_reg(REG_COMMAND, CMD_SAVE_THERMOSTAT)
            self._dirty_fields.difference_update(("hysteresis", "min_on", "min_off", "emergency"))
            self.poll(record=False)
        except Exception as exc:
            messagebox.showerror("Modbus", str(exc))

    def write_anticipation(self):
        requested = self.anticipation_var.get()
        self.anticipation_check.state(["disabled"])
        try:
            self.write_reg(REG_ANTICIPATION_ENABLE, int(requested))
            self.poll(record=False)
            if self._last_anticipation != requested:
                raise RuntimeError("Контроллер не подтвердил изменение упреждения")
        except Exception as exc:
            self.anticipation_var.set(self._last_anticipation)
            self._mark_unavailable(f"Ошибка изменения упреждения: {exc}", record=False)
            messagebox.showerror("Modbus", str(exc))

    @staticmethod
    def _to_i16(v: int) -> int:
        return v - 65536 if v > 32767 else v

    @staticmethod
    def _decode_status_bits(status: int) -> str:
        flags = [
            ("HEAT_EN", bool(status & (1 << 0))),
            ("RELAY_ON", bool(status & (1 << 1))),
            ("FAULT", bool(status & (1 << 3))),
            ("SENSOR_FAULT", bool(status & (1 << 4))),
            ("DI6_CLOSED", bool(status & (1 << 5))),
        ]
        return ", ".join(f"{name}={'1' if val else '0'}" for name, val in flags)

    def _entry_focused(self, entry_widget) -> bool:
        try:
            return self.root.focus_get() == entry_widget
        except Exception:
            return False

    def _prune_history(self, now):
        cutoff = now - HISTORY_SECONDS
        while self.history and self.history[0].timestamp < cutoff:
            self.history.popleft()

    def _append_history(self, temperature, target=None, on_threshold=None,
                        off_threshold=None, contactor=None, now=None):
        now = time.monotonic() if now is None else now
        if temperature is None or not math.isfinite(temperature):
            temperature = None
            self._filtered_temperature = None
        elif (self._filtered_temperature is None or not self.history or
              now - self.history[-1].timestamp > MAX_SAMPLE_GAP_SECONDS):
            self._filtered_temperature = temperature
        else:
            self._filtered_temperature += NOISE_FILTER_ALPHA * (temperature - self._filtered_temperature)
        self.history.append(GraphSample(
            now, temperature, self._filtered_temperature, target,
            on_threshold, off_threshold, contactor,
        ))
        self._prune_history(now)

    def _draw_graph(self, now=None):
        if self.graph_canvas is None:
            return
        now = time.monotonic() if now is None else now
        self._prune_history(now)
        c = self.graph_canvas
        c.delete("all")
        w, h = max(100, c.winfo_width()), max(120, c.winfo_height())
        left, right, top, bottom = 56, 24, 16, 66
        pw, ph = max(10, w - left - right), max(10, h - top - bottom)
        plot_bottom = top + ph
        samples = list(self.history)

        def x_at(timestamp):
            return left + pw * (timestamp - (now - HISTORY_SECONDS)) / HISTORY_SECONDS

        c.create_rectangle(left, top, left + pw, plot_bottom, outline="#999")
        for minute in range(0, 61, 10):
            x = left + pw * minute / 60
            c.create_line(x, top, x, plot_bottom, fill="#eeeeee")
            c.create_text(x, plot_bottom + 12, text="сейчас" if minute == 60 else f"-{60 - minute} мин",
                          fill="#666", font=("Segoe UI", 9))

        scale = [getattr(sample, key) for sample in samples for key, *_rest in GRAPH_SERIES
                 if getattr(sample, key) is not None and math.isfinite(getattr(sample, key))]
        if not scale:
            c.create_text(left + pw / 2, top + ph / 2, text="Нет данных за последний час",
                          fill="#666", font=("Segoe UI", 11))
        else:
            tmin, tmax = min(scale), max(scale)
            margin = max(0.5, (tmax - tmin) * 0.1)
            tmin, tmax = tmin - margin, tmax + margin

            def y_at(value):
                return top + ph * (tmax - value) / (tmax - tmin)

            for i in range(5):
                t = tmax - (tmax - tmin) * i / 4
                y = y_at(t)
                c.create_line(left, y, left + pw, y, fill="#f0f0f0")
                c.create_text(left - 6, y, text=f"{t:.1f}", anchor="e",
                              fill="#666", font=("Segoe UI", 9))

            # Draw separate segments: never bridge missing readings or a TCP outage.
            for key, _label, color, dash in GRAPH_SERIES:
                points = []
                previous = None

                def flush():
                    if len(points) >= 4:
                        c.create_line(*points, fill=color, width=2, dash=dash, tags=(key,))
                    elif points:
                        x, y = points
                        c.create_oval(x - 2, y - 2, x + 2, y + 2,
                                      fill=color, outline=color, tags=(key,))

                for sample in samples:
                    value = getattr(sample, key)
                    if (value is None or not math.isfinite(value) or
                            (previous is not None and
                             sample.timestamp - previous.timestamp > MAX_SAMPLE_GAP_SECONDS)):
                        flush()
                        points = []
                        previous = None
                    if value is None or not math.isfinite(value):
                        continue
                    x, y = x_at(sample.timestamp), y_at(value)
                    if previous is not None and key == "target":
                        points.extend((x, points[-1]))
                    points.extend((x, y))
                    previous = sample
                flush()

        band_y = plot_bottom + 30
        c.create_rectangle(left, band_y, left + pw, band_y + 14, outline="#cbd5e1")
        interval = None

        def paint_interval():
            if interval is not None:
                start, end, enabled = interval
                color = "#f59e0b" if enabled else "#cbd5e1"
                c.create_rectangle(x_at(start), band_y + 1, x_at(end), band_y + 13,
                                   fill=color, outline="", tags=("contactor",))

        # Merge adjacent states instead of creating thousands of rectangles each second.
        for index, sample in enumerate(samples):
            if sample.contactor is None:
                paint_interval()
                interval = None
                continue
            until = samples[index + 1].timestamp if index + 1 < len(samples) else now
            until = min(until, sample.timestamp + MAX_SAMPLE_GAP_SECONDS, now)
            if until <= sample.timestamp:
                continue
            if interval is not None and interval[2] == sample.contactor and sample.timestamp <= interval[1]:
                interval = (interval[0], until, interval[2])
            else:
                paint_interval()
                interval = (sample.timestamp, until, sample.contactor)
        paint_interval()
        c.create_text(left, band_y + 24, anchor="w",
                      text="Контактор: желтый = ВКЛ, серый = ВЫКЛ, пробел = нет данных",
                      fill="#666", font=("Segoe UI", 9))

    def poll(self, record=True):
        if not self.connected or not self.client:
            return
        rr = self._call_with_slave(self.client.read_holding_registers, 0, count=REGISTER_COUNT)
        if rr is None or rr.isError():
            raise RuntimeError(f"Ошибка чтения Modbus: {rr}")
        r = rr.registers
        if len(r) < REGISTER_COUNT:
            raise RuntimeError("Нужны регистры 0...21. Обновите прошивку контроллера")

        status = r[REG_STATUS_BITS]
        temp_raw = r[REG_TEMPERATURE_X10]
        sensor_fault = bool(status & (1 << 4))
        temperature = None if temp_raw == 0x8000 or sensor_fault else self._to_i16(temp_raw) / 10.0
        temp_text = "нет данных" if temperature is None else f"{temperature:.1f}"
        target = self._to_i16(r[REG_TARGET_X10]) / 10.0
        hysteresis = r[REG_HYSTERESIS_X10] / 10.0
        min_on, min_off = r[REG_MIN_ON_SECONDS], r[REG_MIN_OFF_SECONDS]
        emergency = r[REG_EMERGENCY_X10] / 10.0
        cmd_res, fault_code = r[REG_COMMAND_RESULT], r[REG_FAULT_CODE]
        relay_lock = r[REG_RELAY_LOCK_SECONDS]
        motor_direction = r[REG_MOTOR_DIRECTION]
        motor_open_remaining = r[REG_MOTOR_OPEN_REMAINING_SEC]
        heating_enabled = bool(status & (1 << 0))
        heater_relay_on = bool(status & (1 << 1))
        lid_closed = bool(status & (1 << 5))
        anticipation_enabled = bool(r[REG_ANTICIPATION_ENABLE])
        rate = self._to_i16(r[REG_RATE_C_MIN_X100]) / 100.0
        on_threshold = self._to_i16(r[REG_ON_THRESHOLD_X10]) / 10.0
        off_threshold = self._to_i16(r[REG_OFF_THRESHOLD_X10]) / 10.0
        off_seconds, on_seconds = r[REG_OFF_LOOKAHEAD_X10] / 10.0, r[REG_ON_LOOKAHEAD_X10] / 10.0
        learned = r[REG_LEARNED_CYCLES]
        fault = sensor_fault or bool(status & (1 << 3)) or fault_code != 0
        self._latest_target = target
        self._last_anticipation = anticipation_enabled
        self.anticipation_var.set(anticipation_enabled)
        self.anticipation_check.state(["!disabled"])

        self.status_var.set("Подключено, данные обновлены")
        self.contactor_var.set(
            f"Контактор: {'ВКЛ' if heater_relay_on else 'ВЫКЛ'}   |   "
            f"Нагрев: {'разрешен' if heating_enabled else 'выключен'}   |   "
            f"До разрешения переключения: {relay_lock} с"
        )
        self.contactor_label.configure(foreground="#a16207" if heater_relay_on else "#475569")
        lines = [
            f"Температура: {temp_text} °C   Уставка: {target:.1f} °C   Гистерезис: {hysteresis:.1f} °C   AD8495: {r[REG_SENSOR_MV]} мВ",
            f"Мин. ON/OFF: {min_on}/{min_off} с   Авария: {emergency:.1f} °C   "
            f"Мотор: {MOTOR_DIRECTION_TEXT.get(motor_direction, str(motor_direction))}, открытие осталось {motor_open_remaining} с",
            f"StatusBits=0x{status:04X} [{self._decode_status_bits(status)}]",
            f"CmdResult={cmd_res} ({CMD_RESULT_TEXT.get(cmd_res, 'Неизвестный результат')})   "
            f"Fault={fault_code} ({FAULT_TEXT.get(fault_code, 'Неизвестная авария')})",
        ]
        self.values_var.set("\n".join(lines))
        self.anticipation_status_var.set(
            f"Упреждение: {'ВКЛ' if anticipation_enabled else 'ВЫКЛ'}   "
            f"Расчетные пороги ВКЛ / ВЫКЛ: {on_threshold:.1f} / {off_threshold:.1f} °C   "
            f"Скорость: {rate:+.2f} °C/мин"
        )
        if not anticipation_enabled:
            learning = "выключена"
        elif fault:
            learning = "приостановлена: авария / датчик"
        elif not heating_enabled:
            learning = "ожидание включения нагрева"
        elif motor_direction != 0:
            learning = "приостановлена: движение крышки"
        else:
            learning = "наблюдение за температурой"
        self.learning_var.set(
            f"Автоподстройка: {learning}; этапов: {learned}   "
            f"Упреждение ВКЛ / ВЫКЛ: {on_seconds:.1f} / {off_seconds:.1f} с"
        )
        self._sync_settings({
            "target": f"{target:.1f}", "hysteresis": f"{hysteresis:.1f}",
            "min_on": str(min_on), "min_off": str(min_off), "emergency": f"{emergency:.1f}",
        })
        self.motor_open_button.state(["disabled"] if motor_direction == 1 or motor_open_remaining == 0 else ["!disabled"])
        self.motor_close_button.state(["disabled"] if motor_direction == 2 or lid_closed else ["!disabled"])
        self.motor_stop_button.state(["disabled"] if motor_direction == 0 else ["!disabled"])
        self.heat_on_button.state(["disabled"] if heating_enabled else ["!disabled"])
        self.heat_off_button.state(["disabled"] if not heating_enabled else ["!disabled"])
        if record:
            self._append_history(temperature, target, on_threshold, off_threshold, heater_relay_on)
        self._draw_graph()

    def _tick(self):
        try:
            self.poll()
        except Exception as exc:
            self._mark_unavailable(f"Ошибка: {exc}")
        if not self.connected:
            self._draw_graph()
        self._poll_job = self.root.after(1000, self._tick)


if __name__ == "__main__":
    root = tk.Tk()
    app = App(root)
    root.mainloop()
