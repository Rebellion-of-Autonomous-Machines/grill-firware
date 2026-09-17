import inspect
from collections import deque
import tkinter as tk
from tkinter import ttk, messagebox

from pymodbus.client import ModbusSerialClient

REG_STATUS_BITS = 0
REG_TEMPERATURE_X10 = 1
REG_TARGET_X10 = 2
REG_POWER_X10 = 3
REG_KP_X100 = 4
REG_KI_X10000 = 5
REG_KD_X100 = 6
REG_COMMAND = 7
REG_COMMAND_RESULT = 8
REG_AUTOTUNE_CYCLES = 9
REG_FAULT_CODE = 10
REG_HEATING_ENABLE = 11
REG_AUTOTUNE_READY = 12
REG_MOTOR_DIRECTION = 13
REG_MOTOR_OPEN_REMAINING_SEC = 14

CMD_HEAT_ON = 1
CMD_HEAT_OFF = 2
CMD_AUTOTUNE_START = 3
CMD_AUTOTUNE_STOP = 4
CMD_CLEAR_FAULT = 5
CMD_SAVE_PID = 6
CMD_MOTOR_OPEN = 7
CMD_MOTOR_CLOSE = 8
CMD_MOTOR_STOP = 9
NOISE_FILTER_ALPHA = 0.12

CMD_RESULT_TEXT = {
    0: "OK",
    1: "UNKNOWN_CMD",
    2: "BUSY",
    3: "INVALID_ARG",
}

FAULT_TEXT = {
    0: "NONE",
    1: "THERMO_OPEN",
    2: "EMERGENCY_TEMP",
}

MOTOR_DIRECTION_TEXT = {
    0: "STOP",
    1: "OPEN",
    2: "CLOSE",
}


class App:
    def __init__(self, root: tk.Tk):
        self.root = root
        self.root.title("RS-485 Modbus тест")
        self.root.geometry("1000x680")
        self.root.minsize(920, 560)
        self.client = None
        self.connected = False

        self.port_var = tk.StringVar(value="COM4")
        self.baud_var = tk.StringVar(value="9600")
        self.slave_var = tk.StringVar(value="1")

        self.target_var = tk.StringVar(value="180.0")
        self.kp_var = tk.StringVar(value="12.0")
        self.ki_var = tk.StringVar(value="0.0800")
        self.kd_var = tk.StringVar(value="8.0")

        self.status_var = tk.StringVar(value="Не подключено")
        self.values_var = tk.StringVar(value="-")
        self._pid_refresh_counter = 0

        self.target_entry = None
        self.kp_entry = None
        self.ki_entry = None
        self.kd_entry = None
        self.heat_on_button = None
        self.heat_off_button = None
        self.motor_open_button = None
        self.motor_close_button = None
        self.motor_stop_button = None

        self.temp_history = deque(maxlen=3600)
        self.power_history = deque(maxlen=3600)
        self.target_history = deque(maxlen=3600)
        self.graph_canvas = None

        self._build_ui()
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

        ttk.Label(conn, text="COM:").grid(row=0, column=0, sticky="w")
        ttk.Entry(conn, textvariable=self.port_var, width=12).grid(row=0, column=1, sticky="w")
        ttk.Label(conn, text="Baud:").grid(row=0, column=2, sticky="w", padx=(8, 0))
        ttk.Entry(conn, textvariable=self.baud_var, width=10).grid(row=0, column=3, sticky="w")
        ttk.Label(conn, text="Slave ID:").grid(row=0, column=4, sticky="w", padx=(8, 0))
        ttk.Entry(conn, textvariable=self.slave_var, width=8).grid(row=0, column=5, sticky="w")

        ttk.Button(conn, text="Подключить", command=self.connect).grid(row=0, column=6, padx=(8, 0))
        ttk.Button(conn, text="Отключить", command=self.disconnect).grid(row=0, column=7, padx=(4, 0))

        ctrl = ttk.LabelFrame(frm, text="Управление", padding=8)
        ctrl.grid(row=1, column=0, sticky="ew", pady=(8, 0))

        ttk.Label(ctrl, text="Уставка °C:").grid(row=0, column=0, sticky="w")
        self.target_entry = ttk.Entry(ctrl, textvariable=self.target_var, width=10)
        self.target_entry.grid(row=0, column=1, sticky="w")
        ttk.Button(ctrl, text="Записать уставку", command=self.write_target).grid(row=0, column=2, padx=6)

        ttk.Label(ctrl, text="Kp:").grid(row=1, column=0, sticky="w")
        self.kp_entry = ttk.Entry(ctrl, textvariable=self.kp_var, width=10)
        self.kp_entry.grid(row=1, column=1, sticky="w")
        ttk.Label(ctrl, text="Ki:").grid(row=1, column=2, sticky="w")
        self.ki_entry = ttk.Entry(ctrl, textvariable=self.ki_var, width=10)
        self.ki_entry.grid(row=1, column=3, sticky="w")
        ttk.Label(ctrl, text="Kd:").grid(row=1, column=4, sticky="w")
        self.kd_entry = ttk.Entry(ctrl, textvariable=self.kd_var, width=10)
        self.kd_entry.grid(row=1, column=5, sticky="w")
        ttk.Button(ctrl, text="Записать PID", command=self.write_pid).grid(row=1, column=6, padx=6)

        self.heat_on_button = ttk.Button(ctrl, text="Нагрев ON", command=lambda: self.set_heating(True))
        self.heat_on_button.grid(row=2, column=0, pady=8)
        self.heat_off_button = ttk.Button(ctrl, text="Нагрев OFF", command=lambda: self.set_heating(False))
        self.heat_off_button.grid(row=2, column=1, pady=8)
        ttk.Button(ctrl, text="Автотюнинг START", command=lambda: self.send_command(CMD_AUTOTUNE_START)).grid(row=2, column=2, pady=8)
        ttk.Button(ctrl, text="Автотюнинг STOP", command=lambda: self.send_command(CMD_AUTOTUNE_STOP)).grid(row=2, column=3, pady=8)
        ttk.Button(ctrl, text="Сброс аварии", command=lambda: self.send_command(CMD_CLEAR_FAULT)).grid(row=2, column=4, pady=8)
        ttk.Button(ctrl, text="Сохранить PID", command=lambda: self.send_command(CMD_SAVE_PID)).grid(row=2, column=5, pady=8)
        self.motor_open_button = ttk.Button(ctrl, text="Мотор OPEN", command=lambda: self.send_command(CMD_MOTOR_OPEN))
        self.motor_open_button.grid(row=3, column=0, pady=8)
        self.motor_close_button = ttk.Button(ctrl, text="Мотор CLOSE", command=lambda: self.send_command(CMD_MOTOR_CLOSE))
        self.motor_close_button.grid(row=3, column=1, pady=8)
        self.motor_stop_button = ttk.Button(ctrl, text="Мотор STOP", command=lambda: self.send_command(CMD_MOTOR_STOP))
        self.motor_stop_button.grid(row=3, column=2, pady=8)

        stat = ttk.LabelFrame(frm, text="Статус", padding=8)
        stat.grid(row=2, column=0, sticky="ew", pady=(8, 0))
        ttk.Label(stat, textvariable=self.status_var).grid(row=0, column=0, sticky="w")
        ttk.Label(stat, textvariable=self.values_var, justify="left", anchor="w", wraplength=940).grid(row=1, column=0, sticky="w")

        graph = ttk.LabelFrame(frm, text="График температуры", padding=8)
        graph.grid(row=3, column=0, sticky="nsew", pady=(8, 0))
        graph.columnconfigure(0, weight=1)
        graph.rowconfigure(0, weight=1)

        self.graph_canvas = tk.Canvas(
            graph,
            width=960,
            height=260,
            bg="#ffffff",
            highlightthickness=1,
            highlightbackground="#cccccc",
        )
        self.graph_canvas.grid(row=0, column=0, sticky="nsew")

    def connect(self):
        self.disconnect()
        try:
            baud = int(self.baud_var.get())
            self.client = ModbusSerialClient(
                port=self.port_var.get().strip(),
                baudrate=baud,
                bytesize=8,
                parity="N",
                stopbits=1,
                timeout=1,
            )
            self.connected = bool(self.client.connect())
            self.status_var.set("Подключено" if self.connected else "Ошибка подключения")
        except Exception as exc:
            self.connected = False
            self.status_var.set(f"Ошибка: {exc}")

    def disconnect(self):
        if self.client:
            try:
                self.client.close()
            except Exception:
                pass
        self.client = None
        self.connected = False

    def _slave(self):
        return int(self.slave_var.get())

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
        except Exception as exc:
            messagebox.showerror("Modbus", str(exc))

    def set_heating(self, enabled: bool):
        try:
            # Heating has its own RW register. Writing it directly avoids ambiguity
            # with the one-shot command register and mirrors the web UI state.
            self.write_reg(REG_HEATING_ENABLE, 1 if enabled else 0)
        except Exception as exc:
            messagebox.showerror("Modbus", str(exc))

    def write_target(self):
        try:
            val = float(self.target_var.get())
            self.write_reg(REG_TARGET_X10, int(round(val * 10.0)) & 0xFFFF)
        except Exception as exc:
            messagebox.showerror("Modbus", str(exc))

    def write_pid(self):
        try:
            kp = float(self.kp_var.get())
            ki = float(self.ki_var.get())
            kd = float(self.kd_var.get())
            self.write_reg(REG_KP_X100, int(round(kp * 100.0)))
            self.write_reg(REG_KI_X10000, int(round(ki * 10000.0)))
            self.write_reg(REG_KD_X100, int(round(kd * 100.0)))
        except Exception as exc:
            messagebox.showerror("Modbus", str(exc))

    @staticmethod
    def _to_i16(v: int) -> int:
        return v - 65536 if v > 32767 else v

    @staticmethod
    def _decode_status_bits(status: int) -> str:
        flags = [
            ("HEAT_EN", bool(status & (1 << 0))),
            ("RELAY_ON", bool(status & (1 << 1))),
            ("AUTOTUNE", bool(status & (1 << 2))),
            ("FAULT", bool(status & (1 << 3))),
            ("THERMO_OPEN", bool(status & (1 << 4))),
            ("DI6_CLOSED", bool(status & (1 << 5))),
        ]
        return ", ".join(f"{name}={'1' if val else '0'}" for name, val in flags)

    def _entry_focused(self, entry_widget) -> bool:
        try:
            return self.root.focus_get() == entry_widget
        except Exception:
            return False

    def _draw_graph(self):
        if self.graph_canvas is None:
            return

        c = self.graph_canvas
        c.delete("all")

        w = max(100, int(c.winfo_width()))
        h = max(100, int(c.winfo_height()))
        left, right, top, bottom = 50, 44, 10, 28
        pw = max(10, w - left - right)
        ph = max(10, h - top - bottom)

        c.create_rectangle(left, top, left + pw, top + ph, outline="#999")

        vals = [v for v in self.temp_history if v is not None]
        target_vals = [v for v in self.target_history if v is not None]
        if not vals:
            c.create_text(w // 2, h // 2, text="Нет данных", fill="#666", font=("Segoe UI", 11))
            return

        scale_vals = vals + target_vals
        tmin = min(scale_vals)
        tmax = max(scale_vals)
        if abs(tmax - tmin) < 1.0:
            tmin -= 0.5
            tmax += 0.5
        margin = max(0.5, (tmax - tmin) * 0.1)
        tmin -= margin
        tmax += margin

        for i in range(5):
            y = top + int(ph * i / 4)
            t = tmax - (tmax - tmin) * i / 4
            c.create_line(left, y, left + pw, y, fill="#f0f0f0")
            c.create_text(left - 6, y, text=f"{t:.1f}", anchor="e", fill="#666", font=("Segoe UI", 9))

        # Right axis: power in %
        for i in range(5):
            y = top + int(ph * i / 4)
            p = 100 - int(100 * i / 4)
            c.create_text(left + pw + 6, y, text=f"{p}", anchor="w", fill="#1f5fa8", font=("Segoe UI", 9))

        n = len(self.temp_history)
        pts = []
        for i, t in enumerate(self.temp_history):
            if t is None:
                continue
            x = left + int(pw * i / max(1, n - 1))
            yn = (t - tmin) / max(0.001, (tmax - tmin))
            y = top + int(ph * (1.0 - yn))
            pts.append((x, y))

        if len(pts) >= 2:
            flat = []
            for p in pts:
                flat.extend(p)
            c.create_line(*flat, fill="#0f8a5f", width=2, smooth=True)

        # Noise-reduced temperature curve (EMA low-pass filter)
        filt_pts = []
        filt_temp = None
        temps = list(self.temp_history)
        for i, t in enumerate(temps):
            if t is None:
                continue
            if filt_temp is None:
                filt_temp = t
            else:
                filt_temp = (NOISE_FILTER_ALPHA * t) + ((1.0 - NOISE_FILTER_ALPHA) * filt_temp)
            x = left + int(pw * i / max(1, n - 1))
            yn = (filt_temp - tmin) / max(0.001, (tmax - tmin))
            y = top + int(ph * (1.0 - yn))
            filt_pts.append((x, y))

        if len(filt_pts) >= 2:
            flat = []
            for p in filt_pts:
                flat.extend(p)
            c.create_line(*flat, fill="#f59e0b", width=2, smooth=True)

        # Target temperature curve
        target_pts = []
        for i, target in enumerate(self.target_history):
            if target is None:
                continue
            x = left + int(pw * i / max(1, n - 1))
            yn = (target - tmin) / max(0.001, (tmax - tmin))
            y = top + int(ph * (1.0 - yn))
            target_pts.append((x, y))

        if len(target_pts) >= 2:
            flat = []
            for p in target_pts:
                flat.extend(p)
            c.create_line(*flat, fill="#dc2626", width=2)

        # Power curve (0..100%)
        p_pts = []
        for i, pwr in enumerate(self.power_history):
            if pwr is None:
                continue
            x = left + int(pw * i / max(1, n - 1))
            yn = max(0.0, min(1.0, pwr / 100.0))
            y = top + int(ph * (1.0 - yn))
            p_pts.append((x, y))

        if len(p_pts) >= 2:
            flat = []
            for p in p_pts:
                flat.extend(p)
            c.create_line(*flat, fill="#1f5fa8", width=2, smooth=True)

        c.create_text(left + 6, top + 12, anchor="w", text=f"Текущая: {vals[-1]:.2f} °C", fill="#0f8a5f", font=("Segoe UI", 10, "bold"))
        latest_target = next((v for v in reversed(self.target_history) if v is not None), None)
        if latest_target is not None:
            c.create_text(left + 220, top + 28, anchor="w", text=f"Уставка: {latest_target:.1f} °C", fill="#dc2626", font=("Segoe UI", 10, "bold"))
        latest_power = next((v for v in reversed(self.power_history) if v is not None), None)
        if latest_power is not None:
            c.create_text(left + 220, top + 12, anchor="w", text=f"Мощность: {latest_power:.1f} %", fill="#1f5fa8", font=("Segoe UI", 10, "bold"))
        c.create_text(left + pw - 430, top + 12, anchor="w", text="Темп. (зелёный), Фильтр (оранжевый), Уставка (красный), Мощн. (синий)", fill="#444", font=("Segoe UI", 9))

    def poll(self):
        if not self.connected or not self.client:
            return

        read_fn = self.client.read_holding_registers
        params = inspect.signature(read_fn).parameters
        kwargs = {}
        if "count" in params:
            kwargs["count"] = 16
        rr = self._call_with_slave(read_fn, 0, **kwargs)

        if rr.isError():
            self.status_var.set(f"Ошибка чтения: {rr}")
            return

        r = rr.registers
        status = r[REG_STATUS_BITS]
        temp_raw = r[REG_TEMPERATURE_X10]
        target = self._to_i16(r[REG_TARGET_X10]) / 10.0
        power = r[REG_POWER_X10] / 10.0
        kp = r[REG_KP_X100] / 100.0
        ki = r[REG_KI_X10000] / 10000.0
        kd = r[REG_KD_X100] / 100.0
        cmd_res = r[REG_COMMAND_RESULT]
        cycles = r[REG_AUTOTUNE_CYCLES]
        fault_code = r[REG_FAULT_CODE]
        motor_direction = r[REG_MOTOR_DIRECTION]
        motor_open_remaining = r[REG_MOTOR_OPEN_REMAINING_SEC]
        heating_enabled = bool(status & (1 << 0))
        heater_relay_on = bool(status & (1 << 1))
        autotune_active = bool(status & (1 << 2))
        lid_closed = bool(status & (1 << 5))

        cmd_res_text = CMD_RESULT_TEXT.get(cmd_res, f"UNKNOWN({cmd_res})")
        fault_text = FAULT_TEXT.get(fault_code, f"UNKNOWN({fault_code})")
        status_text = self._decode_status_bits(status)

        if temp_raw == 0x8000:
            temp_text = "N/A"
            self.temp_history.append(None)
        else:
            t = self._to_i16(temp_raw) / 10.0
            temp_text = f"{t:.1f}"
            self.temp_history.append(t)
        self.power_history.append(power)
        self.target_history.append(target)

        lines = [
            f"T={temp_text} °C  Target={target:.1f} °C  Power={power:.1f}%",
            f"Heating={'ON' if heating_enabled else 'OFF'}  L/N relay={'ON' if heater_relay_on else 'OFF'}  Autotune={'ON' if autotune_active else 'OFF'}",
            f"Kp={kp:.2f} Ki={ki:.4f} Kd={kd:.2f}",
            f"StatusBits=0x{status:04X} [{status_text}]",
            f"CmdResult={cmd_res} ({cmd_res_text})  Fault={fault_code} ({fault_text})  Cycles={cycles}",
            f"Motor={motor_direction} ({MOTOR_DIRECTION_TEXT.get(motor_direction, 'UNKNOWN')})  OpenRemaining={motor_open_remaining}s",
        ]
        self.values_var.set("\n".join(lines))

        if not self._entry_focused(self.target_entry):
            self.target_var.set(f"{target:.1f}")

        self._pid_refresh_counter += 1
        if self._pid_refresh_counter >= 5:
            self._pid_refresh_counter = 0
            if not self._entry_focused(self.kp_entry):
                self.kp_var.set(f"{kp:.2f}")
            if not self._entry_focused(self.ki_entry):
                self.ki_var.set(f"{ki:.4f}")
            if not self._entry_focused(self.kd_entry):
                self.kd_var.set(f"{kd:.2f}")

        if self.motor_open_button is not None:
            self.motor_open_button.state(["disabled"] if motor_direction == 1 or motor_open_remaining == 0 else ["!disabled"])
        if self.motor_close_button is not None:
            self.motor_close_button.state(["disabled"] if motor_direction == 2 or lid_closed else ["!disabled"])
        if self.motor_stop_button is not None:
            self.motor_stop_button.state(["disabled"] if motor_direction == 0 else ["!disabled"])
        if self.heat_on_button is not None:
            self.heat_on_button.state(["disabled"] if heating_enabled or autotune_active else ["!disabled"])
        if self.heat_off_button is not None:
            self.heat_off_button.state(["disabled"] if not heating_enabled else ["!disabled"])

        self._draw_graph()

    def _tick(self):
        try:
            self.poll()
        except Exception as exc:
            self.status_var.set(f"Ошибка: {exc}")
        self.root.after(1000, self._tick)


if __name__ == "__main__":
    root = tk.Tk()
    app = App(root)
    root.mainloop()
