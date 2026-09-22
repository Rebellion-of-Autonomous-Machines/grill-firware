"""Offline GUI/Modbus checks. Run: py -3 -B -m unittest discover -s test -p test_tkinter_thermostat.py"""
import sys
import tkinter as tk
import unittest
from pathlib import Path
from unittest.mock import patch

sys.path.insert(0, str(Path(__file__).resolve().parents[1] / "tools"))
import rs485_tester as ui


class Response:
    def __init__(self, registers=None, error=False):
        self.registers = registers
        self.error = error

    def isError(self):
        return self.error


class FakeClient:
    def __init__(self):
        self.registers = [
            3, 994, 1000, 20, 10, 10, 3000, 0, 0, 7, 0, 1, 1287, 0, 15,
            1, (-125) & 0xFFFF, 990, 975, 300, 150, 4,
        ]
        self.writes = []
        self.reads = []
        self.fail_write = False
        self.fail_read = False
        self.ignore_write = False
        self.closed = False

    def connect(self):
        return True

    def read_holding_registers(self, address, *, count=1, device_id=1):
        self.reads.append((address, count, device_id))
        if self.fail_read:
            raise TimeoutError("test timeout")
        return Response(self.registers[address:address + count])

    def write_register(self, address, value, *, device_id=1):
        self.writes.append((address, value, device_id))
        if self.fail_write:
            return Response(error=True)
        if not self.ignore_write:
            self.registers[address] = value
        return Response()

    def close(self):
        self.closed = True


class AppTest(unittest.TestCase):
    def setUp(self):
        self.clock = patch.object(ui.time, "monotonic", return_value=10000.0)
        self.clock.start()
        self.messages = patch.object(ui.messagebox, "showerror").start()
        self.root = tk.Tk()
        self.root.withdraw()
        self.app = ui.App(self.root)
        self.client = FakeClient()
        self.app.client = self.client
        self.app.connected = True
        self.root.update_idletasks()

    def tearDown(self):
        self.app.close()
        patch.stopall()

    def test_register_decoding_and_status(self):
        self.app.poll()
        self.assertEqual(self.client.reads, [(0, 22, 1)])
        self.assertEqual(len(self.app.history), 1)
        sample = self.app.history[-1]
        self.assertEqual(sample.temperature, 99.4)
        self.assertEqual(sample.target, 100)
        self.assertEqual(sample.on_threshold, 99)
        self.assertEqual(sample.off_threshold, 97.5)
        self.assertTrue(sample.contactor)
        self.assertTrue(self.app.anticipation_var.get())
        self.assertFalse(self.app.anticipation_check.instate(["disabled"]))
        self.assertIn("-1.25", self.app.anticipation_status_var.get())
        self.assertIn("4", self.app.learning_var.get())
        self.assertIn("15.0 / 30.0", self.app.learning_var.get())

    def test_toggle_roundtrip_does_not_add_a_graph_sample(self):
        self.app.poll()
        self.app.anticipation_check.invoke()
        self.assertEqual(self.client.writes, [(15, 0, 1)])
        self.assertFalse(self.app.anticipation_var.get())
        self.assertIn("выключена", self.app.learning_var.get())
        self.assertEqual(len(self.app.history), 1)
        self.app.anticipation_check.invoke()
        self.assertEqual(self.client.writes[-1], (15, 1, 1))
        self.assertTrue(self.app.anticipation_var.get())
        self.messages.assert_not_called()

    def test_rejected_toggle_reverts_and_reports_error(self):
        self.app.poll()
        self.client.fail_write = True
        self.app.anticipation_check.invoke()
        self.assertTrue(self.app.anticipation_var.get())
        self.assertTrue(self.app.anticipation_check.instate(["disabled"]))
        self.messages.assert_called_once()
        self.assertIn("нет актуальных", self.app.contactor_var.get())

    def test_unconfirmed_toggle_is_not_reported_as_success(self):
        self.app.poll()
        self.client.ignore_write = True
        self.app.anticipation_check.invoke()
        self.messages.assert_called_once()
        self.assertTrue(self.app.anticipation_var.get())
        self.assertIn("не подтвердил", self.app.status_var.get())

    def test_failed_readback_can_recover_on_next_poll(self):
        self.app.poll()
        self.client.fail_read = True
        self.app.anticipation_check.invoke()
        self.assertTrue(self.app.anticipation_var.get())
        self.client.fail_read = False
        self.app.poll()
        self.assertFalse(self.app.anticipation_var.get())
        self.assertFalse(self.app.anticipation_check.instate(["disabled"]))

    def test_multiple_edits_survive_poll_until_successful_write(self):
        self.app.poll()
        self.app.target_var.set("110,5")
        self.app.hysteresis_var.set("3,0")
        self.app.min_on_var.set("15")
        self.app.poll(record=False)
        self.assertEqual(self.app.target_var.get(), "110,5")
        self.assertEqual(self.app.hysteresis_var.get(), "3,0")
        self.app.write_target()
        self.assertEqual(self.client.registers[2], 1105)
        self.assertEqual(self.app.target_var.get(), "110.5")
        self.assertEqual(self.app.hysteresis_var.get(), "3,0")
        self.app.write_thermostat()
        self.assertEqual(self.client.registers[3:5], [30, 15])
        self.assertFalse(self.app._dirty_fields)
        self.messages.assert_not_called()

    def test_bad_values_never_write(self):
        self.app.poll()
        for value in ("nan", "inf", "-1", "301"):
            self.app.target_var.set(value)
            self.app.write_target()
        self.assertFalse(self.client.writes)
        self.assertEqual(self.messages.call_count, 4)
        self.app.min_off_var.set("-1")
        self.app.write_thermostat()
        self.assertFalse(self.client.writes)

    def test_failed_write_preserves_pending_edit(self):
        self.app.poll()
        self.client.fail_write = True
        self.app.target_var.set("115")
        self.app.write_target()
        self.app.poll(record=False)
        self.assertEqual(self.app.target_var.get(), "115")
        self.assertIn("target", self.app._dirty_fields)

    def test_sensor_fault_is_a_temperature_gap_not_zero(self):
        self.app.poll()
        self.client.registers[0] |= 1 << 4
        self.client.registers[1] = 0x8000
        self.app.poll()
        self.assertIsNone(self.app.history[-1].temperature)
        self.assertIsNone(self.app.history[-1].filtered)
        self.assertEqual(self.app.history[-1].target, 100)
        self.assertIn("приостановлена", self.app.learning_var.get())

    def test_disconnect_marks_state_unknown_and_preserves_history(self):
        self.app.poll()
        self.app.disconnect()
        self.assertTrue(self.client.closed)
        self.assertFalse(self.app.connected)
        self.assertIsNone(self.app.history[-1].contactor)
        self.assertEqual(self.app.history[0].temperature, 99.4)
        self.assertTrue(self.app.anticipation_check.instate(["disabled"]))
        self.assertTrue(self.app.heat_on_button.instate(["disabled"]))

    def test_bad_packet_is_reported(self):
        self.client.registers = self.client.registers[:16]
        with self.assertRaisesRegex(RuntimeError, "0...21"):
            self.app.poll()

    def test_hour_uses_elapsed_time_not_number_of_points(self):
        self.app._append_history(90, now=0)
        self.app._append_history(100, now=3599)
        self.assertEqual(len(self.app.history), 2)
        self.app._append_history(101, now=3601)
        self.assertEqual(len(self.app.history), 2)
        self.assertEqual(self.app.history[0].timestamp, 3599)
        self.app._prune_history(7202)
        self.assertFalse(self.app.history)

    def test_filter_resets_after_missing_data_or_long_gap(self):
        self.app._append_history(90, now=0)
        self.app._append_history(100, now=1)
        self.assertAlmostEqual(self.app.history[-1].filtered, 91.2)
        self.app._append_history(None, now=2)
        self.app._append_history(100, now=3)
        self.assertEqual(self.app.history[-1].filtered, 100)
        self.app._append_history(110, now=10)
        self.assertEqual(self.app.history[-1].filtered, 110)

    def test_graph_draws_thresholds_and_breaks_missing_segments(self):
        for timestamp, value in ((9990, 90), (9991, 91), (9992, None),
                                 (9993, 92), (9994, 93), (9999, 94), (10000, 95)):
            self.app._append_history(value, 100, 99, 97.5, True, now=timestamp)
        self.app._draw_graph(now=10000)
        canvas = self.app.graph_canvas
        lines = [item for item in canvas.find_withtag("temperature") if canvas.type(item) == "line"]
        self.assertEqual(len(lines), 3)
        self.assertTrue(canvas.find_withtag("on_threshold"))
        self.assertTrue(canvas.find_withtag("off_threshold"))
        self.assertTrue(canvas.find_withtag("contactor"))
        self.assertTrue(canvas.itemcget(canvas.find_withtag("on_threshold")[0], "dash"))

    def test_motor_interlocks_and_learning_pause_remain(self):
        self.client.registers[0] = 1 | (1 << 5)
        self.client.registers[13] = 2
        self.app.poll()
        self.assertTrue(self.app.motor_close_button.instate(["disabled"]))
        self.assertFalse(self.app.motor_stop_button.instate(["disabled"]))
        self.assertIn("движение крышки", self.app.learning_var.get())
        self.client.registers[13] = 0
        self.client.registers[14] = 0
        self.app.poll()
        self.assertTrue(self.app.motor_open_button.instate(["disabled"]))
        self.assertTrue(self.app.motor_stop_button.instate(["disabled"]))

    def test_full_hour_contactor_history_merges_to_two_intervals(self):
        for timestamp in range(6400, 10001):
            self.app._append_history(99.0, 100, 99, 98, timestamp < 8200, now=timestamp)
        self.app._draw_graph(now=10000)
        self.assertEqual(len(self.app.graph_canvas.find_withtag("contactor")), 2)
        self.assertEqual(len(self.app.history), 3601)

    def test_minimum_window_keeps_graph_visible(self):
        self.app.poll()
        self.root.attributes("-alpha", 0.0)
        self.root.geometry("1000x660")
        self.root.deiconify()
        self.root.update_idletasks()
        canvas = self.app.graph_canvas
        self.assertGreaterEqual(canvas.winfo_height(), 120)
        bottom = canvas.winfo_rooty() - self.root.winfo_rooty() + canvas.winfo_height()
        self.assertLessEqual(bottom, self.root.winfo_height())
        self.root.withdraw()

    def test_legacy_pymodbus_keywords(self):
        def with_slave(address, *, count, slave):
            return address, count, slave

        def with_unit(address, *, count, unit):
            return address, count, unit

        self.app.slave_var.set("7")
        for fn in (with_slave, with_unit):
            self.assertEqual(self.app._call_with_slave(fn, 0, count=22), (0, 22, 7))

    def test_connect_never_mixes_different_device_histories(self):
        with patch.object(ui, "ModbusTcpClient", return_value=self.client):
            self.app.connect()
        self.assertEqual(len(self.app.history), 1)
        self.app.host_var.set("192.168.1.52")
        new_client = FakeClient()
        new_client.registers[1] = 500
        with patch.object(ui, "ModbusTcpClient", return_value=new_client):
            self.app.connect()
        self.assertEqual(len(self.app.history), 1)
        self.assertEqual(self.app.history[0].temperature, 50)
        self.app.slave_var.set("9")
        self.app.poll(record=False)
        self.assertEqual(new_client.reads[-1][2], 1)  # Apply Unit ID only on reconnect.


if __name__ == "__main__":
    unittest.main()
