"""Qt live plotter and serial command console for the powertrain firmware.

The firmware emits lines like:
    t=1234 #42 mode=0 tgt=0.50 Iq=0.12 vel=1.20 Vbus=24.8 RUN

This tool reads those telemetry lines over USB serial, plots them in real time,
and provides a small console for sending serial commands back to the controller.
"""

from __future__ import annotations

import argparse
import queue
import re
import sys
import threading
import time
from collections import deque
from dataclasses import dataclass
from typing import Dict, List, Optional

try:
    import serial
    from serial.tools import list_ports
except ImportError as exc:  # pragma: no cover - handled at runtime
    raise SystemExit(
        "pyserial is required. Install it with: pip install pyserial"
    ) from exc

try:
    from matplotlib.backends.backend_qtagg import FigureCanvasQTAgg as FigureCanvas
    from matplotlib.backends.backend_qtagg import NavigationToolbar2QT as NavigationToolbar
    from matplotlib.figure import Figure
except ImportError as exc:  # pragma: no cover - handled at runtime
    raise SystemExit(
        "matplotlib Qt support is required. Install PySide6 and matplotlib."
    ) from exc

try:
    from PySide6 import QtCore, QtGui, QtWidgets
except ImportError as exc:  # pragma: no cover - handled at runtime
    raise SystemExit(
        "PySide6 is required. Install it with: pip install PySide6"
    ) from exc


KEY_VALUE_RE = re.compile(r"([A-Za-z_]+)=([^\s]+)")


@dataclass
class SerialMessage:
    timestamp_s: float
    raw_line: str
    fields: Optional[Dict[str, float]]


def parse_line(line: str) -> Optional[Dict[str, float]]:
    fields: Dict[str, float] = {}
    for key, value in KEY_VALUE_RE.findall(line):
        try:
            fields[key] = float(value)
        except ValueError:
            continue
    return fields or None


def list_serial_ports() -> List[str]:
    return [port.device for port in list_ports.comports()]


class SerialReader(threading.Thread):
    def __init__(self, port: str, baud: int, output: "queue.Queue[Optional[SerialMessage]]"):
        super().__init__(daemon=True)
        self.port = port
        self.baud = baud
        self.output = output
        self.stop_event = threading.Event()
        self.serial_port: Optional[serial.Serial] = None

    def run(self) -> None:
        try:
            with serial.Serial(self.port, self.baud, timeout=0.2) as ser:
                self.serial_port = ser
                ser.reset_input_buffer()
                while not self.stop_event.is_set():
                    raw = ser.readline()
                    if not raw:
                        continue
                    try:
                        line = raw.decode("utf-8", errors="replace").strip()
                    except Exception:
                        continue
                    fields = parse_line(line)
                    self.output.put(SerialMessage(time.time(), line, fields))
        except Exception as exc:
            self.output.put(None)
            print(f"Serial reader stopped: {exc}", file=sys.stderr)

    def stop(self) -> None:
        self.stop_event.set()

    def write_line(self, text: str) -> None:
        if self.serial_port is None:
            raise RuntimeError("Serial port is not open")
        self.serial_port.write((text.rstrip("\r\n") + "\n").encode("utf-8"))


def build_series(capacity: int):
    return {
        "t_ms": deque(maxlen=capacity),
        "tgt": deque(maxlen=capacity),
        "iq": deque(maxlen=capacity),
        "vel": deque(maxlen=capacity),
        "pos": deque(maxlen=capacity),
        "vbus": deque(maxlen=capacity),
        "mode": deque(maxlen=capacity),
    }


class PlotWindow(QtWidgets.QMainWindow):
    def __init__(self, window_s: float, title: str, baud: int, initial_port: Optional[str]):
        super().__init__()
        self.window_s = window_s
        self.capacity = max(100, int(window_s * 200))
        self.series = build_series(self.capacity)
        self.t0: Optional[float] = None
        self.reader: Optional[SerialReader] = None
        self.samples: "queue.Queue[Optional[SerialMessage]]" = queue.Queue()
        self.current_port: Optional[str] = None

        self.setWindowTitle(title)
        self.resize(1280, 860)

        central = QtWidgets.QWidget(self)
        self.setCentralWidget(central)
        main_layout = QtWidgets.QHBoxLayout(central)

        splitter = QtWidgets.QSplitter(QtCore.Qt.Orientation.Horizontal)
        main_layout.addWidget(splitter)

        left_panel = QtWidgets.QWidget()
        left_layout = QtWidgets.QVBoxLayout(left_panel)
        left_layout.setContentsMargins(0, 0, 0, 0)
        left_layout.setSpacing(10)

        connection_group = QtWidgets.QGroupBox("Connection")
        connection_layout = QtWidgets.QGridLayout(connection_group)
        self.port_combo = QtWidgets.QComboBox()
        self.refresh_button = QtWidgets.QPushButton("Refresh")
        self.baud_spin = QtWidgets.QSpinBox()
        self.baud_spin.setRange(300, 4_000_000)
        self.baud_spin.setValue(baud)
        self.connect_button = QtWidgets.QPushButton("Connect")
        connection_layout.addWidget(QtWidgets.QLabel("Port"), 0, 0)
        connection_layout.addWidget(self.port_combo, 0, 1)
        connection_layout.addWidget(self.refresh_button, 0, 2)
        connection_layout.addWidget(QtWidgets.QLabel("Baud"), 1, 0)
        connection_layout.addWidget(self.baud_spin, 1, 1)
        connection_layout.addWidget(self.connect_button, 1, 2)

        control_group = QtWidgets.QGroupBox("Controls")
        control_layout = QtWidgets.QGridLayout(control_group)
        self.start_stop_button = QtWidgets.QPushButton("Start")
        self.exit_button = QtWidgets.QPushButton("Exit")
        self.window_spin = QtWidgets.QDoubleSpinBox()
        self.window_spin.setRange(1.0, 300.0)
        self.window_spin.setSingleStep(1.0)
        self.window_spin.setDecimals(1)
        self.window_spin.setValue(window_s)
        control_layout.addWidget(QtWidgets.QLabel("Time window (s)"), 0, 0)
        control_layout.addWidget(self.window_spin, 0, 1)
        control_layout.addWidget(self.start_stop_button, 1, 0)
        control_layout.addWidget(self.exit_button, 1, 1)

        command_group = QtWidgets.QGroupBox("Serial Commands")
        command_layout = QtWidgets.QVBoxLayout(command_group)
        self.command_edit = QtWidgets.QLineEdit()
        self.command_edit.setPlaceholderText("Type a command, for example: A, I, M, C, T1.5, V10")
        self.send_button = QtWidgets.QPushButton("Send")
        quick_row = QtWidgets.QHBoxLayout()
        for label in ("A (init)", "I (stop)", "M (measure)", "C (reset)"):
            button = QtWidgets.QPushButton(label)
            button.clicked.connect(lambda checked=False, cmd=label: self.send_command(cmd))
            quick_row.addWidget(button)
        command_layout.addWidget(self.command_edit)
        command_layout.addWidget(self.send_button)
        command_layout.addLayout(quick_row)

        plot_info_group = QtWidgets.QGroupBox("Status")
        plot_info_layout = QtWidgets.QVBoxLayout(plot_info_group)
        self.status_label = QtWidgets.QLabel("Disconnected")
        self.status_label.setWordWrap(True)
        plot_info_layout.addWidget(self.status_label)

        logs_group = QtWidgets.QGroupBox("Serial Logs")
        logs_layout = QtWidgets.QVBoxLayout(logs_group)
        self.log_view = QtWidgets.QPlainTextEdit()
        self.log_view.setReadOnly(True)
        self.log_view.setMaximumBlockCount(1000)
        self.log_view.setLineWrapMode(QtWidgets.QPlainTextEdit.LineWrapMode.NoWrap)
        logs_layout.addWidget(self.log_view)

        left_layout.addWidget(connection_group)
        left_layout.addWidget(control_group)
        left_layout.addWidget(command_group)
        left_layout.addWidget(plot_info_group)
        left_layout.addWidget(logs_group)
        left_layout.addStretch(1)

        plot_widget = QtWidgets.QWidget()
        plot_layout = QtWidgets.QVBoxLayout(plot_widget)
        plot_layout.setContentsMargins(0, 0, 0, 0)

        self.figure = Figure(figsize=(11, 8), constrained_layout=True)
        self.canvas = FigureCanvas(self.figure)
        self.toolbar = NavigationToolbar(self.canvas, self)
        plot_layout.addWidget(self.toolbar)
        plot_layout.addWidget(self.canvas)

        self.axes = self.figure.subplots(5, 1, sharex=True)
        self.line_tgt = self.axes[0].plot([], [], label="target", color="#f97316")[0]
        self.line_iq = self.axes[1].plot([], [], label="Iq", color="#22c55e")[0]
        self.line_vel = self.axes[2].plot([], [], label="velocity", color="#3b82f6")[0]
        self.line_pos = self.axes[3].plot([], [], label="position", color="#a855f7")[0]
        self.line_vbus = self.axes[4].plot([], [], label="Vbus", color="#ef4444")[0]

        self.axes[0].set_ylabel("Target")
        self.axes[1].set_ylabel("Iq [A]")
        self.axes[2].set_ylabel("Vel [rad/s]")
        self.axes[3].set_ylabel("Pos [rad]")
        self.axes[4].set_ylabel("Vbus [V]")
        self.axes[4].set_xlabel("Time [s]")
        for ax in self.axes:
            ax.grid(True, alpha=0.25)
            ax.legend(loc="upper right")

        splitter.addWidget(left_panel)
        splitter.addWidget(plot_widget)
        splitter.setStretchFactor(0, 0)
        splitter.setStretchFactor(1, 1)
        self.setMinimumWidth(1100)

        self.refresh_button.clicked.connect(self.refresh_ports)
        self.connect_button.clicked.connect(self.toggle_connection)
        self.start_stop_button.clicked.connect(self.toggle_start_stop)
        self.exit_button.clicked.connect(self.close)
        self.send_button.clicked.connect(self.send_from_edit)
        self.command_edit.returnPressed.connect(self.send_from_edit)
        self.window_spin.valueChanged.connect(self.on_window_changed)

        self.refresh_ports()
        if initial_port:
            index = self.port_combo.findText(initial_port)
            if index >= 0:
                self.port_combo.setCurrentIndex(index)
            else:
                self.port_combo.insertItem(0, initial_port)
                self.port_combo.setCurrentIndex(0)

        self.timer = QtCore.QTimer(self)
        self.timer.setInterval(50)
        self.timer.timeout.connect(self.process_samples)

        self.figure.suptitle(title)

        if initial_port:
            QtCore.QTimer.singleShot(0, self.toggle_start_stop)

    def refresh_ports(self) -> None:
        selected = self.port_combo.currentText()
        self.port_combo.clear()
        ports = list_serial_ports()
        if ports:
            self.port_combo.addItems(ports)
            index = self.port_combo.findText(selected)
            if index >= 0:
                self.port_combo.setCurrentIndex(index)
        else:
            self.port_combo.addItem("No serial ports found")
        self.update_status("Ports refreshed")

    def selected_port(self) -> Optional[str]:
        port = self.port_combo.currentText().strip()
        if not port or port == "No serial ports found":
            return None
        return port

    def update_status(self, text: str) -> None:
        self.status_label.setText(text)

    def append_log(self, line: str) -> None:
        if not line[0] in ("t"):
            self.log_view.appendPlainText(line)
            cursor = self.log_view.textCursor()
            cursor.movePosition(QtGui.QTextCursor.MoveOperation.End)
            self.log_view.setTextCursor(cursor)
            self.log_view.ensureCursorVisible()

    def set_connection_controls(self, connected: bool) -> None:
        self.connect_button.setText("Disconnect" if connected else "Connect")
        self.port_combo.setEnabled(not connected)
        self.refresh_button.setEnabled(not connected)
        self.baud_spin.setEnabled(not connected)

    def set_running_controls(self, running: bool) -> None:
        self.start_stop_button.setText("Stop" if running else "Start")

    def start_stream(self) -> None:
        if self.reader is None:
            port = self.selected_port()
            if port is None:
                self.update_status("No serial port selected")
                return

            self.samples = queue.Queue()
            self.t0 = None
            self.series = build_series(self.capacity)
            self.reader = SerialReader(port, self.baud_spin.value(), self.samples)
            self.reader.start()
            self.current_port = port
            self.set_connection_controls(True)
            self.update_status(f"Connected to {port} @ {self.baud_spin.value()} baud")

        if not self.timer.isActive():
            self.timer.start()
        self.set_running_controls(True)

    def stop_stream(self) -> None:
        if self.timer.isActive():
            self.timer.stop()
        self.set_running_controls(False)
        self.update_status("Paused")

    def toggle_start_stop(self) -> None:
        if self.timer.isActive():
            self.stop_stream()
        else:
            self.start_stream()

    def toggle_connection(self) -> None:
        if self.reader is not None:
            self.disconnect_serial()
            return

        port = self.selected_port()
        if port is None:
            self.update_status("No serial port selected")
            return

        self.samples = queue.Queue()
        self.t0 = None
        self.series = build_series(self.capacity)
        self.reader = SerialReader(port, self.baud_spin.value(), self.samples)
        self.reader.start()
        self.current_port = port
        self.set_connection_controls(True)
        self.update_status(f"Connected to {port} @ {self.baud_spin.value()} baud")
        self.start_stream()

    def disconnect_serial(self) -> None:
        reader = self.reader
        if reader is None:
            return
        reader.stop()
        reader.join(timeout=0.5)
        self.reader = None
        self.current_port = None
        self.set_connection_controls(False)
        self.update_status("Disconnected")

    def send_command(self, command: str) -> None:
        text = command.strip()
        if not text:
            return
        reader = self.reader
        if reader is None or reader.serial_port is None:
            self.update_status("Connect to a serial port before sending commands")
            return
        try:
            reader.write_line(text)
            self.update_status(f"Sent: {text}")
        except Exception as exc:
            self.update_status(f"Send failed: {exc}")

    def send_from_edit(self) -> None:
        self.send_command(self.command_edit.text())

    def on_window_changed(self, value: float) -> None:
        self.window_s = float(value)

    def process_samples(self) -> None:
        updated = False
        while True:
            try:
                item = self.samples.get_nowait()
            except queue.Empty:
                break

            if item is None:
                self.disconnect_serial()
                self.set_running_controls(False)
                self.update_status("Serial connection ended")
                return

            self.append_log(item.raw_line)

            if item.fields is None:
                continue

            if self.t0 is None and "t" in item.fields:
                self.t0 = item.fields["t"] / 1000.0

            if "t" in item.fields:
                elapsed = (item.fields["t"] / 1000.0) - (self.t0 or 0.0)
            else:
                elapsed = time.time()
                if self.t0 is None:
                    self.t0 = elapsed
                elapsed -= self.t0

            self.series["t_ms"].append(elapsed)
            self.series["tgt"].append(item.fields.get("tgt", 0.0))
            self.series["iq"].append(item.fields.get("Iq", item.fields.get("iq", 0.0)))
            self.series["vel"].append(item.fields.get("vel", 0.0))
            self.series["pos"].append(item.fields.get("pos", 0.0))
            self.series["vbus"].append(item.fields.get("Vbus", item.fields.get("vbus", 0.0)))
            self.series["mode"].append(item.fields.get("mode", float("nan")))
            updated = True

        if not self.series["t_ms"]:
            return

        x = list(self.series["t_ms"])
        self.line_tgt.set_data(x, list(self.series["tgt"]))
        self.line_iq.set_data(x, list(self.series["iq"]))
        self.line_vel.set_data(x, list(self.series["vel"]))
        self.line_pos.set_data(x, list(self.series["pos"]))
        self.line_vbus.set_data(x, list(self.series["vbus"]))

        xmin = max(0.0, x[-1] - self.window_s)
        xmax = max(self.window_s * 0.1, x[-1])
        for ax in self.axes:
            ax.set_xlim(xmin, xmax)
            ax.relim()
            ax.autoscale_view(scalex=False, scaley=True)

        last_mode = self.series["mode"][-1]
        if self.current_port and updated:
            self.update_status(
                f"Port {self.current_port} | samples={len(x)} | last t={x[-1]:.2f}s | mode={last_mode:g}"
            )
        self.canvas.draw_idle()

    def closeEvent(self, event) -> None:  # noqa: N802 - Qt API name
        self.timer.stop()
        self.disconnect_serial()
        super().closeEvent(event)


def main() -> int:
    parser = argparse.ArgumentParser(description="Qt live plot telemetry from the motor controller over serial.")
    parser.add_argument("--port", help="Serial port, for example COM7 or /dev/ttyACM0")
    parser.add_argument("--baud", type=int, default=115200, help="Serial baud rate (default: 115200)")
    parser.add_argument("--window", type=float, default=20.0, help="Time window to display in seconds (default: 20)")
    parser.add_argument("--title", default="Powertrain live plot", help="Window title")
    args = parser.parse_args()

    app = QtWidgets.QApplication(sys.argv)
    window = PlotWindow(args.window, args.title, args.baud, args.port)
    window.show()
    return app.exec()


if __name__ == "__main__":
    raise SystemExit(main())