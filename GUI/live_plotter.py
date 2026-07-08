"""Live serial plotter for the powertrain firmware.

The firmware emits lines like:
	t=1234 #42 mode=0 tgt=0.50 Iq=0.12 vel=1.20 Vbus=24.8 RUN

This script reads those lines over USB serial, parses the key/value fields,
and plots the telemetry in real time.
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
from typing import Deque, Dict, Optional

try:
	import serial
	from serial.tools import list_ports
except ImportError as exc:  # pragma: no cover - handled at runtime
	raise SystemExit(
		"pyserial is required. Install it with: pip install pyserial"
	) from exc

try:
	import matplotlib.pyplot as plt
	from matplotlib.animation import FuncAnimation
except ImportError as exc:  # pragma: no cover - handled at runtime
	raise SystemExit(
		"matplotlib is required. Install it with: pip install matplotlib"
	) from exc


KEY_VALUE_RE = re.compile(r"([A-Za-z_]+)=([^\s]+)")


@dataclass
class Sample:
	timestamp_s: float
	fields: Dict[str, float]


def parse_line(line: str) -> Optional[Dict[str, float]]:
	fields: Dict[str, float] = {}
	for key, value in KEY_VALUE_RE.findall(line):
		try:
			fields[key] = float(value)
		except ValueError:
			continue
	return fields or None


def choose_port(requested: Optional[str]) -> str:
	if requested:
		return requested

	ports = list(list_ports.comports())
	if not ports:
		raise SystemExit("No serial ports found. Pass --port COMx or /dev/ttyACM0.")

	if len(ports) == 1:
		return ports[0].device

	for index, port in enumerate(ports, start=1):
		desc = port.description or ""
		hwid = port.hwid or ""
		print(f"[{index}] {port.device}  {desc} {hwid}")

	while True:
		choice = input("Select port number: ").strip()
		if choice.isdigit():
			idx = int(choice)
			if 1 <= idx <= len(ports):
				return ports[idx - 1].device
		print("Invalid selection.")


class SerialReader(threading.Thread):
	def __init__(self, port: str, baud: int, output: "queue.Queue[Optional[Sample]]"):
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
					if fields:
						self.output.put(Sample(time.time(), fields))
		except Exception as exc:
			self.output.put(None)
			print(f"Serial reader stopped: {exc}", file=sys.stderr)

	def stop(self) -> None:
		self.stop_event.set()

	def write_line(self, text: str) -> None:
		if self.serial_port is None:
			return
		self.serial_port.write((text.rstrip("\r\n") + "\n").encode("utf-8"))


def build_series(capacity: int):
	return {
		"t_ms": deque(maxlen=capacity),
		"tgt": deque(maxlen=capacity),
		"iq": deque(maxlen=capacity),
		"vel": deque(maxlen=capacity),
		"vbus": deque(maxlen=capacity),
		"mode": deque(maxlen=capacity),
	}


def main() -> int:
	parser = argparse.ArgumentParser(description="Live plot telemetry from the motor controller over serial.")
	parser.add_argument("--port", help="Serial port, for example COM7 or /dev/ttyACM0")
	parser.add_argument("--baud", type=int, default=115200, help="Serial baud rate (default: 115200)")
	parser.add_argument("--window", type=float, default=20.0, help="Time window to display in seconds (default: 20)")
	parser.add_argument("--title", default="Powertrain live plot", help="Plot title")
	args = parser.parse_args()

	port = choose_port(args.port)
	print(f"Opening {port} at {args.baud} baud...")

	samples: "queue.Queue[Optional[Sample]]" = queue.Queue()
	reader = SerialReader(port, args.baud, samples)
	reader.start()

	capacity = max(100, int(args.window * 200))
	series = build_series(capacity)
	t0: Optional[float] = None

	fig, axes = plt.subplots(4, 1, sharex=True, figsize=(11, 8))
	fig.canvas.manager.set_window_title(args.title)
	fig.suptitle(args.title)

	(line_tgt,) = axes[0].plot([], [], label="target", color="#f97316")
	(line_iq,) = axes[1].plot([], [], label="Iq", color="#22c55e")
	(line_vel,) = axes[2].plot([], [], label="velocity", color="#3b82f6")
	(line_vbus,) = axes[3].plot([], [], label="Vbus", color="#ef4444")

	axes[0].set_ylabel("Target")
	axes[1].set_ylabel("Iq [A]")
	axes[2].set_ylabel("Vel [rad/s]")
	axes[3].set_ylabel("Vbus [V]")
	axes[3].set_xlabel("Time [s]")

	for ax in axes:
		ax.grid(True, alpha=0.25)
		ax.legend(loc="upper right")

	status = fig.text(0.01, 0.01, "Waiting for data...", fontsize=9)

	def update(_frame: int):
		nonlocal t0
		updated = False

		while True:
			try:
				item = samples.get_nowait()
			except queue.Empty:
				break

			if item is None:
				status.set_text("Serial connection ended.")
				return line_tgt, line_iq, line_vel, line_vbus, status

			if t0 is None and "t" in item.fields:
				t0 = item.fields["t"] / 1000.0

			if "t" in item.fields:
				elapsed = (item.fields["t"] / 1000.0) - (t0 or 0.0)
			else:
				elapsed = time.time()
				if t0 is None:
					t0 = elapsed
				elapsed -= t0

			series["t_ms"].append(elapsed)
			series["tgt"].append(item.fields.get("tgt", 0.0))
			series["iq"].append(item.fields.get("Iq", item.fields.get("iq", 0.0)))
			series["vel"].append(item.fields.get("vel", 0.0))
			series["vbus"].append(item.fields.get("Vbus", item.fields.get("vbus", 0.0)))
			series["mode"].append(item.fields.get("mode", float("nan")))
			updated = True

		if not series["t_ms"]:
			return line_tgt, line_iq, line_vel, line_vbus, status

		x = list(series["t_ms"])
		line_tgt.set_data(x, list(series["tgt"]))
		line_iq.set_data(x, list(series["iq"]))
		line_vel.set_data(x, list(series["vel"]))
		line_vbus.set_data(x, list(series["vbus"]))

		xmin = max(0.0, x[-1] - args.window)
		xmax = max(args.window * 0.1, x[-1])
		for ax in axes:
			ax.set_xlim(xmin, xmax)
			ax.relim()
			ax.autoscale_view(scalex=False, scaley=True)

		last_mode = series["mode"][-1]
		if updated:
			status.set_text(
				f"Port {port} | samples={len(x)} | last t={x[-1]:.2f}s | mode={last_mode:g}"
			)
		return line_tgt, line_iq, line_vel, line_vbus, status

	try:
		FuncAnimation(fig, update, interval=50, blit=False)
		plt.tight_layout(rect=(0, 0.03, 1, 0.95))
		plt.show()
	finally:
		reader.stop()

	return 0


if __name__ == "__main__":
	raise SystemExit(main())
