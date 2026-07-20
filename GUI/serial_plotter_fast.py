"""Qt live plotter and serial command console for the powertrain firmware (v2, high-performance).

Optimisations majeures par rapport à la version précédente :
- pyqtgraph au lieu de matplotlib : rendu GPU-friendly, 10-50x plus rapide pour
  du tracé temps réel (matplotlib redessine TOUTE la figure à chaque frame).
- Lecture série par blocs (ser.read(in_waiting)) au lieu de readline() :
  beaucoup moins d'appels système à haut débit.
- Ring buffer NumPy "miroir" (buffer doublé) : extraction des données ordonnées
  sans copie ni tri, et correction du bug d'ordre après wraparound.
- np.searchsorted (O(log n)) pour la fenêtre temporelle au lieu d'un masque
  booléen (O(n) + copie) à chaque frame.
- Downsampling "peak" + clipToView de pyqtgraph : le nombre de points tracés
  reste borné quelle que soit la taille du buffer.
- Timer d'affichage à 30 FPS découplé de l'acquisition (le thread série
  n'est jamais bloqué par le rendu).
- Auto-range Y natif pyqtgraph (throttlé en interne), axe X piloté manuellement.

Dépendances : pip install pyserial PySide6 pyqtgraph numpy
"""

from __future__ import annotations

import argparse
import csv
import queue
import sys
import threading
import time
from collections import deque
from dataclasses import dataclass
from typing import Dict, List, Optional

import numpy as np

try:
    import serial
    from serial.tools import list_ports
except ImportError as exc:  # pragma: no cover - handled at runtime
    raise SystemExit("pyserial is required. Install it with: pip install pyserial") from exc

try:
    from PySide6 import QtCore, QtGui, QtWidgets
except ImportError as exc:  # pragma: no cover - handled at runtime
    raise SystemExit("PySide6 is required. Install it with: pip install PySide6") from exc

try:
    import pyqtgraph as pg
except ImportError as exc:  # pragma: no cover - handled at runtime
    raise SystemExit(
        "pyqtgraph is required for fast plotting. Install it with: pip install pyqtgraph"
    ) from exc


LOG_MAX_BLOCKS = 1000
GUI_UPDATE_MS = 33          # ~30 FPS pour un affichage fluide
LOG_FLUSH_MS = 100          # flush des logs (moins critique que le tracé)
STATUS_UPDATE_MS = 250      # le label de statut n'a pas besoin de 30 FPS
MIN_PLOT_HEIGHT = 220       # hauteur plancher par graphe avant défilement

# Canaux stockés dans le ring buffer (ordre fixe)
CH_T, CH_TGT, CH_IQ, CH_VEL, CH_POS, CH_VBUS, CH_MODE = range(7)
N_CHANNELS = 7

PLOT_DEFS = [
    ("Target", "#f97316", CH_TGT),
    ("Iq [A]", "#22c55e", CH_IQ),
    ("Vel [rad/s]", "#3b82f6", CH_VEL),
    ("Pos [rad]", "#a855f7", CH_POS),
    ("Vbus [V]", "#ef4444", CH_VBUS),
]


@dataclass
class SerialMessage:
    timestamp_s: float
    raw_line: str
    fields: Optional[Dict[str, float]]


def fast_parse_line(line: str) -> Optional[Dict[str, float]]:
    """Fast manual parser for 'key=value key=value' telemetry lines."""
    fields: Dict[str, float] = {}
    i = 0
    n = len(line)

    while i < n:
        while i < n and line[i] in " \t":
            i += 1
        if i >= n:
            break

        start = i
        while i < n and (line[i].isalnum() or line[i] == "_"):
            i += 1
        if i == start:
            i += 1
            continue
        key = line[start:i]

        while i < n and line[i] in " \t":
            i += 1
        if i >= n or line[i] != "=":
            continue
        i += 1

        while i < n and line[i] in " \t":
            i += 1
        start = i
        while i < n and line[i] not in " \t":
            i += 1
        if i == start:
            continue

        try:
            fields[key] = float(line[start:i])
        except ValueError:
            continue

    return fields or None


@dataclass
class PortInfo:
    device: str
    description: str
    score: int  # score de priorité pour l'auto-sélection (ESP32/STM32)


# (mots-clés dans la description/le fabricant, VID USB) -> score
_KNOWN_TARGETS = [
    # STM32 : VCP natif ou ST-Link (VID STMicroelectronics 0x0483)
    (("stm32", "stlink", "st-link", "stmicroelectronics"), 0x0483, 100),
    # ESP32 : USB-JTAG natif Espressif (VID 0x303A)
    (("esp32", "espressif", "usb jtag"), 0x303A, 100),
    # Ponts USB-série typiques des cartes ESP32 (CP210x Silicon Labs, CH340/CH9102 WCH)
    (("cp210", "silicon labs",), 0x10C4, 80),
    (("ch340", "ch910", "wch",), 0x1A86, 80),
    # FTDI : fréquent sur les cartes de dev, priorité moindre
    (("ftdi", "ft232"), 0x0403, 50),
]


def _score_port(port) -> int:
    text = " ".join(
        s.lower() for s in (port.description or "", port.manufacturer or "", port.product or "") if s
    )
    best = 0
    for keywords, vid, score in _KNOWN_TARGETS:
        if (port.vid is not None and port.vid == vid) or any(k in text for k in keywords):
            best = max(best, score)
    return best


def list_serial_ports() -> List[PortInfo]:
    infos = []
    for port in list_ports.comports():
        desc = (port.description or "").strip()
        if desc.lower() in ("", "n/a"):
            desc = port.manufacturer or ""
        infos.append(PortInfo(port.device, desc, _score_port(port)))
    return infos


class MirroredRingBuffer:
    """Ring buffer NumPy 'miroir' : chaque échantillon est écrit deux fois
    (à idx et idx+capacity), ce qui permet d'obtenir une vue CONTIGUË et
    ORDONNÉE des données sans copie ni np.concatenate, même après wraparound.
    """

    def __init__(self, capacity: int, n_channels: int):
        self.capacity = capacity
        self.data = np.zeros((n_channels, 2 * capacity), dtype=np.float64)
        self.idx = 0
        self.count = 0

    def append(self, values: np.ndarray) -> None:
        i = self.idx
        self.data[:, i] = values
        self.data[:, i + self.capacity] = values
        self.idx = (i + 1) % self.capacity
        if self.count < self.capacity:
            self.count += 1

    def ordered_view(self) -> np.ndarray:
        """Vue (n_channels, count) des échantillons du plus ancien au plus récent."""
        if self.count < self.capacity:
            return self.data[:, : self.count]
        return self.data[:, self.idx : self.idx + self.capacity]

    def clear(self) -> None:
        self.idx = 0
        self.count = 0


class SerialReader(threading.Thread):
    """Thread de lecture série optimisé : lit par blocs (in_waiting) au lieu
    de readline(), et parse les lignes hors du thread GUI."""

    def __init__(self, port: str, baud: int, output: "queue.Queue[Optional[SerialMessage]]"):
        super().__init__(daemon=True)
        self.port = port
        self.baud = baud
        self.output = output
        self.stop_event = threading.Event()
        self.serial_port: Optional[serial.Serial] = None
        self._write_lock = threading.Lock()

    def run(self) -> None:
        buf = b""
        try:
            with serial.Serial(self.port, self.baud, timeout=0.05) as ser:
                self.serial_port = ser
                ser.reset_input_buffer()
                while not self.stop_event.is_set():
                    # Lecture par blocs : draine tout ce qui est disponible
                    chunk = ser.read(ser.in_waiting or 1)
                    if not chunk:
                        continue
                    buf += chunk
                    if b"\n" not in buf:
                        continue
                    *lines, buf = buf.split(b"\n")
                    now = time.time()
                    for raw in lines:
                        line = raw.decode("utf-8", errors="replace").strip()
                        if not line:
                            continue
                        fields = fast_parse_line(line)
                        self.output.put(SerialMessage(now, line, fields))
        except Exception as exc:
            self.output.put(None)
            print(f"Serial reader stopped: {exc}", file=sys.stderr)
        finally:
            self.serial_port = None

    def stop(self) -> None:
        self.stop_event.set()

    def write_line(self, text: str) -> None:
        ser = self.serial_port
        if ser is None:
            raise RuntimeError("Serial port is not open")
        with self._write_lock:
            ser.write((text.rstrip("\r\n") + "\n").encode("utf-8"))


class ScrollableGraphicsLayoutWidget(pg.GraphicsLayoutWidget):
    """GraphicsLayoutWidget qui redirige la molette vers le QScrollArea parent.

    pg.GraphicsLayoutWidget dérive de QGraphicsView, qui ACCEPTE l'événement
    molette (il s'en sert pour zoomer) même quand il n'a rien à faire défiler.
    Résultat : le QScrollArea englobant ne le reçoit jamais et la colonne de
    graphes paraît « non défilable ».

    Un simple event.ignore() ne suffit pas : la remontée automatique vers le
    parent n'est pas fiable ici (et encore moins sous WebAssembly). On pilote
    donc directement la barre de défilement du QScrollArea englobant.
    """

    def _parent_scroll_area(self) -> Optional[QtWidgets.QScrollArea]:
        widget = self.parentWidget()
        while widget is not None:
            if isinstance(widget, QtWidgets.QScrollArea):
                return widget
            widget = widget.parentWidget()
        return None

    def wheelEvent(self, event) -> None:  # noqa: N802 - Qt API name
        area = self._parent_scroll_area()
        if area is None:
            event.ignore()
            return
        bar = area.verticalScrollBar()
        bar.setValue(bar.value() - event.angleDelta().y())
        event.accept()


class PlotWindow(QtWidgets.QMainWindow):
    def __init__(self, window_s: float, title: str, baud: int, initial_port: Optional[str]):
        super().__init__()
        self.window_s = window_s
        # Capacité dimensionnée large (jusqu'à ~1 kHz de télémétrie sur la fenêtre max)
        capacity = max(2000, int(300.0 * 1000))
        self.buffer = MirroredRingBuffer(capacity, N_CHANNELS)

        self.t0: Optional[float] = None
        self.reader: Optional[SerialReader] = None
        self.samples: "queue.Queue[Optional[SerialMessage]]" = queue.Queue()
        self.current_port: Optional[str] = None
        self.log_messages: deque[SerialMessage] = deque(maxlen=100_000)
        self.pending_logs: List[str] = []
        self._sample_scratch = np.zeros(N_CHANNELS)
        self._last_status = 0.0

        self.setWindowTitle(title)
        self.resize(1280, 860)

        central = QtWidgets.QWidget(self)
        self.setCentralWidget(central)
        main_layout = QtWidgets.QHBoxLayout(central)

        splitter = QtWidgets.QSplitter(QtCore.Qt.Orientation.Horizontal)
        main_layout.addWidget(splitter)

        left_panel = self._create_left_panel()
        plot_widget = self._create_plot_widget(title)

        # Les deux moitiés défilent indépendamment : la colonne de contrôles est
        # plus haute qu'une fenêtre courte, et les 5 graphes empilés doivent
        # garder une hauteur lisible au lieu d'être écrasés.
        self.left_scroll = QtWidgets.QScrollArea()
        self.left_scroll.setWidgetResizable(True)
        self.left_scroll.setFrameShape(QtWidgets.QFrame.Shape.NoFrame)
        self.left_scroll.setWidget(left_panel)

        self.plot_scroll = QtWidgets.QScrollArea()
        self.plot_scroll.setWidgetResizable(True)
        self.plot_scroll.setFrameShape(QtWidgets.QFrame.Shape.NoFrame)
        self.plot_scroll.setHorizontalScrollBarPolicy(
            QtCore.Qt.ScrollBarPolicy.ScrollBarAlwaysOff
        )
        # Plancher de hauteur : en dessous, on défile au lieu de rétrécir.
        # Volontairement plus haut que la plupart des écrans avec 5 graphes,
        # pour que chaque courbe reste lisible.
        plot_widget.setMinimumHeight(MIN_PLOT_HEIGHT * len(PLOT_DEFS))
        self.plot_scroll.setWidget(plot_widget)

        splitter.addWidget(self.left_scroll)
        splitter.addWidget(self.plot_scroll)
        splitter.setStretchFactor(0, 0)
        splitter.setStretchFactor(1, 1)
        splitter.setSizes([380, 900])
        self.setMinimumWidth(700)

        # Le bouton doit vivre HORS du panneau, sinon le masquer emporterait
        # aussi le moyen de le ré-afficher.
        toolbar = self.addToolBar("View")
        toolbar.setMovable(False)
        self.toggle_panel_action = toolbar.addAction("Hide panel")
        self.toggle_panel_action.setCheckable(True)
        self.toggle_panel_action.setChecked(True)
        self.toggle_panel_action.setShortcut("Ctrl+B")
        self.toggle_panel_action.setToolTip("Afficher/masquer le panneau de contrôles (Ctrl+B)")
        self.toggle_panel_action.toggled.connect(self.on_toggle_panel)

        self._connect_signals()

        # Timer de flush des logs (découplé du tracé)
        self.log_timer = QtCore.QTimer(self)
        self.log_timer.setInterval(LOG_FLUSH_MS)
        self.log_timer.timeout.connect(self._flush_logs)
        self.log_timer.start()

        # Timer d'affichage ~30 FPS
        self.timer = QtCore.QTimer(self)
        self.timer.setInterval(GUI_UPDATE_MS)
        self.timer.timeout.connect(self.process_samples)

        self.refresh_ports()
        if initial_port:
            if not self._select_device(initial_port):
                self.port_combo.insertItem(0, initial_port, userData=initial_port)
                self.port_combo.setCurrentIndex(0)
            QtCore.QTimer.singleShot(0, self.toggle_start_stop)

    # ------------------------------------------------------------------ UI --

    def _create_left_panel(self) -> QtWidgets.QWidget:
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
        self.baud_spin.setValue(115200)
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
        self.clear_button = QtWidgets.QPushButton("Clear Graphs + Monitor")
        self.save_log_button = QtWidgets.QPushButton("Save CSV")
        self.exit_button = QtWidgets.QPushButton("Exit")
        self.window_spin = QtWidgets.QDoubleSpinBox()
        self.window_spin.setRange(1.0, 300.0)
        self.window_spin.setSingleStep(1.0)
        self.window_spin.setDecimals(1)
        self.window_spin.setValue(self.window_s)
        self.pause_plot_check = QtWidgets.QCheckBox("Freeze plots (keep logging)")
        control_layout.addWidget(QtWidgets.QLabel("Time window (s)"), 0, 0)
        control_layout.addWidget(self.window_spin, 0, 1)
        control_layout.addWidget(self.start_stop_button, 1, 0)
        control_layout.addWidget(self.exit_button, 1, 1)
        control_layout.addWidget(self.clear_button, 2, 0, 1, 2)
        control_layout.addWidget(self.save_log_button, 3, 0, 1, 2)
        control_layout.addWidget(self.pause_plot_check, 4, 0, 1, 2)

        command_group = QtWidgets.QGroupBox("Serial Commands")
        command_layout = QtWidgets.QVBoxLayout(command_group)
        self.command_edit = QtWidgets.QLineEdit()
        self.command_edit.setPlaceholderText("Type a command, for example: A, I, M, C, T1.5, V10")
        self.send_button = QtWidgets.QPushButton("Send")
        quick_row = QtWidgets.QHBoxLayout()
        for label in ("A (init)", "I (stop)", "M (measure)", "C (reset)"):
            button = QtWidgets.QPushButton(label)
            button.clicked.connect(lambda checked=False, cmd=label.split()[0]: self.send_command(cmd))
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
        self.log_view.setMaximumBlockCount(LOG_MAX_BLOCKS)
        # Retour à la ligne automatique pour que les longues lignes restent visibles
        self.log_view.setLineWrapMode(QtWidgets.QPlainTextEdit.LineWrapMode.WidgetWidth)
        self.log_view.setWordWrapMode(QtGui.QTextOption.WrapMode.WrapAnywhere)
        # Sans plancher, la zone de log s'écrase et le panneau n'a jamais
        # besoin de défiler.
        self.log_view.setMinimumHeight(200)
        logs_layout.addWidget(self.log_view)

        left_layout.addWidget(connection_group)
        left_layout.addWidget(control_group)
        left_layout.addWidget(command_group)
        left_layout.addWidget(plot_info_group)
        left_layout.addWidget(logs_group)
        left_layout.addStretch(1)

        return left_panel

    def _create_plot_widget(self, title: str) -> QtWidgets.QWidget:
        pg.setConfigOptions(antialias=False, background="w", foreground="k")

        self.glw = ScrollableGraphicsLayoutWidget()
        self.glw.addLabel(title, row=0, col=0)

        self.plots: List[pg.PlotItem] = []
        self.curves: List[pg.PlotDataItem] = []
        for row, (label, color, _channel) in enumerate(PLOT_DEFS, start=1):
            plot = self.glw.addPlot(row=row, col=0)
            plot.showGrid(x=True, y=True, alpha=0.25)
            plot.setLabel("left", label)
            plot.setMenuEnabled(False)
            plot.hideButtons()
            # Downsampling par pics : nombre de points tracés borné par la
            # largeur en pixels, quel que soit le débit de télémétrie.
            plot.setDownsampling(mode="peak", auto=True)
            plot.setClipToView(True)
            # Auto-range vertical natif (throttlé par pyqtgraph), X piloté à la main
            plot.enableAutoRange(axis="y")
            plot.setMouseEnabled(x=False, y=False)
            curve = plot.plot(pen=pg.mkPen(color, width=1.5))
            if row > 1:
                plot.setXLink(self.plots[0])
            if row < len(PLOT_DEFS):
                plot.getAxis("bottom").setStyle(showValues=False)
            self.plots.append(plot)
            self.curves.append(curve)

        self.plots[-1].setLabel("bottom", "Time [s]")
        self.plots[0].setXRange(0, self.window_s, padding=0)
        return self.glw

    def _connect_signals(self) -> None:
        self.refresh_button.clicked.connect(self.refresh_ports)
        self.connect_button.clicked.connect(self.toggle_connection)
        self.start_stop_button.clicked.connect(self.toggle_start_stop)
        self.clear_button.clicked.connect(self.clear_graphs_and_monitor)
        self.save_log_button.clicked.connect(self.save_logs_csv)
        self.exit_button.clicked.connect(self.close)
        self.send_button.clicked.connect(self.send_from_edit)
        self.command_edit.returnPressed.connect(self.send_from_edit)
        self.window_spin.valueChanged.connect(self.on_window_changed)

    # ------------------------------------------------------------- helpers --

    def refresh_ports(self) -> None:
        previous_device = self.selected_port()
        self.port_combo.clear()
        ports = list_serial_ports()
        if not ports:
            self.port_combo.addItem("No serial ports found")
            self.update_status("Ports refreshed — no serial ports found")
            return

        for info in ports:
            label = f"{info.device} — {info.description}" if info.description else info.device
            # Le device réel est stocké en itemData : le texte affiché peut
            # rester descriptif sans casser l'ouverture du port.
            self.port_combo.addItem(label, userData=info.device)

        # Auto-sélection : d'abord une cible connue (ESP32/STM32 ou pont
        # USB-série associé), sinon on garde le port précédemment choisi.
        best = max(ports, key=lambda p: p.score)
        if best.score > 0:
            self._select_device(best.device)
            self.update_status(f"Ports refreshed — auto-selected {best.device} ({best.description})")
        elif previous_device and self._select_device(previous_device):
            self.update_status("Ports refreshed")
        else:
            self.update_status("Ports refreshed — no ESP32/STM32 detected")

    def _select_device(self, device: str) -> bool:
        index = self.port_combo.findData(device)
        if index < 0:
            return False
        self.port_combo.setCurrentIndex(index)
        return True

    def selected_port(self) -> Optional[str]:
        device = self.port_combo.currentData()
        if isinstance(device, str) and device.strip():
            return device.strip()
        # Repli : entrée insérée manuellement (par ex. via --port) sans userData
        text = self.port_combo.currentText().strip()
        if not text or text == "No serial ports found":
            return None
        return text.split(" — ", 1)[0]

    def update_status(self, text: str) -> None:
        self.status_label.setText(text)

    def append_log(self, line: str) -> None:
        # On n'affiche pas les lignes de télémétrie (commençant par "t=") dans
        # le moniteur, uniquement les messages texte du firmware.
        if not line.startswith("t="):
            self.pending_logs.append(line)

    def _flush_logs(self) -> None:
        if not self.pending_logs:
            return
        # Un seul appendPlainText pour tout le batch : Qt ne re-layout qu'une fois
        if len(self.pending_logs) > LOG_MAX_BLOCKS:
            del self.pending_logs[: len(self.pending_logs) - LOG_MAX_BLOCKS]
        self.log_view.appendPlainText("\n".join(self.pending_logs))
        self.pending_logs.clear()
        scrollbar = self.log_view.verticalScrollBar()
        scrollbar.setValue(scrollbar.maximum())

    def save_logs_csv(self) -> None:
        path, _selected_filter = QtWidgets.QFileDialog.getSaveFileName(
            self,
            "Save serial logs as CSV",
            "serial_logs.csv",
            "CSV Files (*.csv);;All Files (*)",
        )
        if not path:
            return
        if not path.lower().endswith(".csv"):
            path += ".csv"

        fieldnames = ["timestamp_s", "raw_line", "t", "mode", "tgt", "Iq", "vel", "pos", "Vbus"]
        with open(path, "w", newline="", encoding="utf-8") as file_handle:
            writer = csv.DictWriter(file_handle, fieldnames=fieldnames, extrasaction="ignore")
            writer.writeheader()
            for message in self.log_messages:
                row = {"timestamp_s": message.timestamp_s, "raw_line": message.raw_line}
                if message.fields:
                    row.update(message.fields)
                writer.writerow(row)
        self.update_status(f"Saved {len(self.log_messages)} log lines to {path}")

    def clear_graphs_and_monitor(self) -> None:
        self.buffer.clear()
        self.t0 = None
        self.log_messages.clear()
        self.log_view.clear()
        self.pending_logs.clear()
        empty = np.empty(0)
        for curve in self.curves:
            curve.setData(empty, empty)
        self.plots[0].setXRange(0, self.window_s, padding=0)
        self.update_status("Cleared graphs and serial monitor")

    def set_connection_controls(self, connected: bool) -> None:
        self.connect_button.setText("Disconnect" if connected else "Connect")
        self.port_combo.setEnabled(not connected)
        self.refresh_button.setEnabled(not connected)
        self.baud_spin.setEnabled(not connected)

    def set_running_controls(self, running: bool) -> None:
        self.start_stop_button.setText("Stop" if running else "Start")

    # ------------------------------------------------------ stream control --

    def start_stream(self) -> None:
        if self.reader is None:
            port = self.selected_port()
            if port is None:
                self.update_status("No serial port selected")
                return
            self.samples = queue.Queue()
            self.t0 = None
            self.buffer.clear()
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
        self.command_edit.clear()

    def on_window_changed(self, value: float) -> None:
        self.window_s = float(value)

    def on_toggle_panel(self, visible: bool) -> None:
        """Masque/affiche la colonne de contrôles pour élargir les graphes."""
        self.left_scroll.setVisible(visible)
        self.toggle_panel_action.setText("Hide panel" if visible else "Show panel")

    # ---------------------------------------------------------- main loop --

    def process_samples(self) -> None:
        """Draine la queue série, met à jour le ring buffer puis les courbes."""
        new_data = False
        scratch = self._sample_scratch

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

            self.log_messages.append(item)
            self.append_log(item.raw_line)

            fields = item.fields
            if fields is None:
                continue
            t_ms = fields.get("t")
            if t_ms is None:
                continue

            if self.t0 is None:
                self.t0 = t_ms / 1000.0

            scratch[CH_T] = (t_ms / 1000.0) - self.t0
            scratch[CH_TGT] = fields.get("tgt", 0.0)
            scratch[CH_IQ] = fields.get("Iq", fields.get("iq", 0.0))
            scratch[CH_VEL] = fields.get("vel", 0.0)
            scratch[CH_POS] = fields.get("pos", 0.0)
            scratch[CH_VBUS] = fields.get("Vbus", fields.get("vbus", 0.0))
            scratch[CH_MODE] = fields.get("mode", float("nan"))
            self.buffer.append(scratch)
            new_data = True

        if not new_data or self.buffer.count == 0:
            return
        if self.pause_plot_check.isChecked():
            return

        view = self.buffer.ordered_view()  # vue contiguë, sans copie
        x = view[CH_T]
        t_last = x[-1]
        t_start = t_last - self.window_s

        # Recherche binaire du début de fenêtre : O(log n), pas de masque booléen
        i0 = int(np.searchsorted(x, t_start, side="left"))
        x_win = x[i0:]

        for curve, (_label, _color, channel) in zip(self.curves, PLOT_DEFS):
            curve.setData(x_win, view[channel, i0:])

        # Axe X qui défile ; les axes Y sont gérés par l'auto-range pyqtgraph
        self.plots[0].setXRange(max(0.0, t_start), max(self.window_s, t_last), padding=0)

        now = time.monotonic()
        if now - self._last_status > STATUS_UPDATE_MS / 1000.0 and self.current_port:
            self._last_status = now
            last_mode = view[CH_MODE, -1]
            self.update_status(
                f"Port {self.current_port} | samples={self.buffer.count} "
                f"| last t={t_last:.2f}s | mode={last_mode:g}"
            )

    def closeEvent(self, event) -> None:  # noqa: N802 - Qt API name
        self.timer.stop()
        self.log_timer.stop()
        self.disconnect_serial()
        super().closeEvent(event)


def main() -> int:
    parser = argparse.ArgumentParser(description="Qt live plot telemetry from the motor controller over serial.")
    parser.add_argument("--port", help="Serial port, for example COM7 or /dev/ttyACM0")
    parser.add_argument("--baud", type=int, default=115200, help="Serial baud rate (default: 115200)")
    parser.add_argument("--window", type=float, default=20.0, help="Time window to display in seconds (default: 20)")
    parser.add_argument("--title", default="Powertrain live plotter", help="Window title")
    args = parser.parse_args()

    app = QtWidgets.QApplication(sys.argv)
    window = PlotWindow(args.window, args.title, args.baud, args.port)
    window.show()
    return app.exec()


if __name__ == "__main__":
    raise SystemExit(main())
