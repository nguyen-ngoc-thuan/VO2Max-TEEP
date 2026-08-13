#!/usr/bin/env python3
"""
VO2Max-TEEP Real-time Monitor
Receives sensor data via UDP and displays real-time plots.
Also records data to CSV file.

Usage:
    pip install pyqtgraph numpy PyQt5
    python vo2max_monitor.py

Connect to VO2MAX WiFi AP, then run this script.
"""

import sys
import struct
import time
import csv
from datetime import datetime
from collections import deque

import numpy as np

try:
    from PyQt5 import QtWidgets, QtCore, QtGui
    import pyqtgraph as pg
except ImportError:
    print("Required packages: pip install pyqtgraph numpy PyQt5")
    sys.exit(1)

import socket

# UDP settings
UDP_PORT = 8008
BIND_IP = "0.0.0.0"

# Packet format (matches wifiUdpPacket_t)
PACKET_FORMAT = "<BBHfffffffffff"
PACKET_SIZE = struct.calcsize(PACKET_FORMAT)
PACKET_FIELDS = [
    "status", "dummy", "errors",
    "flow", "o2", "ve", "vo2", "vco2",
    "resp_rate", "pressure", "temperature",
    "exhale_temperature", "hr", "rr"
]

# Plot history length
HISTORY_LEN = 600  # 10 minutes at ~1Hz

class VO2Monitor(QtWidgets.QMainWindow):
    def __init__(self):
        super().__init__()
        self.setWindowTitle("VO2Max-TEEP Monitor")
        self.resize(1200, 800)

        # Data buffers
        self.data = {f: deque(maxlen=HISTORY_LEN) for f in PACKET_FIELDS}
        self.time_data = deque(maxlen=HISTORY_LEN)
        self.start_time = time.time()

        # CSV file
        ts = datetime.now().strftime("%Y%m%d_%H%M%S")
        self.csv_file = open(f"vo2max_{ts}.csv", "w", newline="")
        self.csv_writer = csv.writer(self.csv_file)
        self.csv_writer.writerow(["timestamp"] + PACKET_FIELDS)

        # UDP socket
        self.sock = socket.socket(socket.AF_INET, socket.SOCK_DGRAM)
        self.sock.setsockopt(socket.SOL_SOCKET, socket.SO_REUSEADDR, 1)
        self.sock.bind((BIND_IP, UDP_PORT))
        self.sock.setblocking(False)

        self.setup_ui()
        self.setup_timer()

    def setup_ui(self):
        central = QtWidgets.QWidget()
        self.setCentralWidget(central)
        layout = QtWidgets.QVBoxLayout(central)

        # Status bar
        self.status_label = QtWidgets.QLabel("Waiting for data...")
        self.status_label.setStyleSheet("font-size: 14px; color: #888;")
        layout.addWidget(self.status_label)

        # Plots
        pg.setConfigOptions(antialias=True)
        self.plot_widget = pg.GraphicsLayoutWidget()
        layout.addWidget(self.plot_widget)

        # Row 1: VO2 + HR
        self.vo2_plot = self.plot_widget.addPlot(title="VO2 (ml/min/kg)")
        self.vo2_curve = self.vo2_plot.plot(pen=pg.mkPen("c", width=2))
        self.vo2_plot.setYRange(0, 80)

        self.hr_plot = self.plot_widget.addPlot(title="Heart Rate (bpm)")
        self.hr_curve = self.hr_plot.plot(pen=pg.mkPen("r", width=2))
        self.hr_plot.setYRange(40, 200)

        self.plot_widget.nextRow()

        # Row 2: VE + Flow
        self.ve_plot = self.plot_widget.addPlot(title="VE (L/min)")
        self.ve_curve = self.ve_plot.plot(pen=pg.mkPen("g", width=2))
        self.ve_plot.setYRange(0, 200)

        self.flow_plot = self.plot_widget.addPlot(title="Flow Pressure (Pa)")
        self.flow_curve = self.flow_plot.plot(pen=pg.mkPen("y", width=2))
        self.flow_plot.setYRange(0, 30)

        self.plot_widget.nextRow()

        # Row 3: O2% + Resp Rate
        self.o2_plot = self.plot_widget.addPlot(title="O2 (%)")
        self.o2_curve = self.o2_plot.plot(pen=pg.mkPen("w", width=2))
        self.o2_plot.setYRange(15, 22)

        self.rr_plot = self.plot_widget.addPlot(title="Resp Rate (b/min)")
        self.rr_curve = self.rr_plot.plot(pen=pg.mkPen("m", width=2))
        self.rr_plot.setYRange(0, 60)

        # Value labels
        label_layout = QtWidgets.QHBoxLayout()
        self.labels = {}
        for name, color in [("VO2", "#00ffff"), ("HR", "#ff4444"),
                            ("VE", "#44ff44"), ("O2", "#ffffff"),
                            ("RR", "#ff44ff"), ("Cal", "#ffff44")]:
            lbl = QtWidgets.QLabel(f"{name}: --")
            lbl.setStyleSheet(f"font-size: 20px; font-weight: bold; color: {color};")
            label_layout.addWidget(lbl)
            self.labels[name] = lbl
        layout.addLayout(label_layout)

    def setup_timer(self):
        self.timer = QtCore.QTimer()
        self.timer.timeout.connect(self.update)
        self.timer.start(50)  # 20 Hz

    def update(self):
        received = False
        while True:
            try:
                data, addr = self.sock.recvfrom(1024)
                if len(data) >= PACKET_SIZE:
                    values = struct.unpack(PACKET_FORMAT, data[:PACKET_SIZE])
                    t = time.time() - self.start_time

                    self.time_data.append(t)
                    for i, field in enumerate(PACKET_FIELDS):
                        self.data[field].append(values[i])

                    # Write CSV
                    self.csv_writer.writerow([f"{t:.2f}"] + list(values))
                    received = True
            except BlockingIOError:
                break
            except Exception as e:
                print(f"Error: {e}")
                break

        if received and len(self.time_data) > 1:
            t = np.array(self.time_data)
            self.vo2_curve.setData(t, np.array(self.data["vo2"]))
            self.hr_curve.setData(t, np.array(self.data["hr"]))
            self.ve_curve.setData(t, np.array(self.data["ve"]))
            self.flow_curve.setData(t, np.array(self.data["flow"]))
            self.o2_curve.setData(t, np.array(self.data["o2"]))
            self.rr_curve.setData(t, np.array(self.data["resp_rate"]))

            # Update labels
            self.labels["VO2"].setText(f"VO2: {self.data['vo2'][-1]:.1f}")
            self.labels["HR"].setText(f"HR: {self.data['hr'][-1]:.0f}")
            self.labels["VE"].setText(f"VE: {self.data['ve'][-1]:.1f}")
            self.labels["O2"].setText(f"O2: {self.data['o2'][-1]:.2f}%")
            self.labels["RR"].setText(f"RR: {self.data['resp_rate'][-1]:.1f}")

            elapsed = self.time_data[-1]
            mins = int(elapsed // 60)
            secs = int(elapsed % 60)
            self.status_label.setText(
                f"Connected | Packets: {len(self.time_data)} | "
                f"Time: {mins:02d}:{secs:02d} | "
                f"Errors: {int(self.data['errors'][-1])}"
            )

    def closeEvent(self, event):
        self.csv_file.close()
        self.sock.close()
        event.accept()


def main():
    app = QtWidgets.QApplication(sys.argv)
    app.setStyle("Fusion")

    # Dark theme
    palette = QtGui.QPalette()
    palette.setColor(QtGui.QPalette.Window, QtGui.QColor(30, 30, 30))
    palette.setColor(QtGui.QPalette.WindowText, QtGui.QColor(200, 200, 200))
    palette.setColor(QtGui.QPalette.Base, QtGui.QColor(20, 20, 20))
    palette.setColor(QtGui.QPalette.Text, QtGui.QColor(200, 200, 200))
    app.setPalette(palette)

    window = VO2Monitor()
    window.show()
    sys.exit(app.exec_())


if __name__ == "__main__":
    main()
