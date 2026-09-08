#!/usr/bin/env python3
"""
VO2Max-TEEP Serial Monitor & Data Logger
=========================================
Reads $FAST and $DIAG lines from ESP32 via Serial,
saves all data to CSV, and displays real-time charts.

Usage:
    python vo2max_monitor_serial.py [COM_PORT] [BAUD_RATE]
    python vo2max_monitor_serial.py COM7
    python vo2max_monitor_serial.py COM7 115200

Charts displayed:
    1. Flow Waveform (ΔP vs time) — real-time breath pattern
    2. O₂ Concentration — FeO₂ vs FiO₂ baseline
    3. VO₂ & Minute Ventilation — key metabolic metrics
    4. Respiratory Rate & Calories

CSV output:
    - vo2max_fast_YYYYMMDD_HHMMSS.csv  (10 Hz flow data)
    - vo2max_diag_YYYYMMDD_HHMMSS.csv  (per-integration-period data)

Copyright (C) 2025 TEEP Project — GPL V3
"""

import sys
import os
import csv
import time
import threading
import signal
from datetime import datetime
from collections import deque

import serial
import serial.tools.list_ports
import matplotlib
matplotlib.use('TkAgg')
import matplotlib.pyplot as plt
import matplotlib.animation as animation
from matplotlib.ticker import MaxNLocator

# ============================================================
#  Configuration
# ============================================================
DEFAULT_PORT = 'COM7'
DEFAULT_BAUD = 115200
FAST_WINDOW_SEC = 30        # Flow waveform: show last 30 seconds
DIAG_WINDOW_POINTS = 120    # DIAG chart: show last 120 points (~30 min at 15s)
PLOT_UPDATE_MS = 200        # Chart refresh interval (ms)

# ============================================================
#  Data containers (thread-safe via deque)
# ============================================================
# $FAST fields: time, dP, O2%, exhale_state
fast_time = deque(maxlen=FAST_WINDOW_SEC * 12)  # ~10Hz * 30s
fast_dp = deque(maxlen=FAST_WINDOW_SEC * 12)
fast_o2 = deque(maxlen=FAST_WINDOW_SEC * 12)
fast_exhale = deque(maxlen=FAST_WINDOW_SEC * 12)

# $DIAG fields
diag_time = deque(maxlen=DIAG_WINDOW_POINTS)
diag_dp = deque(maxlen=DIAG_WINDOW_POINTS)
diag_o2 = deque(maxlen=DIAG_WINDOW_POINTS)
diag_fio2 = deque(maxlen=DIAG_WINDOW_POINTS)
diag_pamb = deque(maxlen=DIAG_WINDOW_POINTS)
diag_tamb = deque(maxlen=DIAG_WINDOW_POINTS)
diag_ve = deque(maxlen=DIAG_WINDOW_POINTS)
diag_vo2_abs = deque(maxlen=DIAG_WINDOW_POINTS)
diag_vo2_rel = deque(maxlen=DIAG_WINDOW_POINTS)
diag_cal = deque(maxlen=DIAG_WINDOW_POINTS)
diag_resp = deque(maxlen=DIAG_WINDOW_POINTS)
diag_zero = deque(maxlen=DIAG_WINDOW_POINTS)
diag_errors = deque(maxlen=DIAG_WINDOW_POINTS)

# Track peak values
vo2_peak = [0.0]
ve_peak = [0.0]

# ============================================================
#  CSV Writers
# ============================================================
class CSVLogger:
    def __init__(self, prefix, headers):
        ts = datetime.now().strftime("%Y%m%d_%H%M%S")
        self.filename = f"{prefix}_{ts}.csv"
        self.file = open(self.filename, 'w', newline='', buffering=1)
        self.writer = csv.writer(self.file)
        self.writer.writerow(headers)
        self.count = 0

    def write(self, row):
        self.writer.writerow(row)
        self.count += 1
        if self.count % 50 == 0:
            self.file.flush()

    def close(self):
        self.file.flush()
        self.file.close()
        print(f"  CSV saved: {self.filename} ({self.count} rows)")


# ============================================================
#  Serial Reader Thread
# ============================================================
running = True

def serial_reader(ser, fast_csv, diag_csv):
    global running
    while running:
        try:
            raw = ser.readline()
            if not raw:
                continue
            line = raw.decode('ascii', errors='ignore').strip()
            if not line:
                continue

            if line.startswith('$FAST,'):
                parse_fast(line, fast_csv)
            elif line.startswith('$DIAG,'):
                parse_diag(line, diag_csv)
            else:
                # Print other lines (boot messages, errors, etc.)
                print(f"[ESP32] {line}")

        except serial.SerialException:
            print("\n⚠ Serial disconnected!")
            running = False
            break
        except Exception as e:
            pass  # Ignore parse errors silently


def parse_fast(line, csv_logger):
    """Parse: $FAST,time,dP,O2%,exhale_state"""
    try:
        parts = line.split(',')
        if len(parts) < 5:
            return
        t = float(parts[1])
        dp = float(parts[2])
        o2 = float(parts[3])
        exh = int(parts[4])

        fast_time.append(t)
        fast_dp.append(dp)
        fast_o2.append(o2)
        fast_exhale.append(exh)

        csv_logger.write([t, dp, o2, exh])
    except (ValueError, IndexError):
        pass


def parse_diag(line, csv_logger):
    """Parse: $DIAG,time,dP,O2%,FiO2%,Pamb,Tamb,VE,VO2abs,VO2rel,Cal,RespRate,Zero,Errors"""
    try:
        parts = line.split(',')
        if len(parts) < 13:
            return
        t = float(parts[1])
        dp = float(parts[2])
        o2 = float(parts[3])
        fio2 = float(parts[4])
        pamb = float(parts[5])
        tamb = float(parts[6])
        ve = float(parts[7])
        vo2_a = float(parts[8])
        vo2_r = float(parts[9])
        cal = float(parts[10])
        resp = float(parts[11])
        zero = float(parts[12])
        errors = int(parts[13]) if len(parts) > 13 else 0

        diag_time.append(t)
        diag_dp.append(dp)
        diag_o2.append(o2)
        diag_fio2.append(fio2)
        diag_pamb.append(pamb)
        diag_tamb.append(tamb)
        diag_ve.append(ve)
        diag_vo2_abs.append(vo2_a)
        diag_vo2_rel.append(vo2_r)
        diag_cal.append(cal)
        diag_resp.append(resp)
        diag_zero.append(zero)
        diag_errors.append(errors)

        # Update peaks
        if vo2_r > vo2_peak[0]:
            vo2_peak[0] = vo2_r
        if ve > ve_peak[0]:
            ve_peak[0] = ve

        csv_logger.write([t, dp, o2, fio2, pamb, tamb, ve, vo2_a, vo2_r, cal, resp, zero, errors])

        # Print summary to console
        print(f"  [{t:7.1f}s]  O2={o2:.1f}%  FiO2={fio2:.1f}%  VE={ve:.1f} L/min  "
              f"VO2={vo2_r:.1f} ml/min/kg  Cal={cal:.2f}  RR={resp:.0f}  Err={errors}")

    except (ValueError, IndexError):
        pass


# ============================================================
#  Chart Setup & Animation
# ============================================================
def setup_charts():
    """Create the 4-panel chart layout."""
    fig, axes = plt.subplots(2, 2, figsize=(14, 8))
    fig.suptitle('VO2Max-TEEP Real-Time Monitor', fontsize=14, fontweight='bold')
    fig.patch.set_facecolor('#1a1a2e')

    colors = {
        'bg': '#1a1a2e',
        'panel': '#16213e',
        'text': '#e8e8e8',
        'grid': '#2a3a5e',
        'flow': '#00d4ff',
        'exhale': '#ff6b6b',
        'o2': '#4ecdc4',
        'fio2': '#ff9f43',
        'vo2': '#ff6b6b',
        've': '#00d4ff',
        'resp': '#a29bfe',
        'cal': '#fd79a8',
    }

    for ax in axes.flat:
        ax.set_facecolor(colors['panel'])
        ax.tick_params(colors=colors['text'], labelsize=9)
        ax.xaxis.label.set_color(colors['text'])
        ax.yaxis.label.set_color(colors['text'])
        ax.title.set_color(colors['text'])
        ax.grid(True, alpha=0.3, color=colors['grid'])
        for spine in ax.spines.values():
            spine.set_color(colors['grid'])

    # Panel 1: Flow Waveform
    ax1 = axes[0, 0]
    ax1.set_title('🌬 Flow Waveform (ΔP)', fontsize=11)
    ax1.set_ylabel('ΔP (Pa)')
    ax1.set_xlabel('Time (s)')
    line_flow, = ax1.plot([], [], color=colors['flow'], linewidth=0.8, label='ΔP')
    ax1.set_ylim(-0.5, 25)
    ax1.legend(loc='upper right', fontsize=8, facecolor=colors['panel'], labelcolor=colors['text'])

    # Panel 2: O2 Concentration
    ax2 = axes[0, 1]
    ax2.set_title('🫁 O₂ Concentration', fontsize=11)
    ax2.set_ylabel('O₂ (%)')
    ax2.set_xlabel('Time (s)')
    line_o2, = ax2.plot([], [], color=colors['o2'], linewidth=1.5, label='FeO₂')
    line_fio2, = ax2.plot([], [], color=colors['fio2'], linewidth=1.5, linestyle='--', label='FiO₂')
    ax2.set_ylim(14, 22)
    ax2.legend(loc='lower left', fontsize=8, facecolor=colors['panel'], labelcolor=colors['text'])

    # Panel 3: VO2 & VE
    ax3 = axes[1, 0]
    ax3.set_title('💪 VO₂ & Minute Ventilation', fontsize=11)
    ax3.set_ylabel('VO₂ (ml/min/kg)', color=colors['vo2'])
    ax3.set_xlabel('Time (s)')
    line_vo2, = ax3.plot([], [], color=colors['vo2'], linewidth=2, label='VO₂', marker='o', markersize=3)
    ax3b = ax3.twinx()
    ax3b.set_ylabel('VE (L/min)', color=colors['ve'])
    ax3b.tick_params(axis='y', colors=colors['ve'], labelsize=9)
    line_ve, = ax3b.plot([], [], color=colors['ve'], linewidth=2, label='VE', marker='s', markersize=3)
    ax3.legend(loc='upper left', fontsize=8, facecolor=colors['panel'], labelcolor=colors['text'])
    ax3b.legend(loc='upper right', fontsize=8, facecolor=colors['panel'], labelcolor=colors['text'])
    # Text annotation for peak
    peak_text = ax3.text(0.5, 0.95, '', transform=ax3.transAxes, fontsize=9,
                         color='#ffd700', ha='center', va='top',
                         bbox=dict(boxstyle='round', facecolor=colors['panel'], alpha=0.8))

    # Panel 4: Respiratory Rate & Calories
    ax4 = axes[1, 1]
    ax4.set_title('📊 Respiratory Rate & Calories', fontsize=11)
    ax4.set_ylabel('Resp Rate (b/min)', color=colors['resp'])
    ax4.set_xlabel('Time (s)')
    line_resp, = ax4.plot([], [], color=colors['resp'], linewidth=2, label='RR', marker='o', markersize=3)
    ax4b = ax4.twinx()
    ax4b.set_ylabel('Cal (kcal/min)', color=colors['cal'])
    ax4b.tick_params(axis='y', colors=colors['cal'], labelsize=9)
    line_cal, = ax4b.plot([], [], color=colors['cal'], linewidth=2, label='Cal', marker='s', markersize=3)
    ax4.legend(loc='upper left', fontsize=8, facecolor=colors['panel'], labelcolor=colors['text'])
    ax4b.legend(loc='upper right', fontsize=8, facecolor=colors['panel'], labelcolor=colors['text'])

    fig.tight_layout(rect=[0, 0.02, 1, 0.96])

    return fig, {
        'ax1': ax1, 'ax2': ax2, 'ax3': ax3, 'ax3b': ax3b, 'ax4': ax4, 'ax4b': ax4b,
        'line_flow': line_flow,
        'line_o2': line_o2, 'line_fio2': line_fio2,
        'line_vo2': line_vo2, 'line_ve': line_ve,
        'line_resp': line_resp, 'line_cal': line_cal,
        'peak_text': peak_text,
        'colors': colors,
    }


def update_charts(frame, components):
    """Animation callback — update all charts."""
    c = components

    # ---- Panel 1: Flow Waveform ----
    if len(fast_time) > 2:
        t_list = list(fast_time)
        dp_list = list(fast_dp)
        c['line_flow'].set_data(t_list, dp_list)
        c['ax1'].set_xlim(t_list[-1] - FAST_WINDOW_SEC, t_list[-1] + 1)
        max_dp = max(dp_list[-300:]) if len(dp_list) > 10 else 5
        c['ax1'].set_ylim(-0.5, max(max_dp * 1.3, 3))

    # ---- Panel 2: O2 Concentration (from $DIAG) ----
    if len(diag_time) > 1:
        dt = list(diag_time)
        c['line_o2'].set_data(dt, list(diag_o2))
        c['line_fio2'].set_data(dt, list(diag_fio2))
        c['ax2'].set_xlim(dt[0] - 5, dt[-1] + 5)
        all_o2 = list(diag_o2) + list(diag_fio2)
        c['ax2'].set_ylim(min(all_o2) - 1, max(all_o2) + 1)

    # ---- Panel 3: VO2 & VE ----
    if len(diag_time) > 1:
        dt = list(diag_time)
        vo2_list = list(diag_vo2_rel)
        ve_list = list(diag_ve)
        c['line_vo2'].set_data(dt, vo2_list)
        c['line_ve'].set_data(dt, ve_list)
        c['ax3'].set_xlim(dt[0] - 5, dt[-1] + 5)
        c['ax3'].set_ylim(0, max(max(vo2_list) * 1.2, 5))
        c['ax3b'].set_ylim(0, max(max(ve_list) * 1.2, 5))
        c['peak_text'].set_text(f'VO₂peak: {vo2_peak[0]:.1f} ml/min/kg  |  VEpeak: {ve_peak[0]:.1f} L/min')

    # ---- Panel 4: Resp Rate & Calories ----
    if len(diag_time) > 1:
        dt = list(diag_time)
        resp_list = list(diag_resp)
        cal_list = list(diag_cal)
        c['line_resp'].set_data(dt, resp_list)
        c['line_cal'].set_data(dt, cal_list)
        c['ax4'].set_xlim(dt[0] - 5, dt[-1] + 5)
        c['ax4'].set_ylim(0, max(max(resp_list) * 1.2, 20))
        c['ax4b'].set_ylim(0, max(max(cal_list) * 1.2, 1))

    return [c['line_flow'], c['line_o2'], c['line_fio2'],
            c['line_vo2'], c['line_ve'], c['line_resp'], c['line_cal']]


# ============================================================
#  Port Selection
# ============================================================
def select_port():
    """List available COM ports and let user select."""
    ports = serial.tools.list_ports.comports()
    if not ports:
        print("❌ No COM ports found!")
        sys.exit(1)

    print("\n╔══════════════════════════════════════════╗")
    print("║   VO2Max-TEEP Serial Monitor v2.0       ║")
    print("╚══════════════════════════════════════════╝\n")
    print("Available COM ports:")
    for i, p in enumerate(ports):
        print(f"  [{i+1}] {p.device}  —  {p.description}")

    if len(ports) == 1:
        print(f"\n→ Auto-selecting: {ports[0].device}")
        return ports[0].device

    while True:
        choice = input(f"\nSelect port [1-{len(ports)}] or type COM name: ").strip()
        if choice.upper().startswith('COM'):
            return choice.upper()
        try:
            idx = int(choice) - 1
            if 0 <= idx < len(ports):
                return ports[idx].device
        except ValueError:
            pass
        print("Invalid selection, try again.")


# ============================================================
#  Main
# ============================================================
def main():
    global running

    # Parse args
    port = sys.argv[1] if len(sys.argv) > 1 else None
    baud = int(sys.argv[2]) if len(sys.argv) > 2 else DEFAULT_BAUD

    if not port:
        port = select_port()

    print(f"\n🔌 Connecting to {port} at {baud} baud...")

    try:
        ser = serial.Serial(port, baud, timeout=1)
    except serial.SerialException as e:
        print(f"❌ Cannot open {port}: {e}")
        sys.exit(1)

    print(f"✅ Connected to {port}")

    # Create CSV loggers
    fast_csv = CSVLogger('vo2max_fast', ['time_s', 'dP_Pa', 'O2_pct', 'exhale_state'])
    diag_csv = CSVLogger('vo2max_diag', [
        'time_s', 'dP_Pa', 'FeO2_pct', 'FiO2_pct', 'P_amb_hPa', 'T_amb_C',
        'VE_Lmin', 'VO2_abs_Lmin', 'VO2_rel_mlminkkg', 'Cal_kcalmin',
        'RespRate_bpm', 'ZeroOffset_Pa', 'ErrorCount'
    ])
    print(f"📁 Logging to: {fast_csv.filename}, {diag_csv.filename}")
    print(f"📊 Charts updating every {PLOT_UPDATE_MS}ms")
    print(f"   Press Ctrl+C or close chart window to stop.\n")
    print("─" * 80)
    print(f"  {'Time':>8s}  {'O2':>5s}  {'FiO2':>5s}  {'VE':>6s}  {'VO2':>10s}  {'Cal':>5s}  {'RR':>4s}  {'Err':>4s}")
    print("─" * 80)

    # Start serial reader thread
    reader_thread = threading.Thread(target=serial_reader, args=(ser, fast_csv, diag_csv), daemon=True)
    reader_thread.start()

    # Setup and run charts
    fig, components = setup_charts()

    def on_close(event):
        global running
        running = False

    fig.canvas.mpl_connect('close_event', on_close)

    ani = animation.FuncAnimation(
        fig, update_charts, fargs=(components,),
        interval=PLOT_UPDATE_MS, blit=False, cache_frame_data=False
    )

    try:
        plt.show()
    except KeyboardInterrupt:
        pass
    finally:
        running = False
        print("\n\n🛑 Stopping...")
        time.sleep(0.5)
        ser.close()
        fast_csv.close()
        diag_csv.close()
        print("✅ Done!")


if __name__ == '__main__':
    # Handle Ctrl+C gracefully
    signal.signal(signal.SIGINT, lambda s, f: setattr(sys.modules[__name__], 'running', False))
    main()
