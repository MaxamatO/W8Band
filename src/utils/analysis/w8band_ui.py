"""
Simple W8Band UI

START launches w8band_mac.py, sends "1" to trigger 0x01 over BLE,
then waits until the firmware detects motion, records, transfers EOF,
and w8band_mac.py writes w8band_data2.csv. Only then ekf.py analyzes
the CSV and the trajectory is drawn directly in the UI.
"""

from __future__ import annotations

import contextlib
import io
import os
import queue
import subprocess
import sys
import threading
import time
import tkinter as tk
from typing import Any
from pathlib import Path
from tkinter import ttk

BASE_DIR = Path(__file__).resolve().parent
BLE_SCRIPT = BASE_DIR / "w8band_mac.py"
EKF_SCRIPT = BASE_DIR / "ekf.py"
CSV_FILE = BASE_DIR / "w8band_data2.csv"
TRAJECTORY_IMAGE = BASE_DIR / "w8band_trajectory_ekf.png"
VENV_PYTHON = BASE_DIR.parent / "venv" / "bin" / "python"
PYTHON = str(VENV_PYTHON if VENV_PYTHON.exists() else Path(sys.executable))

if VENV_PYTHON.exists() and Path(sys.executable).resolve() != VENV_PYTHON.resolve():
    os.execv(str(VENV_PYTHON), [str(VENV_PYTHON), *sys.argv])

os.environ.setdefault("MPLCONFIGDIR", "/tmp/w8band_matplotlib")

from matplotlib.backends.backend_tkagg import FigureCanvasTkAgg
from matplotlib.figure import Figure

POLL_MS = 100
QUIET_SECONDS = 1.0


class W8BandSimpleUI(tk.Tk):
    def __init__(self) -> None:
        super().__init__()
        self.title("W8Band capture + EKF")
        self.geometry("760x520")
        self.minsize(640, 420)

        self._messages: queue.Queue[tuple[str, Any]] = queue.Queue()
        self._worker: threading.Thread | None = None
        self._running = False
        self._plot_canvas: FigureCanvasTkAgg | None = None
        self._plot_figure: Figure | None = None

        self._build_ui()
        self.after(POLL_MS, self._drain_messages)

    def _build_ui(self) -> None:
        self.columnconfigure(0, weight=1)
        self.rowconfigure(3, weight=1)

        header = ttk.Frame(self, padding=(18, 16, 18, 8))
        header.grid(row=0, column=0, sticky="ew")
        header.columnconfigure(0, weight=1)

        title = ttk.Label(header, text="W8Band", font=("TkDefaultFont", 22, "bold"))
        title.grid(row=0, column=0, sticky="w")

        self.start_button = ttk.Button(header, text="START", command=self._start_capture)
        self.start_button.grid(row=0, column=1, sticky="e")

        self.status_var = tk.StringVar(value="Gotowe. START wysyla 0x01, potem firmware czeka na ruch, nagrywa i przesyla dane.")
        status = ttk.Label(self, textvariable=self.status_var, padding=(18, 0, 18, 8))
        status.grid(row=1, column=0, sticky="ew")

        self.progress = ttk.Progressbar(self, mode="indeterminate")
        self.progress.grid(row=2, column=0, sticky="ew", padx=18, pady=(0, 10))

        log_frame = ttk.Frame(self, padding=(18, 0, 18, 18))
        log_frame.grid(row=3, column=0, sticky="nsew")
        log_frame.columnconfigure(0, weight=1)
        log_frame.rowconfigure(0, weight=3)
        log_frame.rowconfigure(1, weight=1)

        preview_frame = ttk.LabelFrame(log_frame, text="Przeanalizowana trajektoria")
        preview_frame.grid(row=0, column=0, columnspan=2, sticky="nsew", pady=(0, 10))
        preview_frame.columnconfigure(0, weight=1)
        preview_frame.rowconfigure(0, weight=1)

        self._plot_figure = Figure(figsize=(8, 4.8), dpi=100, facecolor="#0A0C10")
        self._plot_canvas = FigureCanvasTkAgg(self._plot_figure, master=preview_frame)
        self._plot_canvas.get_tk_widget().grid(row=0, column=0, sticky="nsew", padx=8, pady=8)
        self._draw_empty_plot()

        self.log = tk.Text(log_frame, wrap="word", height=12, state="disabled")
        self.log.grid(row=1, column=0, sticky="nsew")

        scroll = ttk.Scrollbar(log_frame, orient="vertical", command=self.log.yview)
        scroll.grid(row=1, column=1, sticky="ns")
        self.log.configure(yscrollcommand=scroll.set)

        footer = ttk.Frame(self, padding=(18, 0, 18, 18))
        footer.grid(row=4, column=0, sticky="ew")
        footer.columnconfigure(0, weight=1)

        self.csv_var = tk.StringVar(value=f"CSV: {CSV_FILE}")
        self.image_var = tk.StringVar(value=f"Trajectory: {TRAJECTORY_IMAGE}")
        ttk.Label(footer, textvariable=self.csv_var).grid(row=0, column=0, sticky="w")
        ttk.Label(footer, textvariable=self.image_var).grid(row=1, column=0, sticky="w")

    def _start_capture(self) -> None:
        if self._running:
            return

        self._running = True
        self.start_button.configure(state="disabled")
        self.progress.start(10)
        self._set_status("Startuje BLE... po polaczeniu wysle 0x01.")
        self._clear_log()

        self._worker = threading.Thread(target=self._capture_then_run_ekf, daemon=True)
        self._worker.start()

    def _capture_then_run_ekf(self) -> None:
        try:
            self._require_file(BLE_SCRIPT)
            self._require_file(EKF_SCRIPT)

            start_mtime = CSV_FILE.stat().st_mtime if CSV_FILE.exists() else 0.0

            self._post("status", "Lacze z W8Band i wysylam START (0x01). Porusz urzadzeniem, firmware samo nagra okno 8 s.")
            self._run_ble_capture()

            self._post("status", "Transfer zakonczony. Czekam az CSV bedzie kompletny...")
            self._wait_for_csv(start_mtime)

            self._post("status", "Licze trajektorie...")
            result = self._run_analysis()

            self._post("analysis", result)
            self._post("status", "Gotowe. CSV kompletny, EKF policzyl trajektorie.")
            self._post("log", f"\nFinished.\nCSV: {CSV_FILE}\nTrajectory image: {TRAJECTORY_IMAGE}\n")
        except Exception as exc:
            self._post("status", f"Error: {exc}")
            self._post("log", f"\nERROR: {exc}\n")
        finally:
            self._post("done", "")

    def _run_ble_capture(self) -> None:
        process = subprocess.Popen(
            [PYTHON, str(BLE_SCRIPT)],
            cwd=str(BASE_DIR),
            stdin=subprocess.PIPE,
            stdout=subprocess.PIPE,
            stderr=subprocess.STDOUT,
            text=True,
            bufsize=1,
        )

        if process.stdin is None or process.stdout is None:
            raise RuntimeError("Could not open BLE process pipes.")

        # w8band_mac.py waits for "1" before it writes 0x01 to CHAR_CTRL_UUID.
        process.stdin.write("1\n")
        process.stdin.flush()
        process.stdin.close()

        for line in process.stdout:
            self._post("log", line)

        rc = process.wait()
        if rc != 0:
            raise RuntimeError(f"BLE capture failed with exit code {rc}.")

    def _wait_for_csv(self, start_mtime: float) -> None:
        deadline = time.monotonic() + 20.0
        last_size = -1
        stable_since: float | None = None

        while time.monotonic() < deadline:
            if CSV_FILE.exists():
                stat = CSV_FILE.stat()
                if stat.st_mtime > start_mtime and stat.st_size > 0:
                    if stat.st_size == last_size:
                        if stable_since is None:
                            stable_since = time.monotonic()
                        if time.monotonic() - stable_since >= QUIET_SECONDS:
                            return
                    else:
                        last_size = stat.st_size
                        stable_since = None
            time.sleep(0.2)

        raise TimeoutError(f"CSV did not finish populating: {CSV_FILE}")

    def _run_analysis(self) -> Any:
        import ekf

        output = io.StringIO()
        with contextlib.redirect_stdout(output):
            result = ekf.analyze(CSV_FILE)
            ekf.plot_analysis(result, TRAJECTORY_IMAGE)
            ekf.print_metrics(result)

        self._post("log", output.getvalue())
        return result

    def _require_file(self, path: Path) -> None:
        if not path.exists():
            raise FileNotFoundError(path)

    def _post(self, kind: str, message: Any) -> None:
        self._messages.put((kind, message))

    def _drain_messages(self) -> None:
        while True:
            try:
                kind, message = self._messages.get_nowait()
            except queue.Empty:
                break

            if kind == "status":
                self._set_status(message)
            elif kind == "log":
                self._append_log(message)
            elif kind == "analysis":
                self._draw_analysis_result(message)
            elif kind == "done":
                self._running = False
                self.progress.stop()
                self.start_button.configure(state="normal")

        self.after(POLL_MS, self._drain_messages)

    def _set_status(self, message: str) -> None:
        self.status_var.set(message)

    def _append_log(self, message: str) -> None:
        self.log.configure(state="normal")
        self.log.insert("end", message)
        self.log.see("end")
        self.log.configure(state="disabled")

    def _clear_log(self) -> None:
        self.log.configure(state="normal")
        self.log.delete("1.0", "end")
        self.log.configure(state="disabled")
        self._draw_empty_plot()

    def _draw_empty_plot(self) -> None:
        if self._plot_figure is None or self._plot_canvas is None:
            return

        fig = self._plot_figure
        fig.clear()
        ax = fig.add_subplot(111)
        ax.set_facecolor("#0A0C10")
        fig.patch.set_facecolor("#0A0C10")
        ax.text(
            0.5,
            0.5,
            "Trajektoria pojawi sie tutaj po analizie",
            ha="center",
            va="center",
            color="#8892a0",
            transform=ax.transAxes,
        )
        ax.set_xticks([])
        ax.set_yticks([])
        for spine in ax.spines.values():
            spine.set_color("#1e2430")
        self._plot_canvas.draw_idle()

    def _draw_analysis_result(self, result: Any) -> None:
        if self._plot_figure is None or self._plot_canvas is None:
            return

        import numpy as np
        import ekf

        fig = self._plot_figure
        fig.clear()
        fig.patch.set_facecolor(ekf.BG)

        pos_cm = result.position * 100.0
        vel_cm_s = result.velocity * 100.0
        speed = np.linalg.norm(vel_cm_s, axis=1)
        vert = result.vertical_axis
        horiz = ekf.HORIZ if ekf.HORIZ != vert else (1 if vert != 1 else 0)
        labels = ["X", "Y", "Z"]

        ax_traj = fig.add_subplot(121)
        ax_speed = fig.add_subplot(122)

        for ax in (ax_traj, ax_speed):
            ax.set_facecolor(ekf.BG)
            ax.tick_params(colors="#8892a0", labelsize=8)
            for spine in ax.spines.values():
                spine.set_color("#1e2430")
            ax.grid(True, linewidth=0.35, color="#1e2430", alpha=0.8)

        ax_traj.set_title("Trajektoria", color="#e2e8f0", fontsize=11)
        ax_traj.set_xlabel(f"Poziomo {labels[horiz]} (cm)", color="#8892a0", fontsize=9)
        ax_traj.set_ylabel(f"Pionowo {labels[vert]} (cm)", color="#8892a0", fontsize=9)

        for k in range(1, len(pos_cm)):
            color = ekf.PHASE_COLORS[int(result.phase[k])]
            ax_traj.plot(
                [pos_cm[k - 1, horiz], pos_cm[k, horiz]],
                [pos_cm[k - 1, vert], pos_cm[k, vert]],
                color=color,
                lw=2.0,
                solid_capstyle="round",
            )

        ax_traj.scatter(
            pos_cm[0, horiz],
            pos_cm[0, vert],
            color=ekf.COL_START,
            s=70,
            zorder=10,
            edgecolors="white",
            linewidths=0.7,
        )
        ax_traj.scatter(
            pos_cm[-1, horiz],
            pos_cm[-1, vert],
            color=ekf.COL_END,
            s=70,
            zorder=10,
            edgecolors="white",
            linewidths=0.7,
        )
        for idx in result.references[1:-1]:
            ax_traj.scatter(
                pos_cm[idx, horiz],
                pos_cm[idx, vert],
                color=ekf.COL_TURN,
                s=45,
                zorder=9,
                edgecolors=ekf.BG,
                linewidths=0.8,
            )
        ax_traj.set_aspect("equal", adjustable="datalim")

        ax_speed.set_title("Predkosc", color="#e2e8f0", fontsize=11)
        ax_speed.set_xlabel("Czas (s)", color="#8892a0", fontsize=9)
        ax_speed.set_ylabel("cm/s", color="#8892a0", fontsize=9)
        ax_speed.plot(result.time_s, speed, color="#e2e8f0", lw=1.2)
        for idx in result.references:
            ax_speed.axvline(result.time_s[idx], color=ekf.COL_TURN, lw=0.9, alpha=0.65)

        fig.tight_layout(pad=1.2)
        self._plot_canvas.draw_idle()


if __name__ == "__main__":
    app = W8BandSimpleUI()
    app.mainloop()
