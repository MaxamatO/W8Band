"""
Simple W8Band UI

START launches w8band_mac.py, sends "1" to trigger 0x01 over BLE,
then waits until the firmware detects motion, records, transfers EOF,
and w8band_mac.py writes w8band_data2.csv. Only then ekf.py runs
to generate w8band_trajectory_ekf.png.
"""

from __future__ import annotations

import os
import queue
import subprocess
import sys
import threading
import time
import tkinter as tk
from pathlib import Path
from tkinter import ttk

BASE_DIR = Path(__file__).resolve().parent
BLE_SCRIPT = BASE_DIR / "w8band_mac.py"
EKF_SCRIPT = BASE_DIR / "ekf.py"
CSV_FILE = BASE_DIR / "w8band_data2.csv"
TRAJECTORY_IMAGE = BASE_DIR / "w8band_trajectory_ekf.png"

POLL_MS = 100
QUIET_SECONDS = 1.0


class W8BandSimpleUI(tk.Tk):
    def __init__(self) -> None:
        super().__init__()
        self.title("W8Band capture + EKF")
        self.geometry("760x520")
        self.minsize(640, 420)

        self._messages: queue.Queue[tuple[str, str]] = queue.Queue()
        self._worker: threading.Thread | None = None
        self._running = False
        self._trajectory_photo: tk.PhotoImage | None = None

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
        log_frame.rowconfigure(1, weight=1)

        preview_frame = ttk.LabelFrame(log_frame, text="Przeanalizowana trajektoria")
        preview_frame.grid(row=0, column=0, columnspan=2, sticky="ew", pady=(0, 10))
        preview_frame.columnconfigure(0, weight=1)

        self.image_label = ttk.Label(
            preview_frame,
            text="Trajektoria pojawi sie tutaj po zakonczeniu EKF.",
            anchor="center",
        )
        self.image_label.grid(row=0, column=0, sticky="ew", padx=10, pady=10)

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

            self._post("status", "Running EKF trajectory calculation...")
            self._run_ekf()

            if TRAJECTORY_IMAGE.exists():
                self._post("image", str(TRAJECTORY_IMAGE))
            self._post("status", "Gotowe. CSV kompletny, EKF policzyl trajektorie.")
            self._post("log", f"\nFinished.\nCSV: {CSV_FILE}\nTrajectory image: {TRAJECTORY_IMAGE}\n")
        except Exception as exc:
            self._post("status", f"Error: {exc}")
            self._post("log", f"\nERROR: {exc}\n")
        finally:
            self._post("done", "")

    def _run_ble_capture(self) -> None:
        process = subprocess.Popen(
            [sys.executable, str(BLE_SCRIPT)],
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

    def _run_ekf(self) -> None:
        env = os.environ.copy()
        env.setdefault("MPLBACKEND", "Agg")

        process = subprocess.Popen(
            [sys.executable, str(EKF_SCRIPT)],
            cwd=str(BASE_DIR),
            stdout=subprocess.PIPE,
            stderr=subprocess.STDOUT,
            text=True,
            bufsize=1,
            env=env,
        )

        if process.stdout is None:
            raise RuntimeError("Could not open EKF process output.")

        for line in process.stdout:
            self._post("log", line)

        rc = process.wait()
        if rc != 0:
            raise RuntimeError(f"EKF failed with exit code {rc}.")

    def _require_file(self, path: Path) -> None:
        if not path.exists():
            raise FileNotFoundError(path)

    def _post(self, kind: str, message: str) -> None:
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
            elif kind == "image":
                self._show_trajectory_image(Path(message))
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
        self._trajectory_photo = None
        self.image_label.configure(
            image="",
            text="Trajektoria pojawi sie tutaj po zakonczeniu EKF.",
        )

    def _show_trajectory_image(self, image_path: Path) -> None:
        try:
            photo = tk.PhotoImage(file=str(image_path))
        except tk.TclError as exc:
            self._append_log(f"\nNie moge wyswietlic trajektorii w UI: {exc}\n")
            return

        max_width = max(1, self.image_label.winfo_width() - 24)
        max_height = 260
        factor = max(1, int(max(photo.width() / max_width, photo.height() / max_height)))
        if factor > 1:
            photo = photo.subsample(factor, factor)

        self._trajectory_photo = photo
        self.image_label.configure(image=photo, text="")


if __name__ == "__main__":
    app = W8BandSimpleUI()
    app.mainloop()
