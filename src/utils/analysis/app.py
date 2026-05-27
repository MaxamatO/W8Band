"""
W8Band Desktop App for macOS
─────────────────────────────
Łączy się przez BLE z W8Band, wysyła START, odbiera próbki IMU,
przelicza tor ruchu 2D + metryki VBT i wyświetla wyniki w GUI.

Wymagania:
    pip install bleak matplotlib numpy scipy

Uruchomienie:
    python w8band_app.py
"""

import asyncio
import struct
import threading
import time
import tkinter as tk
from tkinter import ttk, font as tkfont
import numpy as np
from scipy.signal import butter, sosfilt
from scipy.spatial.transform import Rotation as R, Slerp
from scipy.interpolate import interp1d
import matplotlib
matplotlib.use("TkAgg")
import matplotlib.pyplot as plt
from matplotlib.backends.backend_tkagg import FigureCanvasTkAgg
from matplotlib.figure import Figure
import matplotlib.patches as mpatches
from bleak import BleakClient, BleakScanner

# ── BLE UUIDs & protokół ─────────────────────────────────────────────────────
DEVICE_NAME    = "W8Ban"
CHAR_CTRL_UUID = "19B10001-E8F2-537E-4F6C-D104768A1214"
CHAR_DATA_UUID = "19B10002-E8F2-537E-4F6C-D104768A1214"

FRAME_FMT  = '<hhhhhhhHI'
FRAME_SIZE = struct.calcsize(FRAME_FMT)   # 20 B
assert FRAME_SIZE == 20

# ── Parametry przetwarzania IMU ──────────────────────────────────────────────
IMU_HZ        = 120.0
DT            = 1.0 / IMU_HZ
STILL_START_S = 0.5
STILL_END_S   = 2.0
PRIMARY_AXIS  = 2          # Z = pionowa
MIN_REP_S     = 0.8
LP_HZ         = 15.0

# ── Paleta kolorów ───────────────────────────────────────────────────────────
BG       = "#0A0C10"
BG2      = "#12151C"
BG3      = "#1A1E28"
ACCENT   = "#00E5FF"
ACCENT2  = "#FF3D71"
GREEN    = "#00FF9D"
YELLOW   = "#FFD60A"
TEXT     = "#E8EAF0"
TEXT_DIM = "#5A6070"
BORDER   = "#252A36"

REP_PALETTE = ["#00E5FF","#FF3D71","#FFD60A","#00FF9D",
               "#B97AFF","#FF8C42","#40C9FF","#FF6B9D"]


# ═══════════════════════════════════════════════════════════════════════════════
#  DSP – przetwarzanie IMU
# ═══════════════════════════════════════════════════════════════════════════════

def process_imu(frames: list) -> dict:
    """
    Wejście: lista krotek (qw,qx,qy,qz, ax,ay,az [mg], seq, ts_ms)
    Wyjście: słownik z pozycjami, prędkościami, repami i metrykami.
    """
    arr    = np.array(frames, dtype=float)
    quats  = arr[:, 0:4]
    accel  = arr[:, 4:7] / 1000.0 * 9.81   # mg → m/s²
    ts_ms  = arr[:, 8]

    # normalizacja kwaternionów
    norms = np.linalg.norm(quats, axis=1, keepdims=True)
    mask  = norms[:, 0] > 0.5
    quats, accel, ts_ms = quats[mask], accel[mask], ts_ms[mask]
    quats = quats / np.linalg.norm(quats, axis=1, keepdims=True)

    if len(quats) < 50:
        return {}

    # resample do równomiernej siatki
    t_uniform = np.arange(ts_ms[0], ts_ms[-1], DT * 1000.0)
    N = len(t_uniform)

    accel_rs = np.column_stack([
        interp1d(ts_ms, accel[:, i], kind="linear",
                 fill_value="extrapolate")(t_uniform)
        for i in range(3)
    ])

    rots_raw = R.from_quat(quats[:, [1,2,3,0]])   # scipy [x,y,z,w]
    slerp    = Slerp(ts_ms, rots_raw)
    rots     = slerp(t_uniform)
    time_s   = (t_uniform - t_uniform[0]) / 1000.0

    # świat + kalibracja grawitacji
    accel_world = rots.apply(accel_rs)
    cal_s = int(STILL_START_S * IMU_HZ)
    cal_e = int(STILL_END_S   * IMU_HZ)
    if cal_e > N:
        cal_e = N // 4
    gravity  = np.mean(accel_world[cal_s:cal_e], axis=0)
    accel_c  = accel_world - gravity

    # lowpass 15 Hz
    sos     = butter(4, LP_HZ / (IMU_HZ / 2), btype="low", output="sos")
    accel_f = np.column_stack([sosfilt(sos, accel_c[:, i]) for i in range(3)])

    # całkowanie trapezy
    vel = np.zeros((N, 3))
    for i in range(1, N):
        vel[i] = vel[i-1] + (accel_f[i] + accel_f[i-1]) * 0.5 * DT

    pos = np.zeros((N, 3))
    for i in range(1, N):
        pos[i] = pos[i-1] + (vel[i] + vel[i-1]) * 0.5 * DT

    pos_cm = pos * 100.0

    # detekcja repów przez zero-crossings prędkości
    v_primary = vel[:, PRIMARY_AXIS]
    sos2      = butter(2, 3.0 / (IMU_HZ / 2), btype="low", output="sos")
    v_smooth  = sosfilt(sos2, v_primary)

    sign      = np.sign(v_smooth)
    crossings = np.where(np.diff(sign) != 0)[0]

    min_rep_samples = int(MIN_REP_S * IMU_HZ)
    filtered = []
    for i, c in enumerate(crossings):
        if i == 0 or c - filtered[-1] > min_rep_samples:
            filtered.append(c)
    crossings = np.array(filtered, dtype=int)

    boundaries = np.concatenate([[0], crossings, [N-1]])

    # korekcja driftu między turnaroundami
    pos_corr = pos_cm.copy()
    for i in range(len(boundaries) - 1):
        s, e = boundaries[i], boundaries[i+1]
        if e <= s:
            continue
        drift  = pos_corr[e] - pos_corr[s]
        t_seg  = np.linspace(0, 1, e - s + 1)
        for ax in range(3):
            pos_corr[s:e+1, ax] -= t_seg * drift[ax]

    # segmentacja repów (pary zero-crossingów)
    rep_segs = []
    for i in range(0, len(crossings) - 1, 2):
        s = crossings[i]
        e = crossings[i+1] if i+1 < len(crossings) else N-1
        if e - s > min_rep_samples:
            rep_segs.append((s, e))

    speed = np.linalg.norm(vel, axis=1) * 100.0  # cm/s

    # metryki per-rep
    reps_metrics = []
    for s, e in rep_segs:
        rom      = float(np.ptp(pos_corr[s:e, PRIMARY_AXIS]))
        peak_v   = float(np.max(speed[s:e]))
        mean_v   = float(np.mean(speed[s:e]))
        duration = float(time_s[e] - time_s[s])
        reps_metrics.append({
            "rom":      round(rom, 1),
            "peak_v":   round(peak_v, 1),
            "mean_v":   round(mean_v, 1),
            "duration": round(duration, 2),
        })

    return {
        "time_s":      time_s,
        "pos_corr":    pos_corr,
        "speed":       speed,
        "vel":         vel,
        "rep_segs":    rep_segs,
        "reps":        reps_metrics,
        "crossings":   crossings,
        "n_samples":   N,
    }


# ═══════════════════════════════════════════════════════════════════════════════
#  BLE worker (asyncio w osobnym wątku)
# ═══════════════════════════════════════════════════════════════════════════════

class BLEWorker:
    def __init__(self, on_status, on_progress, on_done, on_error):
        self.on_status   = on_status
        self.on_progress = on_progress
        self.on_done     = on_done
        self.on_error    = on_error
        self._thread     = None
        self._loop       = None

    def start(self):
        self._thread = threading.Thread(target=self._run, daemon=True)
        self._thread.start()

    def _run(self):
        self._loop = asyncio.new_event_loop()
        asyncio.set_event_loop(self._loop)
        self._loop.run_until_complete(self._ble_main())

    async def _ble_main(self):
        frames             = []
        transfer_complete  = asyncio.Event()
        expected_total     = [0]

        def on_notify(sender, data: bytearray):
            for offset in range(0, len(data) - FRAME_SIZE + 1, FRAME_SIZE):
                chunk = data[offset:offset + FRAME_SIZE]
                qw_r, qx_r, qy_r, qz_r, ax, ay, az, seq, ts = struct.unpack(FRAME_FMT, chunk)

                if seq == 0xFFFF:
                    expected_total[0] = ts
                    transfer_complete.set()
                    return

                qw = qw_r / 16384.0
                qx = qx_r / 16384.0
                qy = qy_r / 16384.0
                qz = qz_r / 16384.0
                frames.append((qw, qx, qy, qz, ax, ay, az, seq, ts))

            if len(frames) % 50 == 0 and len(frames) > 0:
                self.on_progress(len(frames))

        try:
            self.on_status("Szukam W8Band…", ACCENT)
            device = await BleakScanner.find_device_by_name(DEVICE_NAME, timeout=12.0)
            if not device:
                self.on_error("Nie znaleziono W8Band.\nSprawdź czy urządzenie jest włączone.")
                return

            self.on_status("Łączę…", ACCENT)
            async with BleakClient(device, timeout=20.0) as client:
                self.on_status("Połączono! Czekam na dane…", GREEN)

                await client.start_notify(CHAR_DATA_UUID, on_notify)
                await client.write_gatt_char(CHAR_CTRL_UUID, bytearray([0x01]), response=True)

                self.on_status(f"Nagrywanie… (8s)", YELLOW)

                try:
                    await asyncio.wait_for(transfer_complete.wait(), timeout=45.0)
                except asyncio.TimeoutError:
                    self.on_error("Timeout — brak sygnału EOF z urządzenia.")
                    return

                await client.stop_notify(CHAR_DATA_UUID)

            self.on_status("Przetwarzam…", ACCENT)
            result = process_imu(frames)

            if not result:
                self.on_error("Za mało danych do analizy.")
                return

            self.on_done(frames, result, expected_total[0])

        except Exception as e:
            self.on_error(str(e))


# ═══════════════════════════════════════════════════════════════════════════════
#  GUI
# ═══════════════════════════════════════════════════════════════════════════════

class W8BandApp(tk.Tk):
    def __init__(self):
        super().__init__()
        self.title("W8Band")
        self.configure(bg=BG)
        self.geometry("1200x820")
        self.minsize(900, 680)

        self._load_fonts()
        self._build_ui()
        self._frames_raw = []
        self._result     = None

    def _load_fonts(self):
        self.font_title   = tkfont.Font(family="SF Pro Display", size=22, weight="bold")
        self.font_label   = tkfont.Font(family="SF Pro Text",    size=11)
        self.font_metric  = tkfont.Font(family="SF Mono",        size=28, weight="bold")
        self.font_unit    = tkfont.Font(family="SF Mono",        size=11)
        self.font_status  = tkfont.Font(family="SF Mono",        size=12)
        self.font_rep     = tkfont.Font(family="SF Mono",        size=11, weight="bold")

    # ── layout ────────────────────────────────────────────────────────────────
    def _build_ui(self):
        # ── header ──
        hdr = tk.Frame(self, bg=BG, pady=0)
        hdr.pack(fill="x", padx=28, pady=(22, 0))

        tk.Label(hdr, text="W8", bg=BG, fg=ACCENT,
                 font=self.font_title).pack(side="left")
        tk.Label(hdr, text="Band", bg=BG, fg=TEXT,
                 font=self.font_title).pack(side="left", padx=(0,0))

        tk.Label(hdr, text="velocity based training",
                 bg=BG, fg=TEXT_DIM,
                 font=("SF Pro Text", 11)).pack(side="left", padx=(14,0), pady=(6,0))

        self.btn_start = tk.Button(
            hdr, text="▶  START", bg=ACCENT, fg=BG,
            font=("SF Mono", 13, "bold"),
            relief="flat", bd=0, padx=22, pady=8,
            activebackground="#00BBDD", activeforeground=BG,
            cursor="hand2", command=self._on_start
        )
        self.btn_start.pack(side="right")

        # separator
        sep = tk.Frame(self, bg=BORDER, height=1)
        sep.pack(fill="x", padx=28, pady=(14,0))

        # ── status bar ──
        self.lbl_status = tk.Label(
            self, text="Gotowy. Naciśnij START aby nagrać ruch.",
            bg=BG, fg=TEXT_DIM, font=self.font_status,
            anchor="w"
        )
        self.lbl_status.pack(fill="x", padx=28, pady=(8,0))

        # ── progress bar ──
        style = ttk.Style(self)
        style.theme_use("clam")
        style.configure("W8.Horizontal.TProgressbar",
                        troughcolor=BG3, background=ACCENT,
                        bordercolor=BG3, lightcolor=ACCENT, darkcolor=ACCENT,
                        thickness=3)
        self.progress = ttk.Progressbar(
            self, style="W8.Horizontal.TProgressbar",
            orient="horizontal", mode="determinate", maximum=960
        )
        self.progress.pack(fill="x", padx=28, pady=(4,0))

        # ── main area ──
        main = tk.Frame(self, bg=BG)
        main.pack(fill="both", expand=True, padx=28, pady=14)

        # left: plot
        plot_frame = tk.Frame(main, bg=BG2, bd=0,
                               highlightbackground=BORDER,
                               highlightthickness=1)
        plot_frame.pack(side="left", fill="both", expand=True)

        self.fig = Figure(figsize=(7, 5), facecolor=BG2)
        self.canvas = FigureCanvasTkAgg(self.fig, master=plot_frame)
        self.canvas.get_tk_widget().pack(fill="both", expand=True)
        self._draw_empty_plot()

        # right: metrics panel
        right = tk.Frame(main, bg=BG, width=260)
        right.pack(side="right", fill="y", padx=(14,0))
        right.pack_propagate(False)

        # summary cards
        cards = tk.Frame(right, bg=BG)
        cards.pack(fill="x")

        self._card_reps   = self._make_card(cards, "SERIE",    "—",  "reps",  0)
        self._card_avgrom = self._make_card(cards, "ŚR. ROM",  "—",  "cm",    1)
        self._card_avgv   = self._make_card(cards, "ŚR. PRED", "—",  "cm/s",  2)

        sep2 = tk.Frame(right, bg=BORDER, height=1)
        sep2.pack(fill="x", pady=10)

        tk.Label(right, text="PER REP", bg=BG, fg=TEXT_DIM,
                 font=("SF Mono", 10)).pack(anchor="w")

        # scrollable rep list
        list_frame = tk.Frame(right, bg=BG)
        list_frame.pack(fill="both", expand=True, pady=(6,0))

        self.rep_canvas = tk.Canvas(list_frame, bg=BG, bd=0,
                                    highlightthickness=0)
        scrollbar = tk.Scrollbar(list_frame, orient="vertical",
                                 command=self.rep_canvas.yview)
        self.rep_inner = tk.Frame(self.rep_canvas, bg=BG)

        self.rep_canvas.configure(yscrollcommand=scrollbar.set)
        scrollbar.pack(side="right", fill="y")
        self.rep_canvas.pack(side="left", fill="both", expand=True)
        self.rep_canvas.create_window((0,0), window=self.rep_inner, anchor="nw")
        self.rep_inner.bind("<Configure>",
            lambda e: self.rep_canvas.configure(
                scrollregion=self.rep_canvas.bbox("all")))

    def _make_card(self, parent, label, value, unit, col):
        frame = tk.Frame(parent, bg=BG3,
                         highlightbackground=BORDER, highlightthickness=1)
        frame.grid(row=0, column=col, padx=(0 if col==0 else 4, 0),
                   sticky="nsew", ipadx=8, ipady=6)
        parent.columnconfigure(col, weight=1)

        tk.Label(frame, text=label, bg=BG3, fg=TEXT_DIM,
                 font=("SF Mono", 9)).pack(anchor="w", padx=8, pady=(6,0))

        val_frame = tk.Frame(frame, bg=BG3)
        val_frame.pack(anchor="w", padx=8, pady=(0,6))

        val_lbl = tk.Label(val_frame, text=value, bg=BG3, fg=TEXT,
                           font=("SF Mono", 20, "bold"))
        val_lbl.pack(side="left", anchor="s")

        tk.Label(val_frame, text=f" {unit}", bg=BG3, fg=TEXT_DIM,
                 font=("SF Mono", 9)).pack(side="left", anchor="s", pady=(0,3))

        return val_lbl

    # ── empty plot ─────────────────────────────────────────────────────────────
    def _draw_empty_plot(self):
        self.fig.clear()
        ax = self.fig.add_subplot(111)
        ax.set_facecolor(BG2)
        ax.tick_params(colors=TEXT_DIM)
        ax.spines[:].set_color(BORDER)
        ax.set_xlabel("Poziomo  (cm)", color=TEXT_DIM, fontsize=9)
        ax.set_ylabel("Pionowo Z  (cm)", color=TEXT_DIM, fontsize=9)
        ax.set_title("Tor ruchu 2D", color=TEXT_DIM, fontsize=10)
        ax.text(0.5, 0.5, "Brak danych",
                transform=ax.transAxes, ha="center", va="center",
                color=TEXT_DIM, fontsize=14, alpha=0.4)
        self.fig.tight_layout(pad=1.4)
        self.canvas.draw()

    # ── draw results ──────────────────────────────────────────────────────────
    def _draw_results(self, result):
        self.fig.clear()

        pos    = result["pos_corr"]
        segs   = result["rep_segs"]
        time_s = result["time_s"]
        speed  = result["speed"]

        # wybieramy dwie osie do 2D: pozioma = X (indeks 0), pionowa = Z (indeks 2)
        HORIZ = 0
        VERT  = PRIMARY_AXIS  # 2 = Z

        # ── subplot 1: tor 2D ──
        ax1 = self.fig.add_subplot(1, 2, 1)
        ax1.set_facecolor(BG2)
        ax1.tick_params(colors=TEXT_DIM, labelsize=8)
        for spine in ax1.spines.values():
            spine.set_color(BORDER)
        ax1.set_xlabel("X  (cm)", color=TEXT_DIM, fontsize=9)
        ax1.set_ylabel("Z  (cm)", color=TEXT_DIM, fontsize=9)
        ax1.set_title("Tor ruchu 2D", color=TEXT, fontsize=10, pad=10)
        ax1.grid(alpha=0.07, color="#555")

        if segs:
            for idx, (s, e) in enumerate(segs):
                col = REP_PALETTE[idx % len(REP_PALETTE)]
                ax1.plot(pos[s:e, HORIZ], pos[s:e, VERT],
                         color=col, lw=2.0, alpha=0.9,
                         label=f"Rep {idx+1}")
                # start/end dots
                ax1.scatter(pos[s, HORIZ],  pos[s, VERT],
                            color=col, s=60, zorder=5)
                ax1.scatter(pos[e-1, HORIZ], pos[e-1, VERT],
                            color=col, s=30, zorder=5, alpha=0.5)
            ax1.legend(facecolor=BG3, labelcolor=TEXT,
                       edgecolor=BORDER, fontsize=8,
                       loc="best", framealpha=0.85)
        else:
            ax1.plot(pos[:, HORIZ], pos[:, VERT],
                     color=ACCENT, lw=1.5, alpha=0.8)

        ax1.scatter(*[pos[0, HORIZ], pos[0, VERT]],
                    color=GREEN, s=100, zorder=6, label="Start")
        ax1.scatter(*[pos[-1, HORIZ], pos[-1, VERT]],
                    color=ACCENT2, s=100, zorder=6, label="Koniec")

        # ── subplot 2: prędkość w czasie ──
        ax2 = self.fig.add_subplot(1, 2, 2)
        ax2.set_facecolor(BG2)
        ax2.tick_params(colors=TEXT_DIM, labelsize=8)
        for spine in ax2.spines.values():
            spine.set_color(BORDER)
        ax2.set_xlabel("Czas (s)", color=TEXT_DIM, fontsize=9)
        ax2.set_ylabel("Prędkość (cm/s)", color=TEXT_DIM, fontsize=9)
        ax2.set_title("Prędkość w czasie", color=TEXT, fontsize=10, pad=10)
        ax2.grid(alpha=0.07, color="#555")

        ax2.plot(time_s, speed, color=TEXT_DIM, lw=0.8, alpha=0.4)

        for idx, (s, e) in enumerate(segs):
            col = REP_PALETTE[idx % len(REP_PALETTE)]
            ax2.fill_between(time_s[s:e], speed[s:e],
                             alpha=0.25, color=col)
            ax2.plot(time_s[s:e], speed[s:e],
                     color=col, lw=1.8, alpha=0.9)

        self.fig.patch.set_facecolor(BG2)
        self.fig.tight_layout(pad=1.6)
        self.canvas.draw()

    def _update_metrics(self, result):
        reps = result["reps"]
        n    = len(reps)

        self._card_reps["text"] = str(n)

        if n > 0:
            avg_rom = np.mean([r["rom"]    for r in reps])
            avg_v   = np.mean([r["mean_v"] for r in reps])
            self._card_avgrom["text"] = f"{avg_rom:.1f}"
            self._card_avgv["text"]   = f"{avg_v:.1f}"
        else:
            self._card_avgrom["text"] = "—"
            self._card_avgv["text"]   = "—"

        # czyść listę repów
        for w in self.rep_inner.winfo_children():
            w.destroy()

        for idx, rep in enumerate(reps):
            col = REP_PALETTE[idx % len(REP_PALETTE)]
            row = tk.Frame(self.rep_inner, bg=BG3,
                           highlightbackground=BORDER,
                           highlightthickness=1)
            row.pack(fill="x", pady=(0, 4))

            # numer
            num = tk.Label(row, text=f"{idx+1:02d}",
                           bg=col, fg=BG,
                           font=("SF Mono", 11, "bold"),
                           width=3, pady=6)
            num.pack(side="left")

            # metryki
            metrics_frame = tk.Frame(row, bg=BG3)
            metrics_frame.pack(side="left", padx=8, pady=4)

            self._rep_metric(metrics_frame, "ROM", f"{rep['rom']:.1f}", "cm",   0)
            self._rep_metric(metrics_frame, "V̄",   f"{rep['mean_v']:.1f}", "cm/s", 1)
            self._rep_metric(metrics_frame, "Vmax", f"{rep['peak_v']:.1f}", "cm/s", 2)
            self._rep_metric(metrics_frame, "T",   f"{rep['duration']:.2f}", "s",  3)

    def _rep_metric(self, parent, label, value, unit, col):
        f = tk.Frame(parent, bg=BG3)
        f.grid(row=0, column=col, padx=(0 if col==0 else 10, 0))
        tk.Label(f, text=label, bg=BG3, fg=TEXT_DIM,
                 font=("SF Mono", 8)).pack()
        tk.Label(f, text=f"{value} {unit}", bg=BG3, fg=TEXT,
                 font=("SF Mono", 10, "bold")).pack()

    # ── BLE callbacks (wywoływane z wątku BLE → bezpieczne przez after) ──────
    def _on_status(self, msg, color=TEXT_DIM):
        self.after(0, lambda: self.lbl_status.configure(text=msg, fg=color))

    def _on_progress(self, n):
        self.after(0, lambda: self.progress.configure(value=n))

    def _on_done(self, frames, result, expected):
        def _update():
            n_recv = len(frames)
            lost   = expected - n_recv if expected > 0 else 0
            msg    = (f"Odebrano {n_recv} próbek"
                      + (f"  ·  ⚠ Zgubiono {lost}" if lost > 0 else "")
                      + f"  ·  {len(result['reps'])} rep(y)")
            self.lbl_status.configure(text=msg, fg=GREEN)
            self.progress.configure(value=expected or n_recv)
            self._result = result
            self._draw_results(result)
            self._update_metrics(result)
            self.btn_start.configure(state="normal")
        self.after(0, _update)

    def _on_error(self, msg):
        def _update():
            self.lbl_status.configure(text=f"Błąd: {msg}", fg=ACCENT2)
            self.btn_start.configure(state="normal")
        self.after(0, _update)

    # ── start ─────────────────────────────────────────────────────────────────
    def _on_start(self):
        self.btn_start.configure(state="disabled")
        self.progress.configure(value=0)
        self._draw_empty_plot()
        for w in self.rep_inner.winfo_children():
            w.destroy()
        self._card_reps["text"]   = "—"
        self._card_avgrom["text"] = "—"
        self._card_avgv["text"]   = "—"

        worker = BLEWorker(
            on_status   = self._on_status,
            on_progress = self._on_progress,
            on_done     = self._on_done,
            on_error    = self._on_error,
        )
        worker.start()


# ═══════════════════════════════════════════════════════════════════════════════
if __name__ == "__main__":
    app = W8BandApp()
    app.mainloop()