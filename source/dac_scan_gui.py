#!/usr/bin/env python3
"""Tkinter GUI for the DAC wireless scan receiver."""

from __future__ import annotations

import csv
import queue
import threading
import time
import tkinter as tk
from pathlib import Path
from tkinter import filedialog, messagebox, ttk
from typing import Optional

from dac_scan_host import (
    CMD_ABORT,
    CMD_SCAN_START,
    CMD_SET_DAC,
    EVT_ACK,
    EVT_BLE_RAW,
    EVT_NACK,
    EVT_SCAN_BEGIN,
    EVT_SCAN_END,
    EVT_SCAN_POINTS,
    EVT_SENSOR_READINGS,
    FLAG_SAVE_DAC,
    STATUS,
    TYPE_NAMES,
    Frame,
    FrameParser,
    build_abort,
    build_scan_start,
    build_set_dac,
    describe_frame,
    parse_sensor_readings,
    require_serial,
)


DEFAULT_BAUD = 115200
WAVEFORM_SECONDS = 30.0


class PlotWindow(tk.Toplevel):
    def __init__(self, master: tk.Tk) -> None:
        super().__init__(master)
        self.title("DAC-锁相值曲线")
        self.geometry("760x520")
        self.minsize(560, 380)
        self.points: list[tuple[int, int, int]] = []

        self.columnconfigure(0, weight=1)
        self.rowconfigure(1, weight=1)

        toolbar = ttk.Frame(self)
        toolbar.grid(row=0, column=0, sticky="ew", padx=10, pady=(10, 4))
        toolbar.columnconfigure(0, weight=1)
        self.info_var = tk.StringVar(value="暂无数据")
        ttk.Label(toolbar, textvariable=self.info_var).grid(row=0, column=0, sticky="w")
        ttk.Button(toolbar, text="刷新", command=self.redraw).grid(row=0, column=1, padx=(8, 0))

        self.canvas = tk.Canvas(self, background="white", highlightthickness=1, highlightbackground="#d8dce3")
        self.canvas.grid(row=1, column=0, sticky="nsew", padx=10, pady=(4, 10))
        self.canvas.bind("<Configure>", lambda _event: self.redraw())

    def set_points(self, points: list[tuple[int, int, int]]) -> None:
        self.points = list(points)
        self.redraw()

    def redraw(self) -> None:
        canvas = self.canvas
        canvas.delete("all")

        width = max(canvas.winfo_width(), 2)
        height = max(canvas.winfo_height(), 2)
        left, right, top, bottom = 66, 24, 28, 56
        plot_w = max(width - left - right, 10)
        plot_h = max(height - top - bottom, 10)
        x0, y0 = left, top + plot_h
        x1, y1 = left + plot_w, top

        canvas.create_rectangle(x0, y1, x1, y0, outline="#c9d0da", fill="#fbfcfe")
        canvas.create_text((x0 + x1) / 2, height - 18, text="DAC", fill="#2f3542")
        canvas.create_text(18, (y0 + y1) / 2, text="Lock-in", angle=90, fill="#2f3542")

        if not self.points:
            self.info_var.set("暂无数据")
            canvas.create_text(width / 2, height / 2, text="暂无扫描数据", fill="#7a8494")
            return

        dac_values = [point[1] for point in self.points]
        lockin_values = [point[2] for point in self.points]
        min_dac, max_dac = min(dac_values), max(dac_values)
        min_lockin, max_lockin = min(lockin_values), max(lockin_values)
        if min_dac == max_dac:
            min_dac -= 1
            max_dac += 1
        if min_lockin == max_lockin:
            min_lockin -= 1
            max_lockin += 1

        self.info_var.set(
            f"{len(self.points)} 点    DAC: {min(dac_values)}..{max(dac_values)}    "
            f"Lock-in: {min(lockin_values)}..{max(lockin_values)}"
        )

        for tick in range(6):
            ratio = tick / 5
            x = x0 + ratio * plot_w
            dac_label = min_dac + ratio * (max_dac - min_dac)
            canvas.create_line(x, y1, x, y0, fill="#edf1f5")
            canvas.create_text(x, y0 + 18, text=f"{dac_label:.0f}", fill="#5f6b7a")

            y = y0 - ratio * plot_h
            lockin_label = min_lockin + ratio * (max_lockin - min_lockin)
            canvas.create_line(x0, y, x1, y, fill="#edf1f5")
            canvas.create_text(x0 - 8, y, text=f"{lockin_label:.0f}", anchor="e", fill="#5f6b7a")

        def to_canvas(dac: int, lockin: int) -> tuple[float, float]:
            x = x0 + ((dac - min_dac) / (max_dac - min_dac)) * plot_w
            y = y0 - ((lockin - min_lockin) / (max_lockin - min_lockin)) * plot_h
            return x, y

        line_points: list[float] = []
        for _index, dac, lockin in self.points:
            x, y = to_canvas(dac, lockin)
            line_points.extend([x, y])

        if len(line_points) >= 4:
            canvas.create_line(*line_points, fill="#1f77b4", width=2, smooth=False)

        radius = 3
        for _index, dac, lockin in self.points:
            x, y = to_canvas(dac, lockin)
            canvas.create_oval(x - radius, y - radius, x + radius, y + radius, fill="#1f77b4", outline="white")


class WaveformWindow(tk.Toplevel):
    def __init__(self, master: "DacScanGui") -> None:
        super().__init__(master)
        self.master_app = master
        self.title("传感器 Lock-in 实时波形")
        self.geometry("820x520")
        self.minsize(620, 400)
        self.info_var = tk.StringVar(value="暂无波形数据")

        self.columnconfigure(0, weight=1)
        self.rowconfigure(1, weight=1)

        toolbar = ttk.Frame(self)
        toolbar.grid(row=0, column=0, sticky="ew", padx=10, pady=(10, 4))
        toolbar.columnconfigure(0, weight=1)
        ttk.Label(toolbar, textvariable=self.info_var).grid(row=0, column=0, sticky="w")
        ttk.Button(toolbar, text="保存当前波形", command=self.save_current_waveform).grid(row=0, column=1, padx=(8, 0))
        ttk.Button(toolbar, text="保存全部数据", command=self.save_all_waveform).grid(row=0, column=2, padx=(8, 0))
        ttk.Button(toolbar, text="清空波形", command=master.clear_waveform).grid(row=0, column=3, padx=(8, 0))
        ttk.Button(toolbar, text="刷新", command=self.redraw).grid(row=0, column=4, padx=(8, 0))

        self.canvas = tk.Canvas(self, background="white", highlightthickness=1, highlightbackground="#d8dce3")
        self.canvas.grid(row=1, column=0, sticky="nsew", padx=10, pady=(4, 10))
        self.canvas.bind("<Configure>", lambda _event: self.redraw())

    def set_latest(self, dac: int, lockin: int, adc: int) -> None:
        self.info_var.set(
            f"最近 {WAVEFORM_SECONDS:.0f}s    最新 Lock-in={lockin}    DAC={dac}    ADC={adc}"
        )

    def redraw(self) -> None:
        canvas = self.canvas
        canvas.delete("all")

        width = max(canvas.winfo_width(), 2)
        height = max(canvas.winfo_height(), 2)
        left, right, top, bottom = 68, 24, 28, 52
        plot_w = max(width - left - right, 10)
        plot_h = max(height - top - bottom, 10)
        x0, y0 = left, top + plot_h
        x1, y1 = left + plot_w, top

        canvas.create_rectangle(x0, y1, x1, y0, outline="#c9d0da", fill="#fbfcfe")
        canvas.create_text((x0 + x1) / 2, height - 18, text="时间 (s)", fill="#2f3542")
        canvas.create_text(18, (y0 + y1) / 2, text="Lock-in", angle=90, fill="#2f3542")

        now = time.time()
        x_min = now - WAVEFORM_SECONDS
        visible = [point for point in self.master_app.waveform_points if point[0] >= x_min]
        if not visible:
            self.info_var.set("暂无波形数据")
            canvas.create_text(width / 2, height / 2, text="暂无实时读数", fill="#7a8494")
            return

        values = [point[3] for point in visible]
        min_value, max_value = min(values), max(values)
        if min_value == max_value:
            min_value -= 1
            max_value += 1
        else:
            padding = max(1, int((max_value - min_value) * 0.08))
            min_value -= padding
            max_value += padding

        for tick in range(6):
            ratio = tick / 5
            x = x0 + ratio * plot_w
            seconds_label = -WAVEFORM_SECONDS + ratio * WAVEFORM_SECONDS
            canvas.create_line(x, y1, x, y0, fill="#edf1f5")
            canvas.create_text(x, y0 + 18, text=f"{seconds_label:.0f}", fill="#5f6b7a")

            y = y0 - ratio * plot_h
            value_label = min_value + ratio * (max_value - min_value)
            canvas.create_line(x0, y, x1, y, fill="#edf1f5")
            canvas.create_text(x0 - 8, y, text=f"{value_label:.0f}", anchor="e", fill="#5f6b7a")

        line_points: list[float] = []
        for ts, _seq, _dac, value, _adc in visible:
            x = x0 + ((ts - x_min) / WAVEFORM_SECONDS) * plot_w
            y = y0 - ((value - min_value) / (max_value - min_value)) * plot_h
            line_points.extend([x, y])

        if len(line_points) >= 4:
            canvas.create_line(*line_points, fill="#d14b3f", width=2, smooth=False)
        else:
            x, y = line_points
            canvas.create_oval(x - 3, y - 3, x + 3, y + 3, fill="#d14b3f", outline="")

    def save_current_waveform(self) -> None:
        now = time.time()
        visible = [point for point in self.master_app.waveform_points if point[0] >= now - WAVEFORM_SECONDS]
        self._save_waveform_rows(visible, "current")

    def save_all_waveform(self) -> None:
        self._save_waveform_rows(list(self.master_app.waveform_points), "all")

    def _save_waveform_rows(self, rows: list[tuple[float, int, int, int, int]], suffix: str) -> None:
        if not rows:
            messagebox.showinfo("没有数据", "当前没有可保存的波形数据。")
            return

        default_name = f"lockin_waveform_{suffix}_{time.strftime('%Y%m%d_%H%M%S')}.csv"
        path = filedialog.asksaveasfilename(
            title="保存实时波形数据",
            initialfile=default_name,
            defaultextension=".csv",
            filetypes=[("CSV", "*.csv"), ("All files", "*.*")],
        )
        if not path:
            return

        start_time = rows[0][0]
        with Path(path).open("w", newline="", encoding="utf-8-sig") as handle:
            writer = csv.writer(handle)
            writer.writerow(["timestamp", "elapsed_s", "seq", "dac", "lockin", "adc"])
            for timestamp, seq, dac, lockin, adc in rows:
                local_time = time.strftime("%Y-%m-%d %H:%M:%S", time.localtime(timestamp))
                millis = int((timestamp % 1.0) * 1000)
                writer.writerow([
                    f"{local_time}.{millis:03d}",
                    f"{timestamp - start_time:.3f}",
                    seq,
                    dac,
                    lockin,
                    adc,
                ])

        messagebox.showinfo("保存完成", f"已保存 {len(rows)} 条波形数据。")


class SerialWorker:
    def __init__(self, rx_queue: "queue.Queue[object]") -> None:
        self._rx_queue = rx_queue
        self._parser = FrameParser()
        self._serial = None
        self._thread: Optional[threading.Thread] = None
        self._stop = threading.Event()
        self._write_lock = threading.Lock()

    @property
    def is_open(self) -> bool:
        return self._serial is not None and self._serial.is_open

    def open(self, port: str, baud: int) -> None:
        if self.is_open:
            return
        serial = require_serial()
        self._stop.clear()
        self._serial = serial.Serial(port, baud, timeout=0.05)
        self._thread = threading.Thread(target=self._read_loop, daemon=True)
        self._thread.start()
        self._rx_queue.put(("status", f"已连接 {port} @ {baud}"))

    def close(self) -> None:
        self._stop.set()
        ser = self._serial
        self._serial = None
        if ser is not None:
            try:
                ser.close()
            except Exception as exc:  # pragma: no cover - defensive UI path
                self._rx_queue.put(("error", f"关闭串口失败：{exc}"))
        self._rx_queue.put(("status", "串口已断开"))

    def write(self, data: bytes, label: str) -> None:
        ser = self._serial
        if ser is None or not ser.is_open:
            raise RuntimeError("串口未连接")
        with self._write_lock:
            ser.write(data)
        self._rx_queue.put(("tx", f"{label} ({len(data)} bytes)"))

    def _read_loop(self) -> None:
        while not self._stop.is_set():
            ser = self._serial
            if ser is None:
                break
            try:
                data = ser.read(512)
            except Exception as exc:
                self._rx_queue.put(("error", f"串口读取失败：{exc}"))
                break
            if not data:
                continue
            for frame in self._parser.feed(data):
                self._rx_queue.put(("frame", frame))


class DacScanGui(tk.Tk):
    def __init__(self) -> None:
        super().__init__()
        self.title("DAC 无线扫描上位机")
        self.geometry("1120x760")
        self.minsize(960, 660)

        self.rx_queue: "queue.Queue[object]" = queue.Queue()
        self.worker = SerialWorker(self.rx_queue)
        self.seq = 1
        self.scan_id = 1
        self.points: list[tuple[int, int, int]] = []
        self.sensor_readings: list[tuple[int, int, int, int]] = []
        self.waveform_points: list[tuple[float, int, int, int, int]] = []
        self.plot_window: Optional[PlotWindow] = None
        self.waveform_window: Optional[WaveformWindow] = None
        self.last_heartbeat_log = 0.0
        self.last_sensor_log = 0.0
        self.last_waveform_redraw = 0.0

        self.port_var = tk.StringVar()
        self.baud_var = tk.StringVar(value=str(DEFAULT_BAUD))
        self.dac_min_var = tk.StringVar(value="0")
        self.dac_max_var = tk.StringVar(value="65535")
        self.points_var = tk.StringVar(value="100")
        self.settle_var = tk.StringVar(value="20")
        self.avg_var = tk.StringVar(value="1")
        self.set_dac_var = tk.StringVar(value="32768")
        self.status_var = tk.StringVar(value="未连接")
        self.progress_var = tk.DoubleVar(value=0)

        self._build_ui()
        self.refresh_ports()
        self.after(50, self._poll_queue)
        self.protocol("WM_DELETE_WINDOW", self._on_close)

    def _build_ui(self) -> None:
        self.columnconfigure(0, weight=1)
        self.rowconfigure(2, weight=1)

        connection = ttk.LabelFrame(self, text="连接")
        connection.grid(row=0, column=0, sticky="ew", padx=10, pady=(10, 6))
        connection.columnconfigure(1, weight=1)

        ttk.Label(connection, text="串口").grid(row=0, column=0, padx=8, pady=8, sticky="w")
        self.port_combo = ttk.Combobox(connection, textvariable=self.port_var, width=42)
        self.port_combo.grid(row=0, column=1, padx=4, pady=8, sticky="ew")
        ttk.Button(connection, text="刷新", command=self.refresh_ports).grid(row=0, column=2, padx=4, pady=8)
        ttk.Label(connection, text="波特率").grid(row=0, column=3, padx=(14, 4), pady=8)
        ttk.Entry(connection, textvariable=self.baud_var, width=10).grid(row=0, column=4, padx=4, pady=8)
        self.connect_button = ttk.Button(connection, text="连接", command=self.toggle_connection)
        self.connect_button.grid(row=0, column=5, padx=8, pady=8)

        controls = ttk.LabelFrame(self, text="操作")
        controls.grid(row=1, column=0, sticky="ew", padx=10, pady=6)
        for col in range(12):
            controls.columnconfigure(col, weight=0)
        controls.columnconfigure(11, weight=1)

        self._add_labeled_entry(controls, "起点", self.dac_min_var, 0, 0)
        self._add_labeled_entry(controls, "终点", self.dac_max_var, 0, 2)
        self._add_labeled_entry(controls, "点数", self.points_var, 0, 4)
        self._add_labeled_entry(controls, "等待(ms)", self.settle_var, 0, 6)
        self._add_labeled_entry(controls, "平均", self.avg_var, 0, 8)
        ttk.Button(controls, text="开始扫描", command=self.start_scan).grid(row=0, column=10, padx=8, pady=8)
        ttk.Button(controls, text="终止", command=self.abort_scan).grid(row=0, column=11, padx=4, pady=8, sticky="w")

        self._add_labeled_entry(controls, "DAC", self.set_dac_var, 1, 0)
        ttk.Button(controls, text="设置 DAC", command=self.set_dac).grid(row=1, column=2, padx=4, pady=8, sticky="ew")
        ttk.Button(controls, text="保存 DAC", command=self.save_dac).grid(row=1, column=3, padx=4, pady=8, sticky="ew")
        ttk.Button(controls, text="清空日志", command=self.clear_log).grid(row=1, column=4, padx=4, pady=8, sticky="ew")
        ttk.Button(controls, text="导出点表", command=self.export_points).grid(row=1, column=5, padx=4, pady=8, sticky="ew")
        ttk.Button(controls, text="协议自测", command=self.self_test).grid(row=1, column=6, padx=4, pady=8, sticky="ew")
        ttk.Button(controls, text="绘图窗口", command=self.open_plot_window).grid(row=1, column=7, padx=4, pady=8, sticky="ew")
        ttk.Button(controls, text="实时波形窗口", command=self.open_waveform_window).grid(row=1, column=8, padx=4, pady=8, sticky="ew")
        ttk.Progressbar(controls, variable=self.progress_var, maximum=100).grid(row=1, column=9, columnspan=3, padx=8, pady=8, sticky="ew")

        body = ttk.PanedWindow(self, orient=tk.HORIZONTAL)
        body.grid(row=2, column=0, sticky="nsew", padx=10, pady=6)

        data_tabs = ttk.Notebook(body)

        points_frame = ttk.Frame(data_tabs)
        points_frame.rowconfigure(0, weight=1)
        points_frame.columnconfigure(0, weight=1)
        self.points_tree = ttk.Treeview(points_frame, columns=("index", "dac", "lockin"), show="headings", height=16)
        self.points_tree.heading("index", text="序号")
        self.points_tree.heading("dac", text="DAC")
        self.points_tree.heading("lockin", text="Lock-in")
        self.points_tree.column("index", width=80, anchor="center")
        self.points_tree.column("dac", width=120, anchor="center")
        self.points_tree.column("lockin", width=120, anchor="center")
        self.points_tree.grid(row=0, column=0, sticky="nsew")
        points_scroll = ttk.Scrollbar(points_frame, orient=tk.VERTICAL, command=self.points_tree.yview)
        points_scroll.grid(row=0, column=1, sticky="ns")
        self.points_tree.configure(yscrollcommand=points_scroll.set)
        data_tabs.add(points_frame, text="扫描点")

        readings_frame = ttk.Frame(data_tabs)
        readings_frame.rowconfigure(0, weight=1)
        readings_frame.columnconfigure(0, weight=1)
        self.readings_tree = ttk.Treeview(
            readings_frame,
            columns=("seq", "dac", "lockin", "adc"),
            show="headings",
            height=16,
        )
        self.readings_tree.heading("seq", text="序号")
        self.readings_tree.heading("dac", text="DAC")
        self.readings_tree.heading("lockin", text="Lock-in")
        self.readings_tree.heading("adc", text="ADC")
        self.readings_tree.column("seq", width=80, anchor="center")
        self.readings_tree.column("dac", width=110, anchor="center")
        self.readings_tree.column("lockin", width=110, anchor="center")
        self.readings_tree.column("adc", width=110, anchor="center")
        self.readings_tree.grid(row=0, column=0, sticky="nsew")
        readings_scroll = ttk.Scrollbar(readings_frame, orient=tk.VERTICAL, command=self.readings_tree.yview)
        readings_scroll.grid(row=0, column=1, sticky="ns")
        self.readings_tree.configure(yscrollcommand=readings_scroll.set)
        data_tabs.add(readings_frame, text="传感器读数")
        body.add(data_tabs, weight=1)

        log_frame = ttk.LabelFrame(body, text="通信日志")
        log_frame.rowconfigure(0, weight=1)
        log_frame.columnconfigure(0, weight=1)
        self.log_text = tk.Text(log_frame, wrap="word", height=16)
        self.log_text.grid(row=0, column=0, sticky="nsew")
        log_scroll = ttk.Scrollbar(log_frame, orient=tk.VERTICAL, command=self.log_text.yview)
        log_scroll.grid(row=0, column=1, sticky="ns")
        self.log_text.configure(yscrollcommand=log_scroll.set)
        body.add(log_frame, weight=2)

        status = ttk.Label(self, textvariable=self.status_var, anchor="w")
        status.grid(row=3, column=0, sticky="ew", padx=10, pady=(0, 10))

    def _add_labeled_entry(self, parent: ttk.Widget, label: str, variable: tk.StringVar, row: int, column: int) -> None:
        ttk.Label(parent, text=label).grid(row=row, column=column, padx=(8, 4), pady=8, sticky="w")
        ttk.Entry(parent, textvariable=variable, width=10).grid(row=row, column=column + 1, padx=4, pady=8)

    def refresh_ports(self) -> None:
        try:
            serial = require_serial()
            from serial.tools import list_ports  # type: ignore

            ports = [f"{p.device}  {p.description}" for p in list_ports.comports()]
        except SystemExit as exc:
            messagebox.showerror("缺少依赖", str(exc))
            return
        self.port_combo["values"] = ports
        if ports and not self.port_var.get():
            preferred = next((p for p in ports if "STLink" in p or "STMicroelectronics" in p), ports[0])
            self.port_var.set(preferred)
        self.log(f"刷新串口：{len(ports)} 个")

    def toggle_connection(self) -> None:
        if self.worker.is_open:
            self.worker.close()
            self.connect_button.configure(text="连接")
            return
        port = self._selected_port()
        if not port:
            messagebox.showwarning("缺少串口", "请先选择或输入串口号。")
            return
        try:
            baud = self._read_int(self.baud_var, "波特率", 1200, 3000000)
            self.worker.open(port, baud)
            self.connect_button.configure(text="断开")
        except Exception as exc:
            messagebox.showerror("连接失败", str(exc))

    def start_scan(self) -> None:
        try:
            dac_min = self._read_int(self.dac_min_var, "DAC 起点", 0, 0xFFFF)
            dac_max = self._read_int(self.dac_max_var, "DAC 终点", 0, 0xFFFF)
            points = self._read_int(self.points_var, "点数", 2, 255)
            settle = self._read_int(self.settle_var, "等待时间", 0, 10000)
            avg = self._read_int(self.avg_var, "平均次数", 1, 255)
            frame = build_scan_start(self.scan_id, self.seq, dac_min, dac_max, points, settle, avg)
            self.points.clear()
            self.points_tree.delete(*self.points_tree.get_children())
            self.refresh_plot()
            self.progress_var.set(0)
            self.worker.write(frame, f"CMD_SCAN_START scan_id={self.scan_id} seq={self.seq}")
            self.seq += 1
        except Exception as exc:
            messagebox.showerror("发送失败", str(exc))

    def set_dac(self, save: bool = False) -> None:
        try:
            dac = self._read_int(self.set_dac_var, "DAC", 0, 0xFFFF)
            frame = build_set_dac(self.scan_id, self.seq, dac, save=save)
            save_text = "保存" if save else "不保存"
            self.worker.write(frame, f"CMD_SET_DAC dac={dac} {save_text} seq={self.seq}")
            self.seq += 1
        except Exception as exc:
            messagebox.showerror("发送失败", str(exc))

    def save_dac(self) -> None:
        self.set_dac(save=True)

    def abort_scan(self) -> None:
        try:
            frame = build_abort(self.scan_id, self.seq)
            self.worker.write(frame, f"CMD_ABORT seq={self.seq}")
            self.seq += 1
        except Exception as exc:
            messagebox.showerror("发送失败", str(exc))

    def self_test(self) -> None:
        from dac_scan_host import self_test

        try:
            self_test()
            self.log("协议自测 OK")
            messagebox.showinfo("协议自测", "协议自测通过。")
        except Exception as exc:
            messagebox.showerror("协议自测失败", str(exc))

    def clear_log(self) -> None:
        self.log_text.delete("1.0", tk.END)

    def clear_waveform(self) -> None:
        self.waveform_points.clear()
        self.refresh_waveform()

    def open_plot_window(self) -> None:
        if self.plot_window is None or not self.plot_window.winfo_exists():
            self.plot_window = PlotWindow(self)
        else:
            self.plot_window.lift()
            self.plot_window.focus_force()
        self.plot_window.set_points(self.points)

    def refresh_plot(self) -> None:
        if self.plot_window is not None and self.plot_window.winfo_exists():
            self.plot_window.set_points(self.points)

    def open_waveform_window(self) -> None:
        if self.waveform_window is None or not self.waveform_window.winfo_exists():
            self.waveform_window = WaveformWindow(self)
        else:
            self.waveform_window.lift()
            self.waveform_window.focus_force()
        self.refresh_waveform()

    def refresh_waveform(self) -> None:
        if self.waveform_window is not None and self.waveform_window.winfo_exists():
            self.waveform_window.redraw()

    def export_points(self) -> None:
        if not self.points:
            messagebox.showinfo("没有数据", "当前没有扫描点可导出。")
            return
        default_name = f"dac_scan_{time.strftime('%Y%m%d_%H%M%S')}.csv"
        path = filedialog.asksaveasfilename(
            title="导出扫描点",
            initialfile=default_name,
            defaultextension=".csv",
            filetypes=[("CSV", "*.csv"), ("All files", "*.*")],
        )
        if not path:
            return
        with Path(path).open("w", newline="", encoding="utf-8-sig") as handle:
            writer = csv.writer(handle)
            writer.writerow(["index", "dac", "lockin"])
            writer.writerows(self.points)
        self.log(f"已导出 {len(self.points)} 点到 {path}")

    def _poll_queue(self) -> None:
        while True:
            try:
                item = self.rx_queue.get_nowait()
            except queue.Empty:
                break
            kind = item[0]
            payload = item[1]
            if kind == "frame":
                self._handle_frame(payload)
            elif kind == "tx":
                self.log(f"TX {payload}")
            elif kind == "status":
                self.status_var.set(str(payload))
                self.log(str(payload))
            elif kind == "error":
                self.log(f"ERROR {payload}")
                self.status_var.set(str(payload))
        self.after(50, self._poll_queue)

    def _handle_frame(self, frame: Frame) -> None:
        if frame.frame_type == EVT_BLE_RAW and frame.payload == b"PC_ALIVE":
            now = time.time()
            self.status_var.set("接收器在线")
            if now - self.last_heartbeat_log >= 5:
                self.last_heartbeat_log = now
                self.log("RX PC_ALIVE")
            return

        if frame.frame_type == EVT_SENSOR_READINGS:
            self._handle_sensor_readings(frame)
            now = time.time()
            if now - self.last_sensor_log >= 2:
                self.last_sensor_log = now
                self.log("RX " + describe_frame(frame))
            return

        self.log("RX " + describe_frame(frame))
        if frame.frame_type == EVT_SCAN_BEGIN:
            self.points.clear()
            self.points_tree.delete(*self.points_tree.get_children())
            self.refresh_plot()
            self.progress_var.set(0)
            self.status_var.set("扫描开始")
        elif frame.frame_type == EVT_SCAN_POINTS:
            self._handle_points(frame)
        elif frame.frame_type == EVT_SCAN_END:
            self.progress_var.set(100)
            self.status_var.set("扫描结束")
        elif frame.frame_type == EVT_ACK:
            self.status_var.set(self._ack_text(frame, "ACK"))
        elif frame.frame_type == EVT_NACK:
            self.status_var.set(self._ack_text(frame, "NACK"))
        elif frame.frame_type == EVT_BLE_RAW:
            self.status_var.set("收到 BLE 原始数据")

    def _handle_points(self, frame: Frame) -> None:
        if not frame.payload:
            return
        count = frame.payload[0]
        offset = 1
        for _ in range(count):
            if offset + 5 > len(frame.payload):
                break
            index = frame.payload[offset]
            dac = frame.payload[offset + 1] | (frame.payload[offset + 2] << 8)
            lockin = frame.payload[offset + 3] | (frame.payload[offset + 4] << 8)
            self.points.append((index, dac, lockin))
            self.points_tree.insert("", tk.END, values=(index, dac, lockin))
            offset += 5
        try:
            total = max(1, self._read_int(self.points_var, "点数", 1, 255))
            self.progress_var.set(min(100, len(self.points) * 100 / total))
        except ValueError:
            pass
        self.refresh_plot()

    def _handle_sensor_readings(self, frame: Frame) -> None:
        readings = parse_sensor_readings(frame.payload)
        if not readings:
            return
        now = time.time()
        for seq, dac, lockin, adc in readings:
            self.sensor_readings.append((seq, dac, lockin, adc))
            self.waveform_points.append((now, seq, dac, lockin, adc))
            self.readings_tree.insert("", tk.END, values=(seq, dac, lockin, adc))
        while len(self.sensor_readings) > 1000:
            self.sensor_readings.pop(0)
            children = self.readings_tree.get_children()
            if children:
                self.readings_tree.delete(children[0])
        seq, dac, lockin, adc = readings[-1]
        if self.waveform_window is not None and self.waveform_window.winfo_exists():
            self.waveform_window.set_latest(dac, lockin, adc)
        if now - self.last_waveform_redraw >= 0.1:
            self.last_waveform_redraw = now
            self.refresh_waveform()
        self.status_var.set(f"传感器读数: seq={seq} DAC={dac} Lock-in={lockin} ADC={adc}")

    def _ack_text(self, frame: Frame, label: str) -> str:
        if len(frame.payload) < 2:
            return label
        target = TYPE_NAMES.get(frame.payload[0], f"0x{frame.payload[0]:02X}")
        status = STATUS.get(frame.payload[1], f"0x{frame.payload[1]:02X}")
        detail = ""
        if frame.payload[0] == CMD_SET_DAC and len(frame.payload) >= 5:
            dac = frame.payload[2] | (frame.payload[3] << 8)
            save_text = "已保存" if (frame.payload[4] & FLAG_SAVE_DAC) else "未保存"
            detail = f" DAC确认={dac} {save_text}"
        return f"{label}: {target} {status}{detail}"

    def _selected_port(self) -> str:
        value = self.port_var.get().strip()
        return value.split()[0] if value else ""

    def _read_int(self, variable: tk.StringVar, name: str, minimum: int, maximum: int) -> int:
        try:
            value = int(variable.get().strip(), 0)
        except ValueError as exc:
            raise ValueError(f"{name} 必须是数字") from exc
        if not minimum <= value <= maximum:
            raise ValueError(f"{name} 必须在 {minimum} 到 {maximum} 之间")
        return value

    def log(self, text: str) -> None:
        timestamp = time.strftime("%H:%M:%S")
        self.log_text.insert(tk.END, f"[{timestamp}] {text}\n")
        self.log_text.see(tk.END)

    def _on_close(self) -> None:
        self.worker.close()
        self.destroy()


def main(argv: Optional[list[str]] = None) -> int:
    app = DacScanGui()
    app.mainloop()
    return 0


if __name__ == "__main__":
    raise SystemExit(main())
