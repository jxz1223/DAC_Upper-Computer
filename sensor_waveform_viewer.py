import csv
import struct
import sys
import time
from collections import deque
from datetime import datetime
from pathlib import Path

import numpy as np
import pyqtgraph as pg
import serial
from serial.tools import list_ports
from PyQt5 import QtCore, QtGui, QtWidgets


APP_NAME = "传感器波形查看器"
FRAME_SIZE = 246
FRAME_HEADER = b"\x8f\x8e\x8f"
FRAME_TRAILER = b"\x8e\x8f\x8e"
SAMPLES_PER_FRAME = 120

DC_MAGIC = b"\xA5\x5A"
DC_VERSION = 1
DC_CMD_SCAN_START = 0x10
DC_CMD_SET_DAC = 0x11
DC_CMD_ABORT = 0x12
DC_EVT_SCAN_BEGIN = 0x90
DC_EVT_SCAN_POINTS = 0x91
DC_EVT_SCAN_END = 0x92
DC_EVT_SENSOR_READINGS = 0x93


def dc_crc16(data):
    crc = 0xFFFF
    for byte in data:
        crc ^= byte << 8
        for _ in range(8):
            crc = ((crc << 1) ^ 0x1021) & 0xFFFF if crc & 0x8000 else (crc << 1) & 0xFFFF
    return crc


def dc_encode(frame_type, flags=0, scan_id=1, seq=1, payload=b""):
    header = struct.pack("<BBBHHB", DC_VERSION, frame_type, flags, scan_id, seq, len(payload))
    return DC_MAGIC + header + payload + struct.pack("<H", dc_crc16(header + payload))


class DcFrameParser:
    def __init__(self):
        self.buffer = bytearray()
        self.valid_frames = 0
        self.discarded_bytes = 0

    def clear(self):
        self.buffer.clear()
        self.valid_frames = self.discarded_bytes = 0

    def feed(self, chunk):
        self.buffer.extend(chunk)
        frames = []
        while True:
            pos = self.buffer.find(DC_MAGIC)
            if pos < 0:
                keep = 1 if self.buffer and self.buffer[-1] == DC_MAGIC[0] else 0
                self.discarded_bytes += len(self.buffer) - keep
                self.buffer[:] = self.buffer[-keep:] if keep else b""
                break
            if pos:
                del self.buffer[:pos]
                self.discarded_bytes += pos
            if len(self.buffer) < 12:
                break
            version, kind, flags, scan_id, seq, size = struct.unpack_from("<BBBHHB", self.buffer, 2)
            total = 12 + size
            if size > 246:
                del self.buffer[0]
                self.discarded_bytes += 1
                continue
            if len(self.buffer) < total:
                break
            payload = bytes(self.buffer[10:10 + size])
            received = struct.unpack_from("<H", self.buffer, 10 + size)[0]
            calculated = dc_crc16(bytes(self.buffer[2:10 + size]))
            del self.buffer[:total]
            if version == DC_VERSION and received == calculated:
                frames.append((kind, flags, scan_id, seq, payload))
                self.valid_frames += 1
            else:
                self.discarded_bytes += total
        return frames


class FrameParser:
    """Robust parser for arbitrary serial chunks with automatic resynchronization."""

    def __init__(self):
        self.buffer = bytearray()
        self.valid_frames = 0
        self.discarded_bytes = 0

    def clear(self):
        self.buffer.clear()
        self.valid_frames = 0
        self.discarded_bytes = 0

    def feed(self, chunk):
        self.buffer.extend(chunk)
        frames = []
        while True:
            pos = self.buffer.find(FRAME_HEADER)
            if pos < 0:
                keep = min(len(self.buffer), len(FRAME_HEADER) - 1)
                self.discarded_bytes += len(self.buffer) - keep
                if keep:
                    self.buffer[:] = self.buffer[-keep:]
                else:
                    self.buffer.clear()
                break
            if pos:
                del self.buffer[:pos]
                self.discarded_bytes += pos
            if len(self.buffer) < FRAME_SIZE:
                break
            if self.buffer[FRAME_SIZE - 3:FRAME_SIZE] == FRAME_TRAILER:
                frames.append(bytes(self.buffer[:FRAME_SIZE]))
                del self.buffer[:FRAME_SIZE]
                self.valid_frames += 1
            else:
                del self.buffer[0]
                self.discarded_bytes += 1
        return frames


class SerialWorker(QtCore.QObject):
    bytes_received = QtCore.pyqtSignal(bytes)
    state_changed = QtCore.pyqtSignal(bool, str)
    error = QtCore.pyqtSignal(str)

    def __init__(self):
        super().__init__()
        self.port = None
        self.timer = QtCore.QTimer(self)
        self.timer.setInterval(5)
        self.timer.timeout.connect(self.poll)

    @QtCore.pyqtSlot(str, int)
    def open_port(self, name, baud):
        self.close_port()
        try:
            self.port = serial.Serial(
                port=name,
                baudrate=baud,
                bytesize=serial.EIGHTBITS,
                parity=serial.PARITY_NONE,
                stopbits=serial.STOPBITS_ONE,
                timeout=0,
            )
            self.port.reset_input_buffer()
            self.timer.start()
            self.state_changed.emit(True, f"{name} @ {baud}")
        except Exception as exc:
            self.port = None
            self.error.emit(f"无法打开串口：{exc}")
            self.state_changed.emit(False, "未连接")

    @QtCore.pyqtSlot()
    def close_port(self):
        self.timer.stop()
        if self.port is not None:
            try:
                self.port.close()
            except Exception:
                pass
        self.port = None
        self.state_changed.emit(False, "未连接")

    @QtCore.pyqtSlot()
    def poll(self):
        if self.port is None:
            return
        try:
            count = self.port.in_waiting
            if count:
                self.bytes_received.emit(self.port.read(count))
        except Exception as exc:
            self.error.emit(f"串口读取失败：{exc}")
            self.close_port()

    @QtCore.pyqtSlot(bytes)
    def write(self, data):
        if self.port is None:
            self.error.emit("串口未连接。")
            return
        try:
            self.port.write(data)
        except Exception as exc:
            self.error.emit(f"串口写入失败：{exc}")


class SelectionViewBox(pg.ViewBox):
    range_selected = QtCore.pyqtSignal(float, float)
    point_clicked = QtCore.pyqtSignal(float, float)

    def __init__(self):
        super().__init__(enableMenu=True)
        self.selection_enabled = False
        self.drag_start = None
        self.setMouseMode(self.PanMode)

    def mouseClickEvent(self, event):
        if event.button() == QtCore.Qt.LeftButton:
            point = self.mapSceneToView(event.scenePos())
            self.point_clicked.emit(point.x(), point.y())
            event.accept()
            return
        super().mouseClickEvent(event)

    def mouseDragEvent(self, event, axis=None):
        if self.selection_enabled and event.button() == QtCore.Qt.LeftButton:
            point = self.mapSceneToView(event.scenePos())
            if event.isStart():
                self.drag_start = point.x()
            elif event.isFinish() and self.drag_start is not None:
                x1, x2 = sorted((self.drag_start, point.x()))
                if x2 > x1:
                    self.range_selected.emit(x1, x2)
                self.drag_start = None
            event.accept()
            return
        super().mouseDragEvent(event, axis=axis)


class MainWindow(QtWidgets.QMainWindow):
    request_open = QtCore.pyqtSignal(str, int)
    request_close = QtCore.pyqtSignal()
    request_write = QtCore.pyqtSignal(bytes)

    def __init__(self):
        super().__init__()
        self.setWindowTitle(APP_NAME)
        self.resize(1280, 800)
        self.setMinimumSize(980, 650)

        self.parser = FrameParser()
        self.dc_parser = DcFrameParser()
        self.dc_rows = deque()
        self.dc_scan_points = []
        self.dc_seq = 1
        self.dc_start_time = None
        self.sample_chunks = deque()
        self.sample_count = 0
        self.total_samples = 0
        self.total_bytes = 0
        self.zero_frames = 0
        self.nonzero_frames = 0
        self.paused = False
        self.plot_dirty = False
        self.last_x = np.empty(0)
        self.last_y = np.empty(0)
        self.last_stats_values = np.empty(0)
        self.last_stats_scope = "当前窗口"

        self.worker_thread = QtCore.QThread(self)
        self.worker = SerialWorker()
        self.worker.moveToThread(self.worker_thread)
        self.request_open.connect(self.worker.open_port)
        self.request_close.connect(self.worker.close_port)
        self.request_write.connect(self.worker.write)
        self.worker.bytes_received.connect(self.on_bytes)
        self.worker.state_changed.connect(self.on_serial_state)
        self.worker.error.connect(self.show_error)
        self.worker_thread.start()

        self.build_ui()
        self.refresh_ports()

        self.plot_timer = QtCore.QTimer(self)
        self.plot_timer.timeout.connect(self.refresh_plot)
        self.plot_timer.start(33)

    def build_ui(self):
        pg.setConfigOptions(antialias=False, background="#0f172a", foreground="#cbd5e1")
        root = QtWidgets.QWidget()
        self.setCentralWidget(root)
        layout = QtWidgets.QVBoxLayout(root)
        layout.setContentsMargins(10, 10, 10, 8)

        mode_row = QtWidgets.QHBoxLayout()
        mode_row.addWidget(QtWidgets.QLabel("传感器类型："))
        self.ac_mode = QtWidgets.QCheckBox("交流传感器")
        self.dc_mode = QtWidgets.QCheckBox("直流传感器")
        self.ac_mode.setChecked(True)
        self.sensor_modes = QtWidgets.QButtonGroup(self)
        self.sensor_modes.setExclusive(True)
        self.sensor_modes.addButton(self.ac_mode)
        self.sensor_modes.addButton(self.dc_mode)
        mode_row.addWidget(self.ac_mode)
        mode_row.addWidget(self.dc_mode)
        mode_row.addStretch(1)
        layout.addLayout(mode_row)

        connection_box = QtWidgets.QGroupBox("串口与采样设置")
        grid = QtWidgets.QGridLayout(connection_box)
        self.port_combo = QtWidgets.QComboBox()
        self.port_combo.setMinimumWidth(190)
        self.refresh_button = QtWidgets.QPushButton("刷新串口")
        self.refresh_button.clicked.connect(self.refresh_ports)
        self.baud_combo = QtWidgets.QComboBox()
        self.baud_combo.setEditable(True)
        self.baud_combo.addItems(
            ["9600", "19200", "38400", "57600", "115200", "230400",
             "460800", "921600", "1000000", "2000000"]
        )
        self.baud_combo.setCurrentText("921600")
        self.detect_baud_button = QtWidgets.QPushButton("自动检测波特率")
        self.detect_baud_button.clicked.connect(self.detect_baud)
        self.sample_rate = QtWidgets.QDoubleSpinBox()
        self.sample_rate.setRange(0.001, 10_000_000)
        self.sample_rate.setDecimals(3)
        self.sample_rate.setValue(8000)
        self.sample_rate.setSuffix(" Hz")
        self.sample_rate.valueChanged.connect(self.mark_dirty)
        self.duration_combo = QtWidgets.QComboBox()
        self.duration_combo.setEditable(True)
        self.duration_combo.addItems([
            "1 ms", "2 ms", "5 ms", "10 ms", "20 ms", "50 ms",
            "100 ms", "200 ms", "500 ms", "1 s", "2 s", "5 s",
            "10 s", "20 s", "30 s", "60 s", "2 min", "5 min", "10 min", "1 h", "全部",
        ])
        self.duration_combo.setCurrentText("5 s")
        self.duration_combo.setToolTip("可选择常用毫秒/秒档位，也可输入如 25 ms 或 0.25 s")
        self.duration_combo.currentTextChanged.connect(self.on_duration_changed)
        self.data_format = QtWidgets.QComboBox()
        self.data_format.addItems(["无符号16位（大端）", "有符号16位（大端）"])
        self.connect_button = QtWidgets.QPushButton("连接")
        self.connect_button.setCheckable(True)
        self.connect_button.clicked.connect(self.toggle_connection)

        grid.addWidget(QtWidgets.QLabel("串口"), 0, 0)
        grid.addWidget(self.port_combo, 0, 1)
        grid.addWidget(self.refresh_button, 0, 2)
        grid.addWidget(QtWidgets.QLabel("波特率"), 0, 3)
        grid.addWidget(self.baud_combo, 0, 4)
        grid.addWidget(QtWidgets.QLabel("采样率"), 0, 5)
        grid.addWidget(self.sample_rate, 0, 6)
        grid.addWidget(QtWidgets.QLabel("显示时长"), 1, 0)
        grid.addWidget(self.duration_combo, 1, 1)
        grid.addWidget(QtWidgets.QLabel("数据格式"), 1, 3)
        grid.addWidget(self.data_format, 1, 4)
        grid.addWidget(self.detect_baud_button, 1, 5)
        grid.addWidget(self.connect_button, 1, 6)
        layout.addWidget(connection_box)

        self.dc_controls = QtWidgets.QGroupBox("直流传感器 DAC 控制")
        dc_grid = QtWidgets.QGridLayout(self.dc_controls)
        self.dc_min = QtWidgets.QSpinBox(); self.dc_min.setRange(0, 65535); self.dc_min.setValue(0)
        self.dc_max = QtWidgets.QSpinBox(); self.dc_max.setRange(0, 65535); self.dc_max.setValue(65535)
        self.dc_points = QtWidgets.QSpinBox(); self.dc_points.setRange(2, 255); self.dc_points.setValue(100)
        self.dc_settle = QtWidgets.QSpinBox(); self.dc_settle.setRange(0, 10000); self.dc_settle.setValue(20); self.dc_settle.setSuffix(" ms")
        self.dc_average = QtWidgets.QSpinBox(); self.dc_average.setRange(1, 255); self.dc_average.setValue(1)
        self.dc_value = QtWidgets.QSpinBox(); self.dc_value.setRange(0, 65535); self.dc_value.setValue(32768)
        self.dc_start = QtWidgets.QPushButton("开始 DAC 扫描"); self.dc_start.clicked.connect(self.start_dc_scan)
        self.dc_abort = QtWidgets.QPushButton("终止扫描"); self.dc_abort.clicked.connect(self.abort_dc_scan)
        self.dc_set = QtWidgets.QPushButton("设置 DAC"); self.dc_set.clicked.connect(lambda: self.set_dc_dac(False))
        self.dc_save = QtWidgets.QPushButton("设置并保存 DAC"); self.dc_save.clicked.connect(lambda: self.set_dc_dac(True))
        self.dc_export = QtWidgets.QPushButton("导出扫描点"); self.dc_export.clicked.connect(self.export_dc_scan)
        fields = [("起点", self.dc_min), ("终点", self.dc_max), ("点数", self.dc_points),
                  ("等待", self.dc_settle), ("平均次数", self.dc_average), ("DAC 值", self.dc_value)]
        for column, (label, widget) in enumerate(fields):
            dc_grid.addWidget(QtWidgets.QLabel(label), 0, column)
            dc_grid.addWidget(widget, 1, column)
        dc_grid.addWidget(self.dc_start, 0, 6)
        dc_grid.addWidget(self.dc_abort, 1, 6)
        dc_grid.addWidget(self.dc_set, 0, 7)
        dc_grid.addWidget(self.dc_save, 1, 7)
        dc_grid.addWidget(self.dc_export, 0, 8, 2, 1)
        self.dc_controls.hide()
        layout.addWidget(self.dc_controls)

        action_row = QtWidgets.QHBoxLayout()
        self.auto_y = QtWidgets.QCheckBox("自动 Y 轴")
        self.auto_y.setChecked(True)
        self.auto_y.toggled.connect(self.on_auto_y)
        self.pause_button = QtWidgets.QPushButton("暂停显示")
        self.pause_button.setCheckable(True)
        self.pause_button.clicked.connect(self.toggle_pause)
        self.hold_data = QtWidgets.QCheckBox("保持全部数据")
        self.hold_data.setToolTip("勾选后不删除超出显示时间范围的历史数据")
        self.show_sensor_curve = QtWidgets.QCheckBox("显示传感器数据")
        self.show_sensor_curve.setChecked(True)
        self.show_sensor_curve.toggled.connect(self.mark_dirty)
        self.show_ac_curve = QtWidgets.QCheckBox("显示去直流波形")
        self.show_ac_curve.setChecked(True)
        self.show_ac_curve.toggled.connect(self.mark_dirty)
        self.save_button = QtWidgets.QPushButton("保存数据")
        self.save_button.clicked.connect(self.save_data)
        self.clear_button = QtWidgets.QPushButton("清空界面")
        self.clear_button.clicked.connect(self.clear_data)
        self.reset_view_button = QtWidgets.QPushButton("恢复视图")
        self.reset_view_button.clicked.connect(self.reset_view)
        for widget in (
            self.auto_y, self.pause_button, self.hold_data, self.show_sensor_curve,
            self.show_ac_curve, self.save_button, self.clear_button, self.reset_view_button
        ):
            action_row.addWidget(widget)
        action_row.addStretch(1)
        layout.addLayout(action_row)

        measurement_row = QtWidgets.QHBoxLayout()
        conversion = QtWidgets.QGroupBox("传感器测量换算")
        conversion_layout = QtWidgets.QGridLayout(conversion)
        self.sensitivity_input = QtWidgets.QDoubleSpinBox()
        self.sensitivity_input.setRange(-1_000_000_000_000.0, 1_000_000_000_000.0)
        self.sensitivity_input.setDecimals(9)
        self.sensitivity_input.setValue(1.0)
        self.sensitivity_input.setMinimumWidth(170)
        self.sensitivity_input.setToolTip("测量单位/采样值；允许输入正值或负值")
        self.offset_input = QtWidgets.QDoubleSpinBox()
        self.offset_input.setRange(-1_000_000_000_000.0, 1_000_000_000_000.0)
        self.offset_input.setDecimals(9)
        self.offset_input.setValue(0.0)
        self.offset_input.setMinimumWidth(170)
        self.offset_input.setToolTip("输出直流偏置；允许输入正值或负值")
        self.measurement_type = QtWidgets.QComboBox()
        self.measurement_type.addItems(["电场", "磁场", "电压", "电流"])
        self.measurement_unit = QtWidgets.QComboBox()
        self.measurement_units = {
            "电场": ["V/m", "kV/m", "kV/mm", "kV/cm"],
            "磁场": ["GS", "T"],
            "电压": ["V", "kV"],
            "电流": ["A", "kA"],
        }
        self.sensitivity_unit_label = QtWidgets.QLabel()
        self.offset_unit_label = QtWidgets.QLabel()
        self.measured_pp_label = QtWidgets.QLabel("传感器测量峰峰值：--")
        self.measured_ac_rms_label = QtWidgets.QLabel("传感器测量有效值：--")
        for label in (self.measured_pp_label, self.measured_ac_rms_label):
            label.setMinimumWidth(250)
            label.setStyleSheet("font-weight:600;color:#1d4ed8;")

        conversion_layout.addWidget(QtWidgets.QLabel("灵敏度"), 0, 0)
        conversion_layout.addWidget(self.sensitivity_input, 0, 1)
        conversion_layout.addWidget(self.sensitivity_unit_label, 0, 2)
        conversion_layout.addWidget(QtWidgets.QLabel("直流偏置"), 0, 3)
        conversion_layout.addWidget(self.offset_input, 0, 4)
        conversion_layout.addWidget(self.offset_unit_label, 0, 5)
        conversion_layout.addWidget(QtWidgets.QLabel("测量类型"), 1, 0)
        conversion_layout.addWidget(self.measurement_type, 1, 1)
        conversion_layout.addWidget(QtWidgets.QLabel("输出单位"), 1, 3)
        conversion_layout.addWidget(self.measurement_unit, 1, 4)
        conversion_layout.addWidget(self.measured_pp_label, 2, 0, 1, 3)
        conversion_layout.addWidget(self.measured_ac_rms_label, 2, 3, 1, 3)
        conversion_layout.setColumnStretch(6, 1)
        self.measurement_type.currentTextChanged.connect(self.update_measurement_units)
        self.measurement_unit.currentTextChanged.connect(self.on_conversion_changed)
        self.sensitivity_input.valueChanged.connect(self.on_conversion_changed)
        self.offset_input.valueChanged.connect(self.on_conversion_changed)
        self.update_measurement_units(self.measurement_type.currentText())

        stats = QtWidgets.QGroupBox("数据统计")
        stats_layout = QtWidgets.QGridLayout(stats)
        self.pp_label = QtWidgets.QLabel("峰峰值：--")
        self.rms_label = QtWidgets.QLabel("有效值：--")
        self.mean_label = QtWidgets.QLabel("平均值：--")
        self.ac_mean_label = QtWidgets.QLabel("去直流后平均值：--")
        self.ac_rms_label = QtWidgets.QLabel("去直流后有效值：--")
        self.range_label = QtWidgets.QLabel("最小/最大：-- / --")
        self.selection_label = QtWidgets.QLabel("统计范围：当前窗口")
        stats_layout.addWidget(self.pp_label, 0, 0)
        stats_layout.addWidget(self.rms_label, 0, 1)
        stats_layout.addWidget(self.mean_label, 1, 0)
        stats_layout.addWidget(self.ac_mean_label, 1, 1)
        stats_layout.addWidget(self.ac_rms_label, 2, 0)
        stats_layout.addWidget(self.range_label, 2, 1)
        stats_layout.addWidget(self.selection_label, 3, 0, 1, 2)
        stats_layout.setColumnStretch(0, 1)
        stats_layout.setColumnStretch(1, 1)
        measurement_row.addWidget(conversion, 3)
        measurement_row.addWidget(stats, 2)
        layout.addLayout(measurement_row)

        self.view_box = SelectionViewBox()
        self.plot = pg.PlotWidget(viewBox=self.view_box)
        self.plot.setLabel("bottom", "时间", units="s")
        self.plot.setLabel("left", "采样值")
        self.plot.showGrid(x=True, y=True, alpha=0.25)
        self.plot.addLegend(offset=(12, 12))
        self.curve = self.plot.plot([], [], pen=pg.mkPen("#22d3ee", width=1.4), name="传感器数据")
        self.ac_curve = self.plot.plot([], [], pen=pg.mkPen("#f59e0b", width=1.2), name="去直流偏置")
        self.show_sensor_curve.toggled.connect(self.curve.setVisible)
        self.show_ac_curve.toggled.connect(self.ac_curve.setVisible)
        self.cursor_v = pg.InfiniteLine(angle=90, movable=False, pen=pg.mkPen("#fbbf24", width=1))
        self.cursor_h = pg.InfiniteLine(angle=0, movable=False, pen=pg.mkPen("#fbbf24", width=1))
        self.cursor_text = pg.TextItem(color="#fde68a", anchor=(0, 1))
        for item in (self.cursor_v, self.cursor_h, self.cursor_text):
            item.hide()
            self.plot.addItem(item, ignoreBounds=True)
        self.region = pg.LinearRegionItem(brush=pg.mkBrush(59, 130, 246, 45))
        self.region.hide()
        self.region.sigRegionChanged.connect(self.update_selection_stats)
        self.plot.addItem(self.region)
        self.view_box.range_selected.connect(self.set_selection)
        self.view_box.point_clicked.connect(self.show_coordinate)
        self.dc_scan_plot = pg.PlotWidget()
        self.dc_scan_plot.setLabel("bottom", "DAC")
        self.dc_scan_plot.setLabel("left", "Lock-in")
        self.dc_scan_plot.showGrid(x=True, y=True, alpha=0.25)
        self.dc_scan_curve = self.dc_scan_plot.plot(
            [], [], pen=pg.mkPen("#22d3ee", width=1.5), symbol="o", symbolSize=7,
            symbolBrush=pg.mkBrush("#22d3ee"), symbolPen=pg.mkPen("#e0f2fe"),
        )
        self.dc_scan_curve.setCurveClickable(True, width=10)
        self.dc_scan_curve.sigPointsClicked.connect(self.on_dc_scan_point_clicked)
        self.dc_scan_v = pg.InfiniteLine(angle=90, movable=False, pen=pg.mkPen("#fbbf24", width=1))
        self.dc_scan_h = pg.InfiniteLine(angle=0, movable=False, pen=pg.mkPen("#fbbf24", width=1))
        self.dc_scan_text = pg.TextItem(color="#fde68a", anchor=(0, 1))
        for item in (self.dc_scan_v, self.dc_scan_h, self.dc_scan_text):
            item.hide()
            self.dc_scan_plot.addItem(item, ignoreBounds=True)
        self.dc_scan_info = QtWidgets.QLabel("等待扫描；点击数据点可查看坐标")
        self.dc_scan_info.setStyleSheet("font-weight:600;color:#1d4ed8;padding:4px;")
        dc_scan_page = QtWidgets.QWidget()
        dc_scan_layout = QtWidgets.QVBoxLayout(dc_scan_page)
        dc_scan_layout.setContentsMargins(0, 0, 0, 0)
        dc_scan_layout.addWidget(self.dc_scan_info)
        dc_scan_layout.addWidget(self.dc_scan_plot, 1)
        self.plot_tabs = QtWidgets.QTabWidget()
        self.plot_tabs.addTab(self.plot, "实时波形")
        self.plot_tabs.addTab(dc_scan_page, "DAC 扫描曲线")
        layout.addWidget(self.plot_tabs, 1)

        self.status_left = QtWidgets.QLabel("未连接")
        self.status_right = QtWidgets.QLabel("有效帧 0 | 接收 0 B | 丢弃 0 B")
        self.statusBar().addWidget(self.status_left, 1)
        self.statusBar().addPermanentWidget(self.status_right)
        self.ac_mode.toggled.connect(self.on_sensor_mode_changed)
        self.dc_mode.toggled.connect(self.on_sensor_mode_changed)
        self.apply_style()
        self.on_sensor_mode_changed()

    def apply_style(self):
        self.setStyleSheet("""
            QMainWindow, QWidget { background: #f8fafc; color: #172033; font-size: 13px; }
            QGroupBox { font-weight: 600; border: 1px solid #cbd5e1; border-radius: 7px;
                        margin-top: 9px; padding-top: 9px; }
            QGroupBox::title { subcontrol-origin: margin; left: 10px; padding: 0 5px; }
            QPushButton { background: #e2e8f0; border: 1px solid #b6c2d2; border-radius: 5px;
                          padding: 6px 13px; }
            QPushButton:hover { background: #cbd5e1; }
            QPushButton:checked { background: #2563eb; color: white; border-color: #1d4ed8; }
            QComboBox, QDoubleSpinBox { background: white; border: 1px solid #b6c2d2;
                                       border-radius: 4px; padding: 4px; min-height: 22px; }
            QStatusBar { background: #e2e8f0; }
        """)

    def on_sensor_mode_changed(self, _checked=False):
        is_dc = self.dc_mode.isChecked()
        if self.connect_button.isChecked():
            self.connect_button.setChecked(False)
            self.toggle_connection(False)
        self.dc_controls.setVisible(is_dc)
        self.data_format.setEnabled(not is_dc)
        self.sample_rate.setEnabled(not is_dc)
        self.show_ac_curve.setText("显示去直流波形" if not is_dc else "显示去均值波形")
        self.baud_combo.setCurrentText("115200" if is_dc else "921600")
        self.plot_tabs.setTabEnabled(1, is_dc)
        self.plot_tabs.setCurrentIndex(0)
        self.clear_data()
        self.status_left.setText("直流传感器模式" if is_dc else "交流传感器模式")

    @QtCore.pyqtSlot()
    def refresh_ports(self):
        current = self.port_combo.currentData()
        self.port_combo.clear()
        for item in list_ports.comports():
            label = f"{item.device} — {item.description}"
            self.port_combo.addItem(label, item.device)
        if current:
            index = self.port_combo.findData(current)
            if index >= 0:
                self.port_combo.setCurrentIndex(index)

    def toggle_connection(self, checked):
        if checked:
            port = self.port_combo.currentData()
            if not port:
                self.connect_button.setChecked(False)
                self.show_error("没有可用串口，请连接设备后刷新串口。")
                return
            try:
                baud = int(self.baud_combo.currentText().strip())
                if baud <= 0:
                    raise ValueError
            except ValueError:
                self.connect_button.setChecked(False)
                self.show_error("波特率必须是正整数。")
                return
            self.request_open.emit(port, baud)
        else:
            self.request_close.emit()

    def detect_baud(self):
        port = self.port_combo.currentData()
        if not port:
            self.show_error("没有可检测的串口。")
            return
        if self.connect_button.isChecked():
            self.show_error("请先断开串口，再执行自动检测。")
            return
        candidates = [921600, 460800, 230400, 115200, 1000000, 2000000, 57600, 38400, 19200, 9600]
        self.detect_baud_button.setEnabled(False)
        self.status_left.setText(f"正在检测 {port} 的波特率...")
        QtWidgets.QApplication.setOverrideCursor(QtCore.Qt.WaitCursor)
        found = None
        try:
            for baud in candidates:
                QtWidgets.QApplication.processEvents()
                probe = None
                try:
                    probe = serial.Serial(port, baud, timeout=0.02)
                    probe.reset_input_buffer()
                    parser = DcFrameParser() if self.dc_mode.isChecked() else FrameParser()
                    deadline = QtCore.QElapsedTimer()
                    deadline.start()
                    while deadline.elapsed() < 550:
                        count = probe.in_waiting
                        if count and parser.feed(probe.read(count)):
                            found = baud
                            break
                        QtCore.QThread.msleep(5)
                        QtWidgets.QApplication.processEvents()
                except Exception:
                    pass
                finally:
                    if probe is not None:
                        probe.close()
                if found is not None:
                    break
        finally:
            QtWidgets.QApplication.restoreOverrideCursor()
            self.detect_baud_button.setEnabled(True)
        if found is None:
            protocol = "直流协议帧" if self.dc_mode.isChecked() else "合法246字节帧"
            self.status_left.setText(f"未检测到{protocol}")
            self.show_error("未检测到合法帧。请确认传感器类型选择正确且接收器正在发送数据。")
        else:
            self.baud_combo.setCurrentText(str(found))
            self.status_left.setText(f"检测成功：{port} @ {found}")

    @QtCore.pyqtSlot(bool, str)
    def on_serial_state(self, connected, text):
        self.connect_button.blockSignals(True)
        self.connect_button.setChecked(connected)
        self.connect_button.setText("断开" if connected else "连接")
        self.connect_button.blockSignals(False)
        self.port_combo.setEnabled(not connected)
        self.baud_combo.setEnabled(not connected)
        self.status_left.setText(("已连接：" if connected else "") + text)

    @QtCore.pyqtSlot(bytes)
    def on_bytes(self, chunk):
        self.total_bytes += len(chunk)
        if self.dc_mode.isChecked():
            self.on_dc_bytes(chunk)
            return
        frames = self.parser.feed(chunk)
        if not frames:
            self.update_status()
            return
        dtype = ">u2" if self.data_format.currentIndex() == 0 else ">i2"
        for frame in frames:
            # Sensor payload bytes [3:243]: uint16, MSB first (big-endian).
            values = np.frombuffer(frame[3:243], dtype=dtype, count=SAMPLES_PER_FRAME)
            if np.any(values):
                self.nonzero_frames += 1
            else:
                self.zero_frames += 1
            self.sample_chunks.append(values.astype(np.float64))
            self.sample_count += SAMPLES_PER_FRAME
            self.total_samples += SAMPLES_PER_FRAME
        self.trim_buffer()
        self.plot_dirty = True
        self.update_status()

    def on_dc_bytes(self, chunk):
        frames = self.dc_parser.feed(chunk)
        for kind, _flags, _scan_id, _seq, payload in frames:
            if kind == DC_EVT_SCAN_BEGIN:
                self.dc_scan_points.clear()
                self.dc_scan_curve.setData([], [])
                self.clear_dc_scan_selection()
                self.dc_scan_info.setText(f"扫描进行中：0 / {self.dc_points.value()} 点")
                self.status_left.setText("DAC 扫描开始")
            elif kind == DC_EVT_SCAN_POINTS and payload:
                offset = 1
                for _ in range(payload[0]):
                    if offset + 5 > len(payload):
                        break
                    index = payload[offset]
                    dac, lockin = struct.unpack_from("<HH", payload, offset + 1)
                    self.dc_scan_points.append((index, dac, lockin))
                    offset += 5
                self.refresh_dc_scan_plot()
            elif kind == DC_EVT_SCAN_END:
                self.status_left.setText(f"DAC 扫描完成，共 {len(self.dc_scan_points)} 点")
                self.dc_scan_info.setText(f"扫描完成：{len(self.dc_scan_points)} 点；点击数据点可查看坐标")
            elif kind == DC_EVT_SENSOR_READINGS and payload:
                offset = 1
                now = time.time()
                for sample_index in range(payload[0]):
                    if offset + 8 > len(payload):
                        break
                    seq, dac, lockin, adc = struct.unpack_from("<HHHH", payload, offset)
                    timestamp = now + sample_index * 1e-6
                    if self.dc_start_time is None:
                        self.dc_start_time = timestamp
                    self.dc_rows.append((timestamp, seq, dac, lockin, adc))
                    self.total_samples += 1
                    offset += 8
        self.trim_buffer()
        self.plot_dirty = bool(frames) or self.plot_dirty
        self.update_status()

    def send_dc(self, frame_type, payload=b"", flags=0):
        if not self.connect_button.isChecked():
            self.show_error("请先连接直流传感器串口。")
            return False
        self.request_write.emit(dc_encode(frame_type, flags, 1, self.dc_seq, payload))
        self.dc_seq = (self.dc_seq + 1) & 0xFFFF
        return True

    def start_dc_scan(self):
        if self.dc_min.value() > self.dc_max.value():
            self.show_error("DAC 起点不能大于终点。")
            return
        payload = struct.pack("<HHHHB", self.dc_min.value(), self.dc_max.value(),
                              self.dc_points.value(), self.dc_settle.value(), self.dc_average.value())
        if self.send_dc(DC_CMD_SCAN_START, payload):
            self.dc_scan_points.clear()
            self.dc_scan_curve.setData([], [])
            self.clear_dc_scan_selection()
            self.dc_scan_info.setText(f"扫描命令已发送：0 / {self.dc_points.value()} 点")
            self.plot_tabs.setCurrentIndex(1)
            self.status_left.setText("已发送 DAC 扫描命令")

    def refresh_dc_scan_plot(self):
        if not self.dc_scan_points:
            return
        x = np.asarray([point[1] for point in self.dc_scan_points], dtype=np.float64)
        y = np.asarray([point[2] for point in self.dc_scan_points], dtype=np.float64)
        self.dc_scan_curve.setData(x, y, data=list(range(len(self.dc_scan_points))))
        # Explicitly grow the visible range as scan points arrive. This also
        # handles descending DAC scans and a single first point cleanly.
        x_pad = max(1.0, float(np.ptp(x)) * 0.04)
        y_pad = max(1.0, float(np.ptp(y)) * 0.08)
        self.dc_scan_plot.setXRange(float(np.min(x) - x_pad), float(np.max(x) + x_pad), padding=0)
        self.dc_scan_plot.setYRange(float(np.min(y) - y_pad), float(np.max(y) + y_pad), padding=0)
        received = len(self.dc_scan_points)
        self.dc_scan_info.setText(f"扫描进行中：{received} / {self.dc_points.value()} 点；坐标范围随数据更新")

    def on_dc_scan_point_clicked(self, _curve, points, _event=None):
        if not points:
            return
        point = points[0]
        position = point.pos()
        dac, lockin = float(position.x()), float(position.y())
        index = int(point.data()) if point.data() is not None else -1
        scan_index = self.dc_scan_points[index][0] if 0 <= index < len(self.dc_scan_points) else index
        self.dc_scan_v.setPos(dac)
        self.dc_scan_h.setPos(lockin)
        self.dc_scan_text.setText(f"序号 = {scan_index}\nDAC = {dac:.0f}\nLock-in = {lockin:.0f}")
        self.dc_scan_text.setPos(dac, lockin)
        for item in (self.dc_scan_v, self.dc_scan_h, self.dc_scan_text):
            item.show()
        self.dc_scan_info.setText(f"已选择：序号 {scan_index}，DAC = {dac:.0f}，Lock-in = {lockin:.0f}")

    def clear_dc_scan_selection(self):
        for item in (self.dc_scan_v, self.dc_scan_h, self.dc_scan_text):
            item.hide()

    def abort_dc_scan(self):
        if self.send_dc(DC_CMD_ABORT):
            self.status_left.setText("已发送终止扫描命令")

    def set_dc_dac(self, save):
        if self.send_dc(DC_CMD_SET_DAC, struct.pack("<H", self.dc_value.value()), 1 if save else 0):
            self.status_left.setText(f"已设置 DAC={self.dc_value.value()}" + ("（保存）" if save else ""))

    def export_dc_scan(self):
        if not self.dc_scan_points:
            self.show_error("当前没有 DAC 扫描点可导出。")
            return
        default = f"dac_scan_{datetime.now():%Y%m%d_%H%M%S}.csv"
        path, _ = QtWidgets.QFileDialog.getSaveFileName(self, "导出 DAC 扫描点", str(Path.home() / default), "CSV 文件 (*.csv)")
        if not path:
            return
        with open(path, "w", newline="", encoding="utf-8-sig") as handle:
            writer = csv.writer(handle)
            writer.writerow(["index", "dac", "lockin"])
            writer.writerows(self.dc_scan_points)
        self.statusBar().showMessage(f"扫描点已保存：{path}", 6000)

    def trim_buffer(self):
        if self.hold_data.isChecked() or self.show_all_data():
            return
        if self.dc_mode.isChecked():
            cutoff = time.time() - self.duration_seconds()
            while self.dc_rows and self.dc_rows[0][0] < cutoff:
                self.dc_rows.popleft()
            return
        keep = max(SAMPLES_PER_FRAME, int(self.duration_seconds() * self.sample_rate.value()))
        while self.sample_chunks and self.sample_count - len(self.sample_chunks[0]) >= keep:
            removed = self.sample_chunks.popleft()
            self.sample_count -= len(removed)

    def get_data(self):
        if self.dc_mode.isChecked():
            return np.asarray([row[3] for row in self.dc_rows], dtype=np.float64)
        if not self.sample_chunks:
            return np.empty(0)
        return np.concatenate(tuple(self.sample_chunks))

    def refresh_plot(self):
        if not self.plot_dirty or self.paused:
            return
        data = self.get_data()
        if data.size == 0:
            return
        if self.dc_mode.isChecked():
            rows = list(self.dc_rows)
            duration = self.duration_seconds()
            if duration is not None:
                cutoff = time.time() - duration
                rows = [row for row in rows if row[0] >= cutoff]
            if not rows:
                return
            # Use forward elapsed time from the beginning of this acquisition.
            # A rolling window changes which samples are visible, but never makes
            # the time axis negative or reverse its direction.
            base = self.dc_start_time if self.dc_start_time is not None else rows[0][0]
            time_scale, time_unit, _decimals = self.time_display_settings()
            x = np.asarray([(row[0] - base) * time_scale for row in rows], dtype=np.float64)
            visible = np.asarray([row[3] for row in rows], dtype=np.float64)
            self.last_x, self.last_y = x, visible
            self.plot.setLabel("bottom", "时间", units=time_unit)
            self.curve.setData(x, visible, skipFiniteCheck=True)
            self.ac_curve.setData(x, visible - np.mean(visible), skipFiniteCheck=True)
            self.curve.setVisible(self.show_sensor_curve.isChecked())
            self.ac_curve.setVisible(self.show_ac_curve.isChecked())
            self.plot.setXRange(float(x[0]), float(x[-1]) if x.size > 1 else float(x[0] + 1), padding=0)
            if self.auto_y.isChecked():
                self.plot.enableAutoRange(axis=pg.ViewBox.YAxis, enable=True)
            self.update_stats(visible, "当前窗口")
            self.plot_dirty = False
            return
        rate = self.sample_rate.value()
        duration = self.duration_seconds()
        if duration is None:
            visible = data
        else:
            duration_samples = max(1, int(duration * rate))
            visible = data[-duration_samples:]
        end_sample = self.total_samples
        start_sample = end_sample - visible.size
        time_scale, time_unit, _decimals = self.time_display_settings()
        x = np.arange(start_sample, end_sample, dtype=np.float64) / rate * time_scale
        self.last_x, self.last_y = x, visible
        self.plot.setLabel("bottom", "时间", units=time_unit)
        self.curve.setData(x, visible, skipFiniteCheck=True)
        self.curve.setVisible(self.show_sensor_curve.isChecked())
        if self.show_ac_curve.isChecked():
            self.ac_curve.setData(x, visible - np.mean(visible), skipFiniteCheck=True)
            self.ac_curve.show()
        else:
            self.ac_curve.hide()
        self.plot.setXRange(
            float(x[0]),
            float(x[-1]) if x.size > 1 else float(x[0] + time_scale / rate),
            padding=0,
        )
        if self.auto_y.isChecked():
            self.plot.enableAutoRange(axis=pg.ViewBox.YAxis, enable=True)
        self.update_stats(visible, "当前窗口")
        self.plot_dirty = False

    def toggle_pause(self, checked):
        self.paused = checked
        self.pause_button.setText("继续显示" if checked else "暂停显示")
        self.view_box.selection_enabled = checked
        if checked:
            self.selection_label.setText("统计范围：当前窗口；左键拖动可框选")
        else:
            self.region.hide()
            self.selection_label.setText("统计范围：当前窗口")
            self.plot_dirty = True

    def set_selection(self, x1, x2):
        if not self.paused:
            return
        self.region.setRegion((x1, x2))
        self.region.show()
        self.update_selection_stats()

    def update_selection_stats(self):
        if not self.region.isVisible() or self.last_x.size == 0:
            return
        x1, x2 = self.region.getRegion()
        mask = (self.last_x >= x1) & (self.last_x <= x2)
        values = self.last_y[mask]
        _scale, unit, decimals = self.time_display_settings()
        self.update_stats(
            values,
            f"选区 {x1:.{decimals}f}～{x2:.{decimals}f} {unit}（{values.size}点）",
        )

    def update_stats(self, values, scope):
        self.last_stats_values = np.asarray(values, dtype=np.float64).copy()
        self.last_stats_scope = scope
        if values.size == 0:
            pp = rms = mean = ac_mean = ac_rms = minimum = maximum = "--"
        else:
            pp = f"{int(np.ptp(values))}"
            rms = f"{np.sqrt(np.mean(np.square(values))):.3f}"
            mean_value = np.mean(values)
            mean = f"{mean_value:.3f}"
            ac_values = values - mean_value
            ac_mean = f"{np.mean(ac_values):.3f}"
            ac_rms = f"{np.sqrt(np.mean(np.square(ac_values))):.3f}"
            minimum = f"{int(np.min(values))}"
            maximum = f"{int(np.max(values))}"
        self.pp_label.setText(f"峰峰值：{pp}")
        self.rms_label.setText(f"有效值：{rms}")
        self.mean_label.setText(f"平均值：{mean}")
        self.ac_mean_label.setText(f"去直流后平均值：{ac_mean}")
        self.ac_rms_label.setText(f"去直流后有效值：{ac_rms}")
        self.range_label.setText(f"最小/最大：{minimum} / {maximum}")
        self.selection_label.setText(f"统计范围：{scope}")
        self.update_measurement_values(values)

    def update_measurement_units(self, measurement_type):
        current_unit = self.measurement_unit.currentText()
        self.measurement_unit.blockSignals(True)
        self.measurement_unit.clear()
        self.measurement_unit.addItems(self.measurement_units.get(measurement_type, []))
        index = self.measurement_unit.findText(current_unit)
        if index >= 0:
            self.measurement_unit.setCurrentIndex(index)
        self.measurement_unit.blockSignals(False)
        self.on_conversion_changed()

    def on_conversion_changed(self, _value=None):
        unit = self.measurement_unit.currentText()
        self.sensitivity_unit_label.setText(f"{unit}/采样值")
        self.offset_unit_label.setText(unit)
        self.update_measurement_values(self.last_stats_values)

    def update_measurement_values(self, values):
        unit = self.measurement_unit.currentText()
        if values.size == 0 or not unit:
            measured_pp = measured_ac_rms = "--"
        else:
            sensitivity = self.sensitivity_input.value()
            offset = self.offset_input.value()
            raw_pp = float(np.ptp(values))
            ac_values = values - np.mean(values)
            raw_ac_rms = float(np.sqrt(np.mean(np.square(ac_values))))
            measured_pp = self.format_measurement(sensitivity * raw_pp + offset)
            measured_ac_rms = self.format_measurement(sensitivity * raw_ac_rms + offset)
        suffix = f" {unit}" if unit and measured_pp != "--" else ""
        self.measured_pp_label.setText(f"传感器测量峰峰值：{measured_pp}{suffix}")
        self.measured_ac_rms_label.setText(
            f"传感器测量有效值：{measured_ac_rms}{suffix}"
        )

    @staticmethod
    def format_measurement(value):
        return f"{value:.9g}"

    def show_coordinate(self, x, _y):
        if self.last_x.size == 0:
            return
        index = int(np.argmin(np.abs(self.last_x - x)))
        px, py = float(self.last_x[index]), float(self.last_y[index])
        self.cursor_v.setPos(px)
        self.cursor_h.setPos(py)
        _scale, unit, decimals = self.time_display_settings()
        self.cursor_text.setText(
            f"时间 = {px:.{decimals}f} {unit}\n十进制值 = {int(round(py))}"
        )
        self.cursor_text.setPos(px, py)
        for item in (self.cursor_v, self.cursor_h, self.cursor_text):
            item.show()

    def on_auto_y(self, checked):
        self.plot.enableAutoRange(axis=pg.ViewBox.YAxis, enable=checked)
        if checked:
            self.plot.autoRange()

    def reset_view(self):
        self.plot.autoRange()
        if self.last_x.size:
            self.plot.setXRange(float(self.last_x[0]), float(self.last_x[-1]), padding=0)

    def on_duration_changed(self):
        _scale, unit, _decimals = self.time_display_settings()
        self.plot.setLabel("bottom", "时间", units=unit)
        if self.show_all_data():
            self.mark_dirty()
            return
        try:
            if self.duration_seconds() <= 0:
                raise ValueError
        except ValueError:
            return
        self.trim_buffer()
        self.mark_dirty()

    def duration_seconds(self):
        if self.show_all_data():
            return None
        try:
            text = self.duration_combo.currentText().strip().lower()
            if text.endswith("ms"):
                value = float(text[:-2].strip()) / 1000.0
            elif text.endswith("毫秒"):
                value = float(text[:-2].strip()) / 1000.0
            elif text.endswith("min"):
                value = float(text[:-3].strip()) * 60.0
            elif text.endswith("分钟"):
                value = float(text[:-2].strip()) * 60.0
            elif text.endswith("h"):
                value = float(text[:-1].strip()) * 3600.0
            elif text.endswith("小时"):
                value = float(text[:-2].strip()) * 3600.0
            elif text.endswith("s"):
                value = float(text[:-1].strip())
            elif text.endswith("秒"):
                value = float(text[:-1].strip())
            else:
                # 兼容旧版的无单位自定义输入：默认按秒解析。
                value = float(text)
            return max(0.001, value)
        except ValueError:
            return 5.0

    def time_display_settings(self):
        """返回时间轴的秒换算系数、单位和显示小数位。"""
        text = self.duration_combo.currentText().strip().lower()
        if text.endswith("ms") or text.endswith("毫秒"):
            return 1000.0, "ms", 3
        if text.endswith("min") or text.endswith("分钟"):
            return 1.0 / 60.0, "min", 4
        if text.endswith("h") or text.endswith("小时"):
            return 1.0 / 3600.0, "h", 5
        return 1.0, "s", 6

    def show_all_data(self):
        return self.duration_combo.currentText().strip() in ("全部", "all", "All", "ALL")

    def mark_dirty(self):
        self.plot_dirty = True

    def clear_data(self):
        self.parser.clear()
        self.dc_parser.clear()
        self.sample_chunks.clear()
        self.dc_rows.clear()
        self.dc_scan_points.clear()
        self.dc_start_time = None
        self.sample_count = 0
        self.total_samples = 0
        self.total_bytes = 0
        self.zero_frames = 0
        self.nonzero_frames = 0
        self.last_x = np.empty(0)
        self.last_y = np.empty(0)
        self.curve.setData([], [])
        self.ac_curve.setData([], [])
        self.dc_scan_curve.setData([], [])
        self.clear_dc_scan_selection()
        self.dc_scan_info.setText("等待扫描；点击数据点可查看坐标")
        self.region.hide()
        for item in (self.cursor_v, self.cursor_h, self.cursor_text):
            item.hide()
        self.update_stats(np.empty(0), "当前窗口")
        self.update_status()

    def save_data(self):
        data = self.get_data()
        if data.size == 0:
            self.show_error("当前没有可保存的数据。")
            return
        prefix = "dc_sensor" if self.dc_mode.isChecked() else "ac_sensor"
        default = f"{prefix}_data_{datetime.now():%Y%m%d_%H%M%S}.csv"
        path, _ = QtWidgets.QFileDialog.getSaveFileName(
            self, "保存数据", str(Path.home() / default), "CSV 文件 (*.csv)"
        )
        if not path:
            return
        rate = self.sample_rate.value()
        start_index = self.total_samples - data.size
        try:
            with open(path, "w", newline="", encoding="utf-8-sig") as handle:
                writer = csv.writer(handle)
                if self.dc_mode.isChecked():
                    writer.writerow(["timestamp", "elapsed_s", "seq", "dac", "lockin", "adc"])
                    rows = list(self.dc_rows)
                    start = rows[0][0]
                    for timestamp, seq, dac, lockin, adc in rows:
                        writer.writerow([f"{timestamp:.6f}", f"{timestamp-start:.6f}", seq, dac, lockin, adc])
                else:
                    writer.writerow(["sample_index", "time_s", "value"])
                    for offset, value in enumerate(data):
                        index = start_index + offset
                        writer.writerow([index, f"{index / rate:.9f}", str(int(round(value)))])
            self.statusBar().showMessage(f"数据已保存：{path}", 6000)
        except Exception as exc:
            self.show_error(f"保存失败：{exc}")

    def update_status(self):
        if self.dc_mode.isChecked():
            self.status_right.setStyleSheet("")
            self.status_right.setText(
                f"有效帧 {self.dc_parser.valid_frames} | 接收 {self.total_bytes:,} B | "
                f"实时读数 {len(self.dc_rows):,} | 扫描点 {len(self.dc_scan_points):,} | "
                f"丢弃 {self.dc_parser.discarded_bytes:,} B"
            )
            return
        text = (
            f"有效帧 {self.parser.valid_frames} | 接收 {self.total_bytes:,} B | "
            f"非零帧 {self.nonzero_frames} | 全零帧 {self.zero_frames} | "
            f"丢弃 {self.parser.discarded_bytes:,} B"
        )
        if self.zero_frames >= 3 and self.nonzero_frames == 0:
            text += " | 警告：串口数据区为全0，请检查传感器ADC输出/接线"
            self.status_right.setStyleSheet("color:#dc2626;font-weight:600;")
        else:
            self.status_right.setStyleSheet("")
        self.status_right.setText(text)

    def show_error(self, message):
        QtWidgets.QMessageBox.warning(self, APP_NAME, message)

    def closeEvent(self, event):
        self.request_close.emit()
        self.worker_thread.quit()
        self.worker_thread.wait(1500)
        event.accept()


def main():
    QtWidgets.QApplication.setAttribute(QtCore.Qt.AA_EnableHighDpiScaling, True)
    QtWidgets.QApplication.setAttribute(QtCore.Qt.AA_UseHighDpiPixmaps, True)
    app = QtWidgets.QApplication(sys.argv)
    app.setApplicationName(APP_NAME)
    app.setFont(QtGui.QFont("Microsoft YaHei UI", 9))
    window = MainWindow()
    window.show()
    sys.exit(app.exec_())


if __name__ == "__main__":
    main()
