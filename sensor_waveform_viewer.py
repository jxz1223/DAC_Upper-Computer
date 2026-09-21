import csv
import secrets
import struct
import sys
import time
from collections import deque
from dataclasses import dataclass, field
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

MULTI_VERSION = 2
MULTI_MAX_PAYLOAD = 232
MULTI_NODE_RELAY = 0x0000
MULTI_NODE_BROADCAST = 0xFFFF
MULTI_CMD_SCAN_START = 0x10
MULTI_CMD_SET_DAC = 0x11
MULTI_CMD_ABORT = 0x12
MULTI_CMD_RELEASE_LINK = 0x16
MULTI_CMD_RELAY_DISCHARGE = 0x17
MULTI_RELAY_ACK_TIMEOUT_S = 4.0
MULTI_RELAY_STATE_TIMEOUT_S = 8.0
MULTI_SET_DAC_FLAG_SAVE = 0x04
MULTI_FLAG_LOCKIN_CENTI = 0x08
MULTI_EVT_ACK = 0x80
MULTI_EVT_NACK = 0x81
MULTI_EVT_SCAN_BEGIN = 0x90
MULTI_EVT_SCAN_POINTS = 0x91
MULTI_EVT_SCAN_END = 0x92
MULTI_EVT_SENSOR_READINGS = 0x93
MULTI_EVT_RELAY_STATE = 0x94
MULTI_EVT_RAW_SAMPLES = 0xA0
MULTI_EVT_LINK_STATUS = 0xA1
MULTI_EVT_RELAY_STATS = 0xA2
MULTI_EVT_COMMAND_TRACE = 0xA3
MULTI_MAX_NODES = 8
MULTI_NODE_COLORS = (
    "#22d3ee", "#f472b6", "#a3e635", "#fb923c",
    "#a78bfa", "#facc15", "#60a5fa", "#f87171",
)
MULTI_GATT_STAGE_NAMES = {
    0: "无",
    1: "ATT MTU 协商",
    2: "主服务发现",
    3: "特征发现",
    4: "通知描述符发现",
    5: "使能通知",
    6: "GATT 已就绪",
}
MULTI_HCI_REASON_NAMES = {
    0x05: "认证失败",
    0x08: "连接监控超时",
    0x13: "远端设备主动终止",
    0x14: "远端资源不足",
    0x15: "远端设备关机",
    0x16: "本地主机主动终止",
    0x22: "链路层响应超时",
    0x3B: "连接参数不可接受",
    0x3E: "连接建立失败",
}
MULTI_GATT_ERROR_NAMES = {
    0x0A: "属性未找到",
    0x64: "协议栈资源暂时不足",
    0x91: "BLE 操作失败",
    0x92: "GATT 参数无效",
    0x93: "HCI/GATT 命令忙",
    0xE1: "未找到目标主服务",
    0xE2: "未找到收发特征",
    0xE3: "未找到通知 CCCD",
    0xE4: "GATT 发现阶段超时",
    0xE5: "DAC 扫描业务超时",
    0xE6: "协商后的 ATT MTU 无法承载扫描帧",
    0xFF: "HCI 命令响应超时",
}
MULTI_COMMAND_NAMES = {
    MULTI_CMD_SCAN_START: "开始 DAC 扫描",
    MULTI_CMD_SET_DAC: "设置 DAC",
    MULTI_CMD_ABORT: "终止 DAC 扫描",
    MULTI_CMD_RELEASE_LINK: "断开并腾出位置",
    MULTI_CMD_RELAY_DISCHARGE: "继电器放电",
}
MULTI_COMMAND_TRACE_NAMES = {
    0x00: "中继 UART 已收到并校验完整命令帧",
    0x01: "中继已解析串口命令并加入 BLE 队列",
    0x02: "中继 BLE 写入接口已接受命令",
    0x03: "中继已收到传感器事件",
    0x04: "中继已收到传感器 ACK",
    0x05: "中继已收到传感器 NACK",
    0x06: "中继收到的传感器帧无效",
    0x07: "中继正在重发命令",
    0x08: "中继等待传感器确认超时",
    0x09: "中继 BLE 写入失败",
}
MULTI_STATUS_NAMES = {
    0x00: "成功",
    0x01: "CRC 校验失败",
    0x02: "协议版本错误",
    0x03: "帧长度或参数错误",
    0x04: "未知命令",
    0x05: "没有可用链路",
    0x06: "设备忙",
    0x07: "不支持",
    0x08: "接收缓冲区溢出",
    0x09: "命令确认超时",
    0x0A: "队列已满",
    0x0B: "未找到目标节点",
    0x0C: "目标链路尚未就绪",
    0x0D: "中继到传感器的 BLE 写入失败",
}


def dc_crc16(data):
    crc = 0xFFFF
    for byte in data:
        crc ^= byte << 8
        for _ in range(8):
            crc = ((crc << 1) ^ 0x1021) & 0xFFFF if crc & 0x8000 else (crc << 1) & 0xFFFF
    return crc


def decode_counted_records(payload, record_format):
    """Reject an entire malformed batch before any readings reach the UI."""
    record_size = struct.calcsize(record_format)
    if not payload or len(payload) != 1 + payload[0] * record_size:
        raise ValueError("点数与载荷长度不匹配")
    return struct.iter_unpack(record_format, payload[1:])


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


@dataclass(frozen=True)
class MultiFrame:
    kind: int
    flags: int
    node_id: int
    scan_id: int
    seq: int
    payload: bytes


def multi_encode(frame_type, node_id, flags=0, scan_id=1, seq=1, payload=b""):
    if not 0 <= node_id <= 0xFFFF:
        raise ValueError("NodeId 超出 uint16 范围")
    if len(payload) > MULTI_MAX_PAYLOAD:
        raise ValueError(f"一对多协议负载不能超过 {MULTI_MAX_PAYLOAD} 字节")
    header = struct.pack(
        "<BBBHHHB", MULTI_VERSION, frame_type, flags, node_id, scan_id, seq, len(payload)
    )
    return DC_MAGIC + header + payload + struct.pack("<H", dc_crc16(header + payload))


class MultiFrameParser:
    """Parser for the relay V2 protocol carrying an explicit uint16 NodeId."""

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
            pos = self.buffer.find(DC_MAGIC)
            if pos < 0:
                keep = 1 if self.buffer and self.buffer[-1] == DC_MAGIC[0] else 0
                self.discarded_bytes += len(self.buffer) - keep
                self.buffer[:] = self.buffer[-keep:] if keep else b""
                break
            if pos:
                del self.buffer[:pos]
                self.discarded_bytes += pos
            if len(self.buffer) < 14:
                break
            version, kind, flags, node_id, scan_id, seq, size = struct.unpack_from(
                "<BBBHHHB", self.buffer, 2
            )
            if size > MULTI_MAX_PAYLOAD:
                del self.buffer[0]
                self.discarded_bytes += 1
                continue
            total = 14 + size
            if len(self.buffer) < total:
                break
            payload = bytes(self.buffer[12:12 + size])
            received = struct.unpack_from("<H", self.buffer, 12 + size)[0]
            calculated = dc_crc16(bytes(self.buffer[2:12 + size]))
            del self.buffer[:total]
            if version == MULTI_VERSION and received == calculated:
                frames.append(MultiFrame(kind, flags, node_id, scan_id, seq, payload))
                self.valid_frames += 1
            else:
                self.discarded_bytes += total
        return frames


@dataclass
class MultiNodeData:
    node_id: int
    color_index: int
    online: bool = False
    link_state: int = 0
    link_stage: int = 0
    hci_reason: int = 0
    gatt_error: int = 0
    att_mtu: int = 23
    address: str = "--"
    frames: int = 0
    last_seen: float = 0.0
    rows: deque = field(default_factory=deque)
    scan_points: list = field(default_factory=list)
    scan_expected: int = 0
    scan_state: str = "idle"
    scan_id: int = 0
    relay_level: int | None = None
    relay_phase: str = "unknown"
    relay_pending_seq: int | None = None
    relay_pending_scan_id: int | None = None
    relay_deadline: float = 0.0
    relay_detail: str = ""


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
    bytes_written = QtCore.pyqtSignal(int)
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
            written = self.port.write(data)
            self.bytes_written.emit(int(written or 0))
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


@dataclass(frozen=True)
class DebugLogEntry:
    timestamp: str
    level: str
    category: str
    node: str
    message: str
    node_color: str = ""


class DebugLogWindow(QtWidgets.QDialog):
    MAX_ENTRIES = 5000
    LEVEL_COLORS = {
        "信息": "#334155",
        "确认": "#15803d",
        "进度": "#0369a1",
        "连接": "#15803d",
        "发现": "#b45309",
        "设备": "#0369a1",
        "警告": "#b45309",
        "错误": "#b91c1c",
    }

    def __init__(self, parent=None):
        super().__init__(parent)
        self.setWindowTitle("蓝牙调试日志")
        self.resize(980, 620)
        self.setMinimumSize(760, 460)
        self.setAttribute(QtCore.Qt.WA_DeleteOnClose, False)
        self.entries = deque(maxlen=self.MAX_ENTRIES)
        self.build_ui()

    def build_ui(self):
        layout = QtWidgets.QVBoxLayout(self)
        layout.setContentsMargins(10, 10, 10, 10)
        layout.setSpacing(8)

        banner = QtWidgets.QFrame()
        banner.setObjectName("logBanner")
        banner_layout = QtWidgets.QHBoxLayout(banner)
        banner_layout.setContentsMargins(14, 9, 14, 9)
        title = QtWidgets.QLabel("蓝牙事件终端")
        title.setObjectName("logTitle")
        detail = QtWidgets.QLabel("串口 → 中继 → GATT → 传感器")
        detail.setObjectName("logDetail")
        self.log_count_label = QtWidgets.QLabel("0 条")
        self.log_count_label.setObjectName("logCount")
        banner_layout.addWidget(title)
        banner_layout.addSpacing(12)
        banner_layout.addWidget(detail)
        banner_layout.addStretch(1)
        banner_layout.addWidget(self.log_count_label)
        layout.addWidget(banner)

        toolbar = QtWidgets.QHBoxLayout()
        toolbar.addWidget(QtWidgets.QLabel("显示："))
        self.filter_combo = QtWidgets.QComboBox()
        self.filter_combo.addItem("全部事件", "all")
        self.filter_combo.addItem("连接事件", "link")
        self.filter_combo.addItem("协议事件", "protocol")
        self.filter_combo.addItem("设备诊断", "device")
        self.filter_combo.addItem("警告与错误", "problem")
        self.filter_combo.currentIndexChanged.connect(self.rebuild_table)
        toolbar.addWidget(self.filter_combo)
        self.auto_scroll = QtWidgets.QCheckBox("自动滚动")
        self.auto_scroll.setChecked(True)
        toolbar.addWidget(self.auto_scroll)
        toolbar.addStretch(1)
        self.pause_button = QtWidgets.QPushButton("暂停刷新")
        self.pause_button.setCheckable(True)
        self.pause_button.toggled.connect(self.on_pause_changed)
        copy_button = QtWidgets.QPushButton("复制可见日志")
        copy_button.clicked.connect(self.copy_visible)
        save_button = QtWidgets.QPushButton("保存日志")
        save_button.clicked.connect(self.save_log)
        clear_button = QtWidgets.QPushButton("清空")
        clear_button.clicked.connect(self.clear_log)
        for widget in (self.pause_button, copy_button, save_button, clear_button):
            toolbar.addWidget(widget)
        layout.addLayout(toolbar)

        self.table = QtWidgets.QTableWidget(0, 4)
        self.table.setHorizontalHeaderLabels(["时间", "级别", "节点", "事件"])
        self.table.setEditTriggers(QtWidgets.QAbstractItemView.NoEditTriggers)
        self.table.setSelectionBehavior(QtWidgets.QAbstractItemView.SelectRows)
        self.table.setAlternatingRowColors(True)
        self.table.verticalHeader().setVisible(False)
        header = self.table.horizontalHeader()
        header.setSectionResizeMode(0, QtWidgets.QHeaderView.Fixed)
        header.setSectionResizeMode(1, QtWidgets.QHeaderView.Fixed)
        header.setSectionResizeMode(2, QtWidgets.QHeaderView.Fixed)
        header.setSectionResizeMode(3, QtWidgets.QHeaderView.Stretch)
        self.table.setColumnWidth(0, 112)
        self.table.setColumnWidth(1, 66)
        self.table.setColumnWidth(2, 142)
        layout.addWidget(self.table, 1)

        self.empty_label = QtWidgets.QLabel("等待事件。连接数据中继后，蓝牙状态变化会显示在这里。")
        self.empty_label.setObjectName("logEmpty")
        self.empty_label.setAlignment(QtCore.Qt.AlignCenter)
        layout.addWidget(self.empty_label)

        self.setStyleSheet("""
            QDialog { background:#f8fafc; color:#172033; font-size:13px; }
            QLabel { background:transparent; }
            QFrame#logBanner { background:#172554; border:1px solid #1d4ed8; border-radius:8px; }
            QLabel#logTitle { color:white; font-size:16px; font-weight:700; }
            QLabel#logDetail { color:#bfdbfe; }
            QLabel#logCount { color:#dcfce7; background:#166534; border-radius:10px;
                              padding:4px 10px; font-weight:700; }
            QLabel#logEmpty { color:#64748b; padding:8px; }
            QPushButton { background:#e2e8f0; border:1px solid #b6c2d2; border-radius:5px;
                          padding:6px 12px; }
            QPushButton:hover { background:#cbd5e1; }
            QPushButton:checked { background:#b45309; color:white; border-color:#92400e; }
            QComboBox { background:white; border:1px solid #b6c2d2; border-radius:4px;
                        padding:4px; min-height:22px; }
            QTableWidget { background:#ffffff; alternate-background-color:#f1f5f9;
                           border:1px solid #cbd5e1; gridline-color:#e2e8f0; }
            QHeaderView::section { background:#e2e8f0; color:#334155; border:0;
                                   border-right:1px solid #cbd5e1; padding:6px; font-weight:600; }
        """)

    def append_event(self, level, category, node, message, node_color=""):
        entry = DebugLogEntry(
            datetime.now().strftime("%H:%M:%S.%f")[:-3],
            level,
            category,
            node,
            message,
            node_color,
        )
        self.entries.append(entry)
        self.log_count_label.setText(f"{len(self.entries):,} 条")
        if not self.pause_button.isChecked() and self.matches_filter(entry):
            if self.table.rowCount() >= self.MAX_ENTRIES:
                self.table.removeRow(0)
            self.append_row(entry)
        self.update_empty_state()

    def matches_filter(self, entry):
        selected = self.filter_combo.currentData()
        if selected == "all":
            return True
        if selected == "problem":
            return entry.level in ("警告", "错误")
        return entry.category == selected

    def append_row(self, entry):
        row = self.table.rowCount()
        self.table.insertRow(row)
        items = [
            QtWidgets.QTableWidgetItem(entry.timestamp),
            QtWidgets.QTableWidgetItem(entry.level),
            QtWidgets.QTableWidgetItem(entry.node),
            QtWidgets.QTableWidgetItem(entry.message),
        ]
        level_color = QtGui.QColor(self.LEVEL_COLORS.get(entry.level, "#334155"))
        items[1].setForeground(QtGui.QBrush(level_color))
        font = items[1].font(); font.setBold(True); items[1].setFont(font)
        if entry.node_color:
            items[2].setForeground(QtGui.QBrush(QtGui.QColor(entry.node_color)))
            node_font = items[2].font(); node_font.setBold(True); items[2].setFont(node_font)
        if entry.level in ("警告", "错误"):
            items[3].setForeground(QtGui.QBrush(level_color))
        for column, item in enumerate(items):
            self.table.setItem(row, column, item)
        if self.auto_scroll.isChecked():
            self.table.scrollToBottom()

    def rebuild_table(self, _index=None):
        self.table.setRowCount(0)
        if not self.pause_button.isChecked():
            for entry in self.entries:
                if self.matches_filter(entry):
                    self.append_row(entry)
        self.update_empty_state()

    def on_pause_changed(self, paused):
        self.pause_button.setText("继续刷新" if paused else "暂停刷新")
        if not paused:
            self.rebuild_table()

    def update_empty_state(self):
        self.empty_label.setVisible(self.table.rowCount() == 0)

    def visible_text(self):
        lines = []
        for row in range(self.table.rowCount()):
            lines.append("\t".join(
                self.table.item(row, column).text() for column in range(self.table.columnCount())
            ))
        return "\n".join(lines)

    def copy_visible(self):
        QtWidgets.QApplication.clipboard().setText(self.visible_text())

    def save_log(self):
        if not self.entries:
            QtWidgets.QMessageBox.information(self, self.windowTitle(), "当前没有日志可保存。")
            return
        default = f"ble_debug_{datetime.now():%Y%m%d_%H%M%S}.csv"
        path, _ = QtWidgets.QFileDialog.getSaveFileName(
            self, "保存蓝牙调试日志", str(Path.home() / default), "CSV 文件 (*.csv)"
        )
        if not path:
            return
        with open(path, "w", newline="", encoding="utf-8-sig") as handle:
            writer = csv.writer(handle)
            writer.writerow(["time", "level", "category", "node", "event"])
            for entry in self.entries:
                writer.writerow([
                    entry.timestamp, entry.level, entry.category, entry.node, entry.message
                ])

    def clear_log(self):
        self.entries.clear()
        self.table.setRowCount(0)
        self.log_count_label.setText("0 条")
        self.update_empty_state()

    def closeEvent(self, event):
        event.ignore()
        self.hide()


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
        self.multi_parser = MultiFrameParser()
        self.dc_rows = deque()
        self.dc_scan_points = []
        self.dc_seq = 1
        self.dc_start_time = None
        self.multi_nodes = {}
        self.multi_archived_nodes = {}
        self.multi_release_pending = {}
        self.multi_curves = {}
        # A new PC process may reconnect to a still-running BLE session. Independent
        # random seeds avoid restarting with the sensor's last accepted request key.
        self.multi_seq = secrets.randbelow(0xFFFF) + 1
        self.multi_scan_id = secrets.randbelow(0xFFFF) + 1
        self.multi_start_time = None
        self.multi_selected_node_id = None
        self.multi_relay_stats = None
        self.multi_paused = False
        self.multi_plot_dirty = False
        self.multi_last_stats_values = np.empty(0)
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
        self.single_link_data_seen = False
        self.last_logged_mode = None
        self.debug_log_window = DebugLogWindow(self)

        self.worker_thread = QtCore.QThread(self)
        self.worker = SerialWorker()
        self.worker.moveToThread(self.worker_thread)
        self.request_open.connect(self.worker.open_port)
        self.request_close.connect(self.worker.close_port)
        self.request_write.connect(self.worker.write)
        self.worker.bytes_received.connect(self.on_bytes)
        self.worker.bytes_written.connect(self.on_serial_bytes_written)
        self.worker.state_changed.connect(self.on_serial_state)
        self.worker.error.connect(self.on_serial_error)
        self.worker_thread.start()

        self.build_ui()
        self.refresh_ports()

        self.plot_timer = QtCore.QTimer(self)
        self.plot_timer.timeout.connect(self.refresh_plot)
        self.plot_timer.start(33)
        self.relay_timer = QtCore.QTimer(self)
        self.relay_timer.timeout.connect(self.check_multi_relay_timeouts)
        self.relay_timer.start(250)

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
        mode_row.addSpacing(26)
        mode_row.addWidget(QtWidgets.QLabel("连接方式："))
        self.single_mode = QtWidgets.QRadioButton("一对一")
        self.multi_mode = QtWidgets.QRadioButton("一对多")
        self.single_mode.setChecked(True)
        self.topology_modes = QtWidgets.QButtonGroup(self)
        self.topology_modes.setExclusive(True)
        self.topology_modes.addButton(self.single_mode)
        self.topology_modes.addButton(self.multi_mode)
        mode_row.addWidget(self.single_mode)
        mode_row.addWidget(self.multi_mode)
        mode_row.addStretch(1)
        self.debug_log_button = QtWidgets.QPushButton("调试日志")
        self.debug_log_button.setObjectName("debugLogButton")
        self.debug_log_button.setToolTip("打开蓝牙连接、GATT 发现和协议事件日志")
        self.debug_log_button.clicked.connect(self.open_debug_log)
        mode_row.addWidget(self.debug_log_button)
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
            "1 s", "2 s", "5 s",
            "10 s", "20 s", "30 s", "60 s", "2 min", "5 min", "10 min", "30 min", "1 h", "全部",
        ])
        self.duration_combo.setCurrentText("5 s")
        self.duration_combo.setToolTip("可选择秒/分钟/小时档位，也可自定义输入如 15 s 或 10 min")
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

        self.content_stack = QtWidgets.QStackedWidget()
        single_page = QtWidgets.QWidget()
        single_layout = QtWidgets.QVBoxLayout(single_page)
        single_layout.setContentsMargins(0, 0, 0, 0)

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
        single_layout.addWidget(self.dc_controls)

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
        single_layout.addLayout(action_row)

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
        single_layout.addLayout(measurement_row)

        self.view_box = SelectionViewBox()
        self.plot = pg.PlotWidget(viewBox=self.view_box)
        self.plot.setLabel("bottom", "时间", units="s")
        self.plot.setLabel("left", "采样值")
        self.plot.showGrid(x=True, y=True, alpha=0.25)
        # Decimate only the rendered curve; stored samples, statistics and CSV stay complete.
        self.plot.setDownsampling(auto=True, mode="peak")
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
        single_layout.addWidget(self.plot_tabs, 1)

        self.content_stack.addWidget(single_page)
        self.content_stack.addWidget(self.build_multi_page())
        layout.addWidget(self.content_stack, 1)

        self.status_left = QtWidgets.QLabel("未连接")
        self.status_right = QtWidgets.QLabel("有效帧 0 | 接收 0 B | 丢弃 0 B")
        self.statusBar().addWidget(self.status_left, 1)
        self.statusBar().addPermanentWidget(self.status_right)
        self.ac_mode.toggled.connect(self.on_sensor_mode_changed)
        self.dc_mode.toggled.connect(self.on_sensor_mode_changed)
        self.single_mode.toggled.connect(self.on_topology_mode_changed)
        self.multi_mode.toggled.connect(self.on_topology_mode_changed)
        self.apply_style()
        self.on_sensor_mode_changed()

    def build_multi_page(self):
        page = QtWidgets.QWidget()
        page_layout = QtWidgets.QVBoxLayout(page)
        page_layout.setContentsMargins(0, 4, 0, 0)
        page_layout.setSpacing(8)

        banner = QtWidgets.QFrame()
        banner.setObjectName("multiBanner")
        banner_layout = QtWidgets.QHBoxLayout(banner)
        banner_layout.setContentsMargins(14, 8, 14, 8)
        title = QtWidgets.QLabel("多节点采集台")
        title.setObjectName("multiTitle")
        self.multi_banner_detail = QtWidgets.QLabel(
            f"V2 NodeId 路由 · 最多 {MULTI_MAX_NODES} 个传感器 · 串口固定建议 921600 baud"
        )
        self.multi_banner_detail.setObjectName("multiDetail")
        banner_layout.addWidget(title)
        banner_layout.addSpacing(12)
        banner_layout.addWidget(self.multi_banner_detail)
        banner_layout.addStretch(1)
        self.multi_online_badge = QtWidgets.QLabel(f"0 / {MULTI_MAX_NODES} 在线")
        self.multi_online_badge.setObjectName("onlineBadge")
        banner_layout.addWidget(self.multi_online_badge)
        page_layout.addWidget(banner)

        splitter = QtWidgets.QSplitter(QtCore.Qt.Horizontal)
        splitter.setChildrenCollapsible(False)

        left_panel = QtWidgets.QWidget()
        left_panel.setMinimumWidth(320)
        left_panel.setMaximumWidth(430)
        left_layout = QtWidgets.QVBoxLayout(left_panel)
        left_layout.setContentsMargins(0, 0, 4, 0)

        node_box = QtWidgets.QGroupBox("传感器节点机架")
        node_layout = QtWidgets.QVBoxLayout(node_box)
        self.multi_node_table = QtWidgets.QTableWidget(0, 5)
        self.multi_node_table.setHorizontalHeaderLabels(
            ["通道", "状态", "NodeId", "帧数", "最新值"]
        )
        self.multi_node_table.setSelectionBehavior(QtWidgets.QAbstractItemView.SelectRows)
        self.multi_node_table.setSelectionMode(QtWidgets.QAbstractItemView.SingleSelection)
        self.multi_node_table.setEditTriggers(QtWidgets.QAbstractItemView.NoEditTriggers)
        self.multi_node_table.verticalHeader().setVisible(False)
        self.multi_node_table.horizontalHeader().setSectionResizeMode(QtWidgets.QHeaderView.Fixed)
        for column, width in enumerate((54, 48, 68, 42, 68)):
            self.multi_node_table.setColumnWidth(column, width)
        self.multi_node_table.horizontalHeader().setStretchLastSection(True)
        self.multi_node_table.setHorizontalScrollBarPolicy(QtCore.Qt.ScrollBarAlwaysOff)
        self.multi_node_table.setMinimumHeight(155)
        self.multi_node_table.itemSelectionChanged.connect(self.on_multi_node_selected)
        node_layout.addWidget(self.multi_node_table)
        self.multi_target_label = QtWidgets.QLabel("控制目标：尚未发现节点")
        self.multi_target_label.setObjectName("targetLabel")
        node_layout.addWidget(self.multi_target_label)
        self.multi_release_button = QtWidgets.QPushButton("断开并腾出位置")
        self.multi_release_button.setEnabled(False)
        self.multi_release_button.setToolTip(
            "断开选中设备（正在进行的扫描会中断）；暂缓重连 30 秒，让其他设备接入。"
            "之后有空位时自动重连，无需手动允许。"
        )
        self.multi_release_button.clicked.connect(self.release_multi_link)
        node_layout.addWidget(self.multi_release_button)
        release_hint = QtWidgets.QLabel("断开后暂缓重连 30 秒，之后有空位自动重连")
        release_hint.setWordWrap(True)
        node_layout.addWidget(release_hint)
        self.multi_node_table.setToolTip(
            f"CH1–CH{MULTI_MAX_NODES} 的离线位置可被新设备复用；按 NodeId 区分设备。"
            "点击一行选择控制目标；被替换设备的数据仍保留用于导出。"
        )
        left_layout.addWidget(node_box, 1)

        actions = QtWidgets.QGroupBox("显示与数据")
        actions_layout = QtWidgets.QGridLayout(actions)
        self.multi_show_all = QtWidgets.QCheckBox("叠加全部在线节点")
        self.multi_show_all.setChecked(True)
        self.multi_show_all.toggled.connect(self.mark_multi_dirty)
        self.multi_hold_data = QtWidgets.QCheckBox("保持全部数据")
        self.multi_auto_y = QtWidgets.QCheckBox("自动 Y 轴")
        self.multi_auto_y.setChecked(True)
        self.multi_auto_y.toggled.connect(self.on_multi_auto_y)
        self.multi_pause_button = QtWidgets.QPushButton("暂停显示")
        self.multi_pause_button.setCheckable(True)
        self.multi_pause_button.toggled.connect(self.toggle_multi_pause)
        self.multi_clear_button = QtWidgets.QPushButton("清空多节点数据")
        self.multi_clear_button.clicked.connect(self.clear_multi_data)
        self.multi_save_button = QtWidgets.QPushButton("导出多节点 CSV")
        self.multi_save_button.clicked.connect(self.save_multi_data)
        actions_layout.addWidget(self.multi_show_all, 0, 0)
        actions_layout.addWidget(self.multi_hold_data, 0, 1)
        actions_layout.addWidget(self.multi_auto_y, 1, 0)
        actions_layout.addWidget(self.multi_pause_button, 1, 1)
        actions_layout.addWidget(self.multi_clear_button, 2, 0)
        actions_layout.addWidget(self.multi_save_button, 2, 1)
        left_layout.addWidget(actions)

        multi_stats = QtWidgets.QGroupBox("选中节点统计")
        multi_stats_layout = QtWidgets.QGridLayout(multi_stats)
        self.multi_value_name = QtWidgets.QLabel("当前字段：ADC")
        self.multi_latest_label = QtWidgets.QLabel("最新值：--")
        self.multi_pp_label = QtWidgets.QLabel("峰峰值：--")
        self.multi_rms_label = QtWidgets.QLabel("有效值：--")
        self.multi_mean_label = QtWidgets.QLabel("平均值：--")
        self.multi_range_label = QtWidgets.QLabel("最小/最大：-- / --")
        self.multi_samples_label = QtWidgets.QLabel("窗口点数：0")
        multi_stats_layout.addWidget(self.multi_value_name, 0, 0)
        multi_stats_layout.addWidget(self.multi_samples_label, 0, 1)
        multi_stats_layout.addWidget(self.multi_latest_label, 1, 0)
        multi_stats_layout.addWidget(self.multi_pp_label, 1, 1)
        multi_stats_layout.addWidget(self.multi_rms_label, 2, 0)
        multi_stats_layout.addWidget(self.multi_mean_label, 2, 1)
        multi_stats_layout.addWidget(self.multi_range_label, 3, 0, 1, 2)
        left_layout.addWidget(multi_stats)
        splitter.addWidget(left_panel)

        right_panel = QtWidgets.QWidget()
        right_layout = QtWidgets.QVBoxLayout(right_panel)
        right_layout.setContentsMargins(4, 0, 0, 0)

        self.multi_relay_controls = QtWidgets.QGroupBox("继电器放电（选中节点）")
        relay_layout = QtWidgets.QGridLayout(self.multi_relay_controls)
        self.multi_relay_button = QtWidgets.QPushButton("继电器放电")
        self.multi_relay_button.setEnabled(False)
        self.multi_relay_button.clicked.connect(self.start_multi_relay_discharge)
        self.multi_relay_button.setToolTip("仅向当前选中的在线节点发送一次 5 秒放电请求")
        self.multi_relay_status = QtWidgets.QLabel("状态未知（等待设备上报）")
        self.multi_relay_status.setWordWrap(True)
        relay_hint = QtWidgets.QLabel(
            "PD7 拉高 5 秒后由传感器自动拉低；断连不会中止，状态以设备反馈为准。"
        )
        relay_hint.setWordWrap(True)
        relay_layout.addWidget(self.multi_relay_button, 0, 0)
        relay_layout.addWidget(self.multi_relay_status, 0, 1)
        relay_layout.addWidget(relay_hint, 1, 0, 1, 2)
        relay_layout.setColumnStretch(1, 1)
        right_layout.addWidget(self.multi_relay_controls)

        self.multi_dc_controls = QtWidgets.QGroupBox("选中节点 DAC 控制（V2 定向命令）")
        multi_dc_grid = QtWidgets.QGridLayout(self.multi_dc_controls)
        self.multi_dc_min = QtWidgets.QSpinBox(); self.multi_dc_min.setRange(0, 65535)
        self.multi_dc_max = QtWidgets.QSpinBox(); self.multi_dc_max.setRange(0, 65535); self.multi_dc_max.setValue(65535)
        self.multi_dc_points = QtWidgets.QSpinBox(); self.multi_dc_points.setRange(2, 255); self.multi_dc_points.setValue(100)
        self.multi_dc_settle = QtWidgets.QSpinBox(); self.multi_dc_settle.setRange(0, 10000); self.multi_dc_settle.setValue(20); self.multi_dc_settle.setSuffix(" ms")
        self.multi_dc_average = QtWidgets.QSpinBox(); self.multi_dc_average.setRange(1, 255); self.multi_dc_average.setValue(1)
        self.multi_dc_value = QtWidgets.QSpinBox(); self.multi_dc_value.setRange(0, 65535); self.multi_dc_value.setValue(32768)
        multi_dc_fields = [
            ("起点", self.multi_dc_min), ("终点", self.multi_dc_max),
            ("点数", self.multi_dc_points), ("等待", self.multi_dc_settle),
            ("平均次数", self.multi_dc_average), ("DAC 值", self.multi_dc_value),
        ]
        for column, (label, widget) in enumerate(multi_dc_fields):
            multi_dc_grid.addWidget(QtWidgets.QLabel(label), 0, column)
            multi_dc_grid.addWidget(widget, 1, column)
        self.multi_dc_start = QtWidgets.QPushButton("开始扫描")
        self.multi_dc_start.clicked.connect(self.start_multi_dc_scan)
        self.multi_dc_abort = QtWidgets.QPushButton("终止扫描")
        self.multi_dc_abort.clicked.connect(self.abort_multi_dc_scan)
        self.multi_dc_set = QtWidgets.QPushButton("设置 DAC")
        self.multi_dc_set.clicked.connect(lambda: self.set_multi_dc_dac(False))
        self.multi_dc_save = QtWidgets.QPushButton("设置并保存")
        self.multi_dc_save.clicked.connect(lambda: self.set_multi_dc_dac(True))
        self.multi_dc_export = QtWidgets.QPushButton("导出扫描点")
        self.multi_dc_export.clicked.connect(self.export_multi_dc_scan)
        multi_dc_grid.addWidget(self.multi_dc_start, 0, 6)
        multi_dc_grid.addWidget(self.multi_dc_abort, 1, 6)
        multi_dc_grid.addWidget(self.multi_dc_set, 0, 7)
        multi_dc_grid.addWidget(self.multi_dc_save, 1, 7)
        multi_dc_grid.addWidget(self.multi_dc_export, 0, 8, 2, 1)
        right_layout.addWidget(self.multi_dc_controls)

        self.multi_plot = pg.PlotWidget()
        self.multi_plot.setDownsampling(auto=True, mode="peak")
        self.multi_plot.setLabel("bottom", "相对采集时间", units="s")
        self.multi_plot.setLabel("left", "ADC")
        self.multi_plot.showGrid(x=True, y=True, alpha=0.25)
        self.multi_plot.addLegend(offset=(12, 12))

        self.multi_scan_plot = pg.PlotWidget()
        self.multi_scan_plot.setLabel("bottom", "DAC")
        self.multi_scan_plot.setLabel("left", "Lock-in")
        self.multi_scan_plot.showGrid(x=True, y=True, alpha=0.25)
        self.multi_scan_curve = self.multi_scan_plot.plot(
            [], [], pen=pg.mkPen("#22d3ee", width=1.6), symbol="o", symbolSize=7,
            symbolBrush=pg.mkBrush("#22d3ee"), symbolPen=pg.mkPen("#e0f2fe"),
        )
        multi_scan_page = QtWidgets.QWidget()
        multi_scan_layout = QtWidgets.QVBoxLayout(multi_scan_page)
        multi_scan_layout.setContentsMargins(0, 0, 0, 0)
        self.multi_scan_info = QtWidgets.QLabel("选择节点后可执行 DAC 扫描")
        self.multi_scan_info.setObjectName("scanInfo")
        multi_scan_layout.addWidget(self.multi_scan_info)
        multi_scan_layout.addWidget(self.multi_scan_plot, 1)

        self.multi_plot_tabs = QtWidgets.QTabWidget()
        self.multi_plot_tabs.addTab(self.multi_plot, "多节点实时波形")
        self.multi_plot_tabs.addTab(multi_scan_page, "选中节点 DAC 扫描")
        right_layout.addWidget(self.multi_plot_tabs, 1)
        splitter.addWidget(right_panel)
        splitter.setStretchFactor(0, 0)
        splitter.setStretchFactor(1, 1)
        page_layout.addWidget(splitter, 1)
        return page

    def apply_style(self):
        self.setStyleSheet("""
            QMainWindow, QWidget { background: #f8fafc; color: #172033; font-size: 13px; }
            QLabel { background: transparent; }
            QGroupBox { font-weight: 600; border: 1px solid #cbd5e1; border-radius: 7px;
                        margin-top: 9px; padding-top: 9px; }
            QGroupBox::title { subcontrol-origin: margin; left: 10px; padding: 0 5px; }
            QPushButton { background: #e2e8f0; border: 1px solid #b6c2d2; border-radius: 5px;
                          padding: 6px 13px; }
            QPushButton:hover { background: #cbd5e1; }
            QPushButton:checked { background: #2563eb; color: white; border-color: #1d4ed8; }
            QPushButton#debugLogButton { background: #172554; color: white; border-color: #1d4ed8;
                                         font-weight: 700; padding: 6px 16px; }
            QPushButton#debugLogButton:hover { background: #1e3a8a; }
            QRadioButton { spacing: 6px; font-weight: 600; }
            QComboBox, QDoubleSpinBox, QSpinBox { background: white; border: 1px solid #b6c2d2;
                                       border-radius: 4px; padding: 4px; min-height: 22px; }
            QTableWidget { background: white; alternate-background-color: #f1f5f9;
                           border: 1px solid #cbd5e1; gridline-color: #e2e8f0; }
            QHeaderView::section { background: #e2e8f0; color: #334155; border: 0;
                                   border-right: 1px solid #cbd5e1; padding: 5px; font-weight: 600; }
            QFrame#multiBanner { background: #172554; border: 1px solid #1d4ed8; border-radius: 8px; }
            QLabel#multiTitle { color: #ffffff; font-size: 16px; font-weight: 700; }
            QLabel#multiDetail { color: #bfdbfe; }
            QLabel#onlineBadge { color: #dcfce7; background: #166534; border-radius: 10px;
                                 padding: 4px 10px; font-weight: 700; }
            QLabel#targetLabel { color: #1d4ed8; font-weight: 700; padding: 3px; }
            QLabel#hintLabel { color: #64748b; font-size: 12px; }
            QLabel#scanInfo { color: #1d4ed8; font-weight: 600; padding: 4px; }
            QStatusBar { background: #e2e8f0; }
        """)

    def open_debug_log(self):
        self.debug_log_window.show()
        self.debug_log_window.raise_()
        self.debug_log_window.activateWindow()

    def log_event(self, level, category, message, node="上位机", node_color=""):
        self.debug_log_window.append_event(level, category, node, message, node_color)

    def multi_node_log_identity(self, node_id):
        node = self.multi_nodes.get(node_id) or self.multi_archived_nodes.get(node_id)
        if node is None:
            return f"0x{node_id:04X}", ""
        color = MULTI_NODE_COLORS[node.color_index]
        return f"CH{node.color_index + 1} · 0x{node_id:04X}", color

    @QtCore.pyqtSlot(str)
    def on_serial_error(self, message):
        self.log_event("错误", "problem", message, "串口")
        self.show_error(message)

    @QtCore.pyqtSlot(int)
    def on_serial_bytes_written(self, count):
        self.log_event(
            "串口", "protocol",
            f"上位机已向系统串口发送缓冲区写入 {count} 字节（不代表中继已接收）",
            "上位机",
        )

    def on_sensor_mode_changed(self, _checked=False):
        if not self.ac_mode.isChecked() and not self.dc_mode.isChecked():
            return
        is_dc = self.dc_mode.isChecked()
        if self.connect_button.isChecked():
            self.connect_button.setChecked(False)
            self.toggle_connection(False)
        is_multi = self.is_multi_mode()
        self.dc_controls.setVisible(is_dc and not is_multi)
        self.multi_dc_controls.setVisible(is_dc and is_multi)
        self.data_format.setEnabled(not is_dc and not is_multi)
        self.sample_rate.setEnabled(not is_dc and not is_multi)
        self.show_ac_curve.setText("显示去直流波形" if not is_dc else "显示去均值波形")
        self.baud_combo.setCurrentText("921600" if is_multi or not is_dc else "115200")
        self.plot_tabs.setTabEnabled(1, is_dc)
        self.plot_tabs.setCurrentIndex(0)
        self.multi_plot_tabs.setTabEnabled(1, is_dc)
        self.multi_plot_tabs.setCurrentIndex(0)
        self.multi_value_name.setText("当前字段：Lock-in" if is_dc else "当前字段：ADC")
        self.multi_plot.setLabel("left", "Lock-in" if is_dc else "ADC")
        self.multi_banner_detail.setText(
            f"V2 NodeId 路由 · 最多 {MULTI_MAX_NODES} 个传感器 · 当前显示 {'Lock-in' if is_dc else 'ADC'}"
        )
        self.clear_data()
        self.clear_multi_data(True)
        topology = "一对多" if is_multi else "一对一"
        self.status_left.setText(f"{'直流' if is_dc else '交流'}传感器 · {topology}模式")
        self.single_link_data_seen = False
        mode_signature = (is_dc, is_multi)
        if mode_signature != self.last_logged_mode:
            self.last_logged_mode = mode_signature
            self.log_event(
                "信息", "protocol", f"切换到{'直流' if is_dc else '交流'}传感器 · {topology}模式"
            )

    def on_topology_mode_changed(self, _checked=False):
        if not self.single_mode.isChecked() and not self.multi_mode.isChecked():
            return
        if self.connect_button.isChecked():
            self.connect_button.setChecked(False)
            self.toggle_connection(False)
        is_multi = self.is_multi_mode()
        self.content_stack.setCurrentIndex(1 if is_multi else 0)
        self.baud_combo.setCurrentText(
            "921600" if is_multi or self.ac_mode.isChecked() else "115200"
        )
        self.on_sensor_mode_changed()

    def is_multi_mode(self):
        return hasattr(self, "multi_mode") and self.multi_mode.isChecked()

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
            self.log_event("信息", "link", f"请求打开 {port} @ {baud}", "串口")
            self.request_open.emit(port, baud)
        else:
            self.log_event("信息", "link", "请求关闭串口", "串口")
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
        self.log_event("信息", "protocol", f"开始自动检测 {port} 波特率", "串口")
        QtWidgets.QApplication.setOverrideCursor(QtCore.Qt.WaitCursor)
        found = None
        try:
            for baud in candidates:
                QtWidgets.QApplication.processEvents()
                probe = None
                try:
                    probe = serial.Serial(port, baud, timeout=0.02)
                    probe.reset_input_buffer()
                    if self.is_multi_mode():
                        parser = MultiFrameParser()
                    else:
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
            if self.is_multi_mode():
                protocol = "V2 多节点协议帧"
            else:
                protocol = "直流协议帧" if self.dc_mode.isChecked() else "合法246字节帧"
            self.status_left.setText(f"未检测到{protocol}")
            self.log_event("警告", "problem", f"未检测到{protocol}", "串口")
            self.show_error("未检测到合法帧。请确认传感器类型选择正确且接收器正在发送数据。")
        else:
            self.baud_combo.setCurrentText(str(found))
            self.status_left.setText(f"检测成功：{port} @ {found}")
            self.log_event("信息", "protocol", f"波特率检测成功：{port} @ {found}", "串口")

    @QtCore.pyqtSlot(bool, str)
    def on_serial_state(self, connected, text):
        self.connect_button.blockSignals(True)
        self.connect_button.setChecked(connected)
        self.connect_button.setText("断开" if connected else "连接")
        self.connect_button.blockSignals(False)
        self.port_combo.setEnabled(not connected)
        self.baud_combo.setEnabled(not connected)
        self.ac_mode.setEnabled(not connected)
        self.dc_mode.setEnabled(not connected)
        self.single_mode.setEnabled(not connected)
        self.multi_mode.setEnabled(not connected)
        if not connected and self.is_multi_mode():
            self.multi_release_pending.clear()
            for node in (*self.multi_nodes.values(), *self.multi_archived_nodes.values()):
                node.online = False
                node.link_state = 0
                self.reset_multi_relay_state(node)
            self.update_multi_node_table()
            self.multi_plot_dirty = True
        self.single_link_data_seen = False if not connected else self.single_link_data_seen
        self.update_multi_target_label()
        self.status_left.setText(("已连接：" if connected else "") + text)
        if connected:
            self.log_event("连接", "link", f"串口已打开：{text}", "串口")
        else:
            self.log_event("信息", "link", "串口已关闭；等待重新连接", "串口")

    @QtCore.pyqtSlot(bytes)
    def on_bytes(self, chunk):
        self.total_bytes += len(chunk)
        if self.is_multi_mode():
            self.on_multi_bytes(chunk)
            return
        if self.dc_mode.isChecked():
            self.on_dc_bytes(chunk)
            return
        discarded_before = self.parser.discarded_bytes
        frames = self.parser.feed(chunk)
        discarded_delta = self.parser.discarded_bytes - discarded_before
        if discarded_delta:
            self.log_event(
                "警告", "problem", f"交流帧同步丢弃 {discarded_delta} 字节", "一对一接收器"
            )
        if not frames:
            self.update_status()
            return
        if not self.single_link_data_seen:
            self.single_link_data_seen = True
            self.log_event(
                "连接", "link", "收到首个合法交流数据帧；推断蓝牙数据链路可用", "一对一接收器"
            )
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
        discarded_before = self.dc_parser.discarded_bytes
        frames = self.dc_parser.feed(chunk)
        discarded_delta = self.dc_parser.discarded_bytes - discarded_before
        if discarded_delta:
            self.log_event(
                "警告", "problem", f"直流 V1 协议丢弃 {discarded_delta} 字节", "一对一接收器"
            )
        if frames and not self.single_link_data_seen:
            self.single_link_data_seen = True
            self.log_event(
                "连接", "link", "收到首个合法直流 V1 帧；推断蓝牙数据链路可用", "一对一接收器"
            )
        for kind, _flags, _scan_id, _seq, payload in frames:
            if kind == DC_EVT_SCAN_BEGIN:
                self.dc_scan_points.clear()
                self.dc_scan_curve.setData([], [])
                self.clear_dc_scan_selection()
                self.dc_scan_info.setText(f"扫描进行中：0 / {self.dc_points.value()} 点")
                self.status_left.setText("DAC 扫描开始")
                self.log_event("信息", "protocol", "DAC 扫描开始", "一对一传感器")
            elif kind == DC_EVT_SCAN_POINTS:
                try:
                    points = decode_counted_records(payload, "<BHH")
                except ValueError as exc:
                    self.log_event("警告", "problem", f"丢弃 V1 扫描点：{exc}", "一对一传感器")
                    continue
                self.dc_scan_points.extend(points)
                self.refresh_dc_scan_plot()
            elif kind == DC_EVT_SCAN_END:
                self.status_left.setText(f"DAC 扫描完成，共 {len(self.dc_scan_points)} 点")
                self.dc_scan_info.setText(f"扫描完成：{len(self.dc_scan_points)} 点；点击数据点可查看坐标")
                self.log_event(
                    "信息", "protocol", f"DAC 扫描完成：{len(self.dc_scan_points)} 点", "一对一传感器"
                )
            elif kind == DC_EVT_SENSOR_READINGS:
                try:
                    readings = decode_counted_records(payload, "<HHHH")
                except ValueError as exc:
                    self.log_event("警告", "problem", f"丢弃 V1 实时读数：{exc}", "一对一传感器")
                    continue
                now = time.time()
                for sample_index, (seq, dac, lockin, adc) in enumerate(readings):
                    timestamp = now + sample_index * 1e-6
                    if self.dc_start_time is None:
                        self.dc_start_time = timestamp
                    self.dc_rows.append((timestamp, seq, dac, lockin, adc))
                    self.total_samples += 1
        self.trim_buffer()
        self.plot_dirty = bool(frames) or self.plot_dirty
        self.update_status()

    def ensure_multi_node(self, node_id):
        if node_id in (MULTI_NODE_RELAY, MULTI_NODE_BROADCAST):
            return None
        node = self.multi_nodes.get(node_id)
        if node is not None:
            return node
        if len(self.multi_nodes) >= MULTI_MAX_NODES:
            offline = sorted(
                (item for item in self.multi_nodes.values()
                 if not item.online and item.link_state != 2),
                key=lambda item: item.last_seen,
            )
            if not offline:
                self.status_left.setText(
                    f"忽略 NodeId 0x{node_id:04X}：当前 {MULTI_MAX_NODES} 个位置均被连接占用"
                )
                return None
            retired = offline[0]
            self.multi_archived_nodes[retired.node_id] = self.multi_nodes.pop(retired.node_id)
            self.multi_plot.removeItem(self.multi_curves.pop(retired.node_id))
            self.multi_release_pending.pop(retired.node_id, None)
            self.reset_multi_relay_state(retired)
            if self.multi_selected_node_id == retired.node_id:
                self.multi_selected_node_id = node_id
            self.log_event(
                "信息", "link",
                f"离线 NodeId 0x{retired.node_id:04X} 的显示位置已腾出，历史数据保留用于导出",
                "上位机",
            )
        occupied = {item.color_index for item in self.multi_nodes.values()}
        color_index = next(i for i in range(MULTI_MAX_NODES) if i not in occupied)
        node = self.multi_archived_nodes.pop(node_id, None)
        returning = node is not None
        if node is None:
            node = MultiNodeData(node_id=node_id, color_index=color_index)
        node.color_index = color_index
        self.reset_multi_relay_state(node)
        self.multi_nodes[node_id] = node
        color = MULTI_NODE_COLORS[node.color_index]
        self.multi_curves[node_id] = self.multi_plot.plot(
            [], [], pen=pg.mkPen(color, width=1.5), name=f"CH{node.color_index + 1} · 0x{node_id:04X}"
        )
        if self.multi_selected_node_id is None:
            self.multi_selected_node_id = node_id
        self.update_multi_node_table()
        self.log_event(
            "发现", "link", f"{'重新发现' if returning else '首次发现'} NodeId 0x{node_id:04X}，分配为 CH{node.color_index + 1}",
            f"CH{node.color_index + 1} · 0x{node_id:04X}", color,
        )
        return node

    def on_multi_bytes(self, chunk):
        discarded_before = self.multi_parser.discarded_bytes
        frames = self.multi_parser.feed(chunk)
        discarded_delta = self.multi_parser.discarded_bytes - discarded_before
        if discarded_delta:
            self.log_event(
                "警告", "problem", f"V2 帧校验/同步丢弃 {discarded_delta} 字节", "数据中继"
            )
        for frame in frames:
            if frame.kind == MULTI_EVT_RAW_SAMPLES:
                if frame.node_id == MULTI_NODE_RELAY:
                    source, color = "数据中继", ""
                else:
                    source, color = self.multi_node_log_identity(frame.node_id)
                diagnostic = frame.payload.decode("utf-8", errors="replace").strip("\x00\r\n ")
                if not diagnostic:
                    diagnostic = frame.payload.hex(" ").upper() or "<空诊断帧>"
                self.log_event("设备", "device", diagnostic, source, color)
                continue
            if frame.kind == MULTI_EVT_RELAY_STATS and frame.node_id == MULTI_NODE_RELAY:
                if len(frame.payload) >= 6:
                    duplicate_count = int.from_bytes(frame.payload[3:6], "little")
                    new_stats = (
                        frame.payload[0], frame.payload[1], frame.payload[2], duplicate_count
                    )
                    if self.multi_relay_stats is None or new_stats[0] != self.multi_relay_stats[0]:
                        self.log_event(
                            "信息", "link", f"中继连接数更新：{new_stats[0]}，BLE 队列深度 {new_stats[1]}",
                            "数据中继",
                        )
                    self.multi_relay_stats = new_stats
                continue
            # Relay-local ACK confirms acceptance only; physical disconnect is
            # confirmed separately by LINK_STATUS. Never resurrect a history slot.
            if (frame.kind in (MULTI_EVT_ACK, MULTI_EVT_NACK) and frame.payload
                    and frame.payload[0] == MULTI_CMD_RELEASE_LINK):
                success = frame.kind == MULTI_EVT_ACK and len(frame.payload) >= 2 and frame.payload[1] == 0
                if not success and self.multi_release_pending.get(frame.node_id) == frame.seq:
                    self.multi_release_pending.pop(frame.node_id, None)
                reason = frame.payload[1] if len(frame.payload) >= 2 else 0xFF
                message = ("中继已接受释放请求，等待蓝牙实际断开；断开后 30 秒可自动重连"
                           if success else "中继拒绝释放连接：" + MULTI_STATUS_NAMES.get(reason, f"0x{reason:02X}"))
                identity, color = self.multi_node_log_identity(frame.node_id)
                self.log_event("确认" if success else "错误", "link", message, identity, color)
                self.status_left.setText(message)
                continue
            if (frame.node_id in self.multi_archived_nodes
                    and frame.kind != MULTI_EVT_SENSOR_READINGS
                    and not (frame.kind == MULTI_EVT_LINK_STATUS and frame.payload
                             and frame.payload[0] in (1, 2))):
                # Late ACK/disconnect/scan frames must not displace a new peer.
                continue
            if frame.kind == MULTI_EVT_RELAY_STATE and frame.node_id not in self.multi_nodes:
                continue
            node = self.ensure_multi_node(frame.node_id)
            if node is None:
                continue
            node.frames += 1
            node.last_seen = time.time()
            if frame.kind == MULTI_EVT_COMMAND_TRACE:
                stage = frame.payload[0] if frame.payload else 0
                command = frame.payload[1] if len(frame.payload) >= 2 else 0
                status = frame.payload[2] if len(frame.payload) >= 3 else 0
                attempt = frame.payload[3] if len(frame.payload) >= 4 else 0
                queue_depth = frame.payload[4] if len(frame.payload) >= 5 else 0
                frame_length = frame.payload[5] if len(frame.payload) >= 6 else 0
                observed = frame.payload[6] if len(frame.payload) >= 7 else 0
                write_handle = (
                    struct.unpack_from("<H", frame.payload, 7)[0]
                    if len(frame.payload) >= 9 else 0
                )
                stage_name = MULTI_COMMAND_TRACE_NAMES.get(stage, f"未知链路阶段 {stage}")
                command_name = MULTI_COMMAND_NAMES.get(command, f"命令 0x{command:02X}")
                details = (
                    f"{stage_name}：{command_name}；尝试 {attempt}；"
                    f"BLE 队列 {queue_depth}；帧长 {frame_length}；"
                    f"RX 句柄 0x{write_handle:04X}"
                )
                if observed:
                    details += f"；传感器事件 0x{observed:02X}"
                if status:
                    details += f"；状态 0x{status:02X}"
                identity, color = self.multi_node_log_identity(node.node_id)
                if stage in (0x05, 0x06, 0x08, 0x09):
                    level, category = "错误", "problem"
                elif stage == 0x07:
                    level, category = "重试", "problem"
                elif stage in (0x02, 0x04):
                    level, category = "确认", "protocol"
                else:
                    level, category = "链路", "protocol"
                self.log_event(level, category, details, identity, color)
                if stage in (0x08, 0x09) and command == MULTI_CMD_SCAN_START:
                    node.scan_state = "failed"
                    if node.node_id == self.multi_selected_node_id:
                        self.multi_scan_info.setText(
                            f"CH{node.color_index + 1} 扫描失败：{stage_name}"
                        )
            elif frame.kind == MULTI_EVT_LINK_STATUS:
                previous_state = node.link_state
                previous_stage = node.link_stage
                previous_hci_reason = node.hci_reason
                previous_gatt_error = node.gatt_error
                if frame.payload:
                    node.link_state = frame.payload[0]
                    node.online = node.link_state == 1
                    if node.link_state != previous_state or not node.online:
                        self.reset_multi_relay_state(node)
                    if (node.link_state == 0 and
                            node.scan_state in ("waiting_ack", "accepted", "executing")):
                        node.scan_state = "failed"
                        if node.node_id == self.multi_selected_node_id:
                            self.multi_scan_info.setText(
                                f"CH{node.color_index + 1} 扫描失败：BLE 链路已断开"
                            )
                if len(frame.payload) >= 8:
                    node.address = ":".join(f"{value:02X}" for value in frame.payload[2:8])
                node.hci_reason = frame.payload[8] if len(frame.payload) >= 9 else 0
                node.link_stage = frame.payload[9] if len(frame.payload) >= 10 else 0
                node.gatt_error = frame.payload[10] if len(frame.payload) >= 11 else 0
                node.att_mtu = (
                    struct.unpack_from("<H", frame.payload, 11)[0]
                    if len(frame.payload) >= 13 else 23
                )
                state_name = {0: "已断开", 1: "已就绪", 2: "发现中"}.get(
                    node.link_state, f"状态 {node.link_state}"
                )
                self.status_left.setText(f"NodeId 0x{node.node_id:04X} {state_name}")
                event_changed = (
                    node.link_state != previous_state
                    or node.link_stage != previous_stage
                    or node.hci_reason != previous_hci_reason
                    or node.gatt_error != previous_gatt_error
                    or node.frames == 1
                )
                if event_changed:
                    identity, color = self.multi_node_log_identity(node.node_id)
                    if node.link_state == 0:
                        manual_release = self.multi_release_pending.pop(node.node_id, None) is not None
                        level = "信息" if manual_release else "错误"
                        details = [f"蓝牙连接已断开；地址 {node.address}"]
                        if manual_release:
                            details.append("位置已释放；30 秒后有空位时自动重新连接")
                        if node.hci_reason:
                            reason_name = MULTI_HCI_REASON_NAMES.get(
                                node.hci_reason, "未收录的 HCI 原因"
                            )
                            details.append(
                                f"HCI 0x{node.hci_reason:02X}（{reason_name}）"
                            )
                        if node.link_stage:
                            details.append(
                                "阶段 " + MULTI_GATT_STAGE_NAMES.get(
                                    node.link_stage, f"未知 {node.link_stage}"
                                )
                            )
                        if node.gatt_error:
                            error_name = MULTI_GATT_ERROR_NAMES.get(
                                node.gatt_error, "未收录的 GATT/中继错误"
                            )
                            details.append(
                                f"错误 0x{node.gatt_error:02X}（{error_name}）"
                            )
                        message = "；".join(details)
                    elif node.link_state == 1:
                        level = "连接"
                        message = (
                            f"GATT 服务已就绪；ATT MTU={node.att_mtu}；地址 {node.address}"
                        )
                    elif node.link_state == 2:
                        level = "发现"
                        stage_name = MULTI_GATT_STAGE_NAMES.get(
                            node.link_stage, f"未知阶段 {node.link_stage}"
                        )
                        message = (
                            f"物理 BLE 已连接，GATT 阶段：{stage_name}；地址 {node.address}"
                        )
                    else:
                        level = "警告"
                        message = f"收到未知链路状态 {node.link_state}；地址 {node.address}"
                    self.log_event(level, "link", message, identity, color)
            elif frame.kind == MULTI_EVT_RELAY_STATE:
                self.on_multi_relay_state(node, frame)
            elif frame.kind == MULTI_EVT_SENSOR_READINGS:
                high_precision = bool(frame.flags & MULTI_FLAG_LOCKIN_CENTI)
                try:
                    readings = decode_counted_records(
                        frame.payload, "<HHIH" if high_precision else "<HHHH"
                    )
                except ValueError as exc:
                    identity, color = self.multi_node_log_identity(node.node_id)
                    self.log_event("警告", "problem", f"丢弃 V2 实时读数：{exc}", identity, color)
                    continue
                was_online = node.online
                node.online = True
                node.link_state = 1
                if not was_online:
                    self.reset_multi_relay_state(node)
                    identity, color = self.multi_node_log_identity(node.node_id)
                    self.log_event(
                        "连接", "link", "收到传感器读数；链路状态恢复为可用", identity, color
                    )
                now = time.time()
                for sample_index, (seq, dac, lockin, adc) in enumerate(readings):
                    if high_precision:
                        lockin /= 100.0
                    timestamp = now + sample_index * 1e-6
                    if self.multi_start_time is None:
                        self.multi_start_time = timestamp
                    node.rows.append((timestamp, seq, dac, lockin, adc))
                    self.total_samples += 1
            elif frame.kind == MULTI_EVT_SCAN_BEGIN:
                node.scan_points.clear()
                node.scan_state = "executing"
                node.scan_id = frame.scan_id
                if len(frame.payload) >= 11:
                    dac_min, dac_max, total_points, settle_ms = struct.unpack_from(
                        "<HHHH", frame.payload, 0
                    )
                    average = frame.payload[8]
                    restored_dac = struct.unpack_from("<H", frame.payload, 9)[0]
                    node.scan_expected = total_points
                    begin_message = (
                        f"传感器开始 DAC 扫描：{dac_min}→{dac_max}，"
                        f"共 {total_points} 点，等待 {settle_ms} ms，"
                        f"平均 {average} 次，完成后恢复 DAC={restored_dac}"
                    )
                else:
                    begin_message = "传感器开始 DAC 扫描"
                identity, color = self.multi_node_log_identity(node.node_id)
                self.log_event("信息", "protocol", begin_message, identity, color)
                if node.node_id == self.multi_selected_node_id:
                    self.multi_scan_curve.setData([], [])
                    self.multi_scan_info.setText(
                        f"CH{node.color_index + 1} 传感器正在执行：0 / {node.scan_expected} 点"
                    )
                    self.multi_plot_tabs.setCurrentIndex(1)
            elif frame.kind == MULTI_EVT_SCAN_POINTS:
                high_precision = bool(frame.flags & MULTI_FLAG_LOCKIN_CENTI)
                try:
                    points = decode_counted_records(
                        frame.payload, "<BHI" if high_precision else "<BHH"
                    )
                except ValueError as exc:
                    identity, color = self.multi_node_log_identity(node.node_id)
                    self.log_event("警告", "problem", f"丢弃 V2 扫描点：{exc}", identity, color)
                    continue
                added = 0
                for index, dac, lockin in points:
                    if high_precision:
                        lockin /= 100.0
                    node.scan_points.append((index, dac, lockin))
                    added += 1
                identity, color = self.multi_node_log_identity(node.node_id)
                self.log_event(
                    "进度", "protocol",
                    f"传感器返回 {added} 个扫描点，累计 "
                    f"{len(node.scan_points)} / {node.scan_expected or '?'} 点",
                    identity, color,
                )
                if node.node_id == self.multi_selected_node_id:
                    self.refresh_multi_scan_plot()
            elif frame.kind == MULTI_EVT_SCAN_END:
                result = frame.payload[0] if frame.payload else 0
                completed = len(node.scan_points)
                restored_dac = None
                if len(frame.payload) >= 5:
                    completed, restored_dac = struct.unpack_from("<HH", frame.payload, 1)
                node.scan_state = "completed" if result == 0 else "aborted"
                if node.node_id == self.multi_selected_node_id:
                    self.multi_scan_info.setText(
                        f"CH{node.color_index + 1} 扫描"
                        f"{'完成' if result == 0 else '中止'}：{completed} 点"
                        + (f"，DAC 已恢复为 {restored_dac}" if restored_dac is not None else "")
                    )
                self.status_left.setText(
                    f"NodeId 0x{node.node_id:04X} DAC 扫描"
                    f"{'完成' if result == 0 else '中止'}，共 {completed} 点"
                )
                identity, color = self.multi_node_log_identity(node.node_id)
                self.log_event(
                    "信息" if result == 0 else "警告", "protocol",
                    f"传感器报告 DAC 扫描{'完成' if result == 0 else '中止'}：{completed} 点"
                    + (f"，恢复 DAC={restored_dac}" if restored_dac is not None else ""),
                    identity, color,
                )
            elif (frame.kind in (MULTI_EVT_ACK, MULTI_EVT_NACK)
                  and frame.payload[:1] == bytes([MULTI_CMD_RELAY_DISCHARGE])):
                self.on_multi_relay_reply(node, frame)
            elif frame.kind == MULTI_EVT_ACK:
                original = frame.payload[0] if frame.payload else 0
                status = frame.payload[1] if len(frame.payload) >= 2 else 0
                command_name = MULTI_COMMAND_NAMES.get(original, f"命令 0x{original:02X}")
                status_name = MULTI_STATUS_NAMES.get(status, f"状态码 0x{status:02X}")
                self.status_left.setText(
                    f"NodeId 0x{node.node_id:04X} 传感器已确认：{command_name}（{status_name}）"
                )
                if original == MULTI_CMD_SCAN_START and status == 0:
                    node.scan_state = "accepted"
                    if node.node_id == self.multi_selected_node_id:
                        self.multi_scan_info.setText(
                            f"CH{node.color_index + 1} 传感器已接受命令，等待扫描开始："
                            f"0 / {node.scan_expected} 点"
                        )
                identity, color = self.multi_node_log_identity(node.node_id)
                self.log_event(
                    "确认", "protocol",
                    f"传感器 ACK：{command_name}，{status_name}", identity, color
                )
            elif frame.kind == MULTI_EVT_NACK:
                reason = frame.payload[1] if len(frame.payload) >= 2 else 0
                original = frame.payload[0] if frame.payload else 0
                command_name = MULTI_COMMAND_NAMES.get(original, f"命令 0x{original:02X}")
                reason_name = MULTI_STATUS_NAMES.get(reason, f"原因码 0x{reason:02X}")
                self.status_left.setText(
                    f"NodeId 0x{node.node_id:04X} 未执行 {command_name}：{reason_name}"
                )
                if original == MULTI_CMD_SCAN_START:
                    node.scan_state = "failed"
                    if node.node_id == self.multi_selected_node_id:
                        self.multi_scan_info.setText(
                            f"CH{node.color_index + 1} 扫描未启动：{reason_name}"
                        )
                identity, color = self.multi_node_log_identity(node.node_id)
                self.log_event(
                    "错误", "problem", f"NACK：{command_name}，{reason_name}",
                    identity, color,
                )
        if frames:
            self.trim_multi_buffers()
            self.update_multi_node_table()
            self.multi_plot_dirty = True
        self.update_status()

    def update_multi_node_table(self):
        if not hasattr(self, "multi_node_table"):
            return
        nodes = sorted(self.multi_nodes.values(), key=lambda item: item.color_index)
        self.multi_node_table.blockSignals(True)
        self.multi_node_table.setRowCount(len(nodes))
        is_dc = self.dc_mode.isChecked()
        selected_row = -1
        for row, node in enumerate(nodes):
            color = QtGui.QColor(MULTI_NODE_COLORS[node.color_index])
            channel = QtWidgets.QTableWidgetItem(f"● CH{node.color_index + 1}")
            channel.setForeground(QtGui.QBrush(color))
            channel.setData(QtCore.Qt.UserRole, node.node_id)
            if node.link_state == 2:
                state_text = "发现中"
            else:
                state_text = "在线" if node.online else "离线"
            latest = "--"
            if node.rows:
                latest = f"{node.rows[-1][3]:.2f}" if is_dc else str(node.rows[-1][4])
            values = [channel, QtWidgets.QTableWidgetItem(state_text),
                      QtWidgets.QTableWidgetItem(f"0x{node.node_id:04X}"),
                      QtWidgets.QTableWidgetItem(str(node.frames)),
                      QtWidgets.QTableWidgetItem(latest)]
            for column, item in enumerate(values):
                self.multi_node_table.setItem(row, column, item)
            if node.node_id == self.multi_selected_node_id:
                selected_row = row
        if selected_row >= 0:
            self.multi_node_table.selectRow(selected_row)
        self.multi_node_table.blockSignals(False)
        online = sum(1 for node in nodes if node.online)
        self.multi_online_badge.setText(f"{online} / {MULTI_MAX_NODES} 在线")
        self.update_multi_target_label()

    def on_multi_node_selected(self):
        row = self.multi_node_table.currentRow()
        item = self.multi_node_table.item(row, 0) if row >= 0 else None
        if item is None:
            return
        self.multi_selected_node_id = item.data(QtCore.Qt.UserRole)
        self.update_multi_target_label()
        self.refresh_multi_scan_plot()
        self.multi_plot_dirty = True

    def update_multi_target_label(self):
        self.update_multi_relay_controls()
        node = self.multi_nodes.get(self.multi_selected_node_id)
        self.multi_release_button.setEnabled(
            self.connect_button.isChecked() and node is not None
            and (node.online or node.link_state == 2)
        )
        if node is None:
            self.multi_target_label.setText("控制目标：尚未发现节点")
            self.multi_target_label.setToolTip("")
            return
        state = "在线" if node.online else "离线"
        self.multi_target_label.setText(
            f"控制目标：CH{node.color_index + 1} · 0x{node.node_id:04X} · {state}"
        )
        self.multi_target_label.setToolTip(f"BLE 地址：{node.address}")

    def send_multi(self, frame_type, payload=b"", flags=0):
        if not self.connect_button.isChecked():
            self.show_error("请先连接数据中继串口。")
            return False
        node = self.multi_nodes.get(self.multi_selected_node_id)
        if node is None:
            self.show_error("请先在节点机架中选择一个传感器。")
            return False
        if not node.online and not (frame_type == MULTI_CMD_RELEASE_LINK and node.link_state == 2):
            self.show_error(f"NodeId 0x{node.node_id:04X} 当前不在线，未发送命令。")
            return False
        if frame_type == MULTI_CMD_RELAY_DISCHARGE:
            if payload or flags or not self.can_start_multi_relay(node):
                return False
            # Arm before emitting so even a synchronous reply targets this request.
            node.relay_level = None
            node.relay_phase = "waiting_ack"
            node.relay_pending_seq = self.multi_seq
            node.relay_pending_scan_id = self.multi_scan_id
            node.relay_deadline = time.monotonic() + MULTI_RELAY_ACK_TIMEOUT_S
            node.relay_detail = ""
        packet = multi_encode(
            frame_type, node.node_id, flags, self.multi_scan_id, self.multi_seq, payload
        )
        if frame_type == MULTI_CMD_RELEASE_LINK:
            self.multi_release_pending[node.node_id] = self.multi_seq
        self.request_write.emit(packet)
        identity, color = self.multi_node_log_identity(node.node_id)
        self.log_event(
            "信息", "protocol",
            f"发送命令 0x{frame_type:02X}，ScanId={self.multi_scan_id}，Seq={self.multi_seq}",
            identity, color,
        )
        self.multi_seq = (self.multi_seq + 1) & 0xFFFF
        return True

    @staticmethod
    def reset_multi_relay_state(node):
        """A connection transition invalidates our knowledge of the physical GPIO."""
        node.relay_level = None
        node.relay_phase = "unknown"
        node.relay_pending_seq = None
        node.relay_pending_scan_id = None
        node.relay_deadline = 0.0
        node.relay_detail = ""

    def can_start_multi_relay(self, node):
        return (
            self.is_multi_mode() and self.connect_button.isChecked()
            and node is not None and node.online and node.relay_level != 1
            and node.relay_phase not in ("waiting_ack", "waiting_state", "active", "busy")
        )

    def update_multi_relay_controls(self):
        node = self.multi_nodes.get(self.multi_selected_node_id)
        self.multi_relay_button.setEnabled(self.can_start_multi_relay(node))
        if node is None:
            self.multi_relay_status.setText("状态未知（尚未选择节点）")
            return
        prefix = f"CH{node.color_index + 1} · 0x{node.node_id:04X}："
        if not self.connect_button.isChecked():
            state = "状态未知（串口未连接）"
        elif not node.online:
            state = "状态未知（节点离线）"
        else:
            state = {
                "unknown": "状态未知（等待设备上报）",
                "waiting_ack": "等待确认（已发送 5 秒放电请求）",
                "waiting_state": "已确认，等待设备状态",
                "active": "正在放电：PD7 高电平（等待设备自动拉低）",
                "low": "PD7 低电平",
                "busy": "设备忙，等待状态反馈",
                "timeout": "状态未知（反馈超时，可重试）",
                "rejected": f"状态未知（请求被拒绝：{node.relay_detail}；可重试）",
            }.get(node.relay_phase, "状态未知")
            if node.relay_phase == "active" and node.relay_level == 0:
                state = "PD7 低电平；等待当前放电请求反馈"
        self.multi_relay_status.setText(prefix + state)

    def start_multi_relay_discharge(self):
        node = self.multi_nodes.get(self.multi_selected_node_id)
        if not self.can_start_multi_relay(node):
            self.update_multi_relay_controls()
            return False
        # Give each user request its own identifier; no automatic retries here.
        self.multi_scan_id = ((self.multi_scan_id + 1) & 0xFFFF) or 1
        if not self.send_multi(MULTI_CMD_RELAY_DISCHARGE):
            return False
        self.status_left.setText(f"已向 NodeId 0x{node.node_id:04X} 请求继电器放电 5 秒")
        self.update_multi_relay_controls()
        return True

    def on_multi_relay_reply(self, node, frame):
        identity, color = self.multi_node_log_identity(node.node_id)
        if len(frame.payload) != 2:
            self.log_event("警告", "problem", "丢弃继电器确认：载荷长度错误", identity, color)
            return
        if (frame.seq != node.relay_pending_seq
                or frame.scan_id != node.relay_pending_scan_id):
            self.log_event("信息", "protocol", "忽略非当前继电器请求的确认", identity, color)
            return
        status = frame.payload[1]
        status_name = MULTI_STATUS_NAMES.get(status, f"状态码 0x{status:02X}")
        if frame.kind == MULTI_EVT_ACK and status == 0:
            # An ACK confirms acceptance, never the current pin level or completion.
            if node.relay_level != 1:
                node.relay_phase = "waiting_state"
            node.relay_deadline = time.monotonic() + MULTI_RELAY_STATE_TIMEOUT_S
            message = "继电器请求已确认；等待设备状态反馈"
        else:
            node.relay_pending_seq = None
            node.relay_pending_scan_id = None
            node.relay_detail = status_name
            if node.relay_level == 1:
                node.relay_phase = "active"
                node.relay_deadline = time.monotonic() + MULTI_RELAY_STATE_TIMEOUT_S
            elif status == 0x06:
                node.relay_level = None
                node.relay_phase = "busy"
                node.relay_deadline = time.monotonic() + MULTI_RELAY_STATE_TIMEOUT_S
            else:
                node.relay_level = None
                node.relay_phase = "rejected"
                node.relay_deadline = 0.0
            message = f"继电器请求未执行：{status_name}；未重新触发或延长放电"
        self.log_event("确认" if status == 0 else "警告", "protocol", message, identity, color)
        self.update_multi_relay_controls()

    def on_multi_relay_state(self, node, frame):
        identity, color = self.multi_node_log_identity(node.node_id)
        if len(frame.payload) != 1 or frame.payload[0] not in (0, 1):
            self.log_event("警告", "problem", "丢弃继电器状态：应为单字节 0 或 1", identity, color)
            return
        if not node.online:
            # Late traffic must not revalidate state across a disconnect.
            return
        node.relay_level = frame.payload[0]
        matching_request = (
            node.relay_pending_seq is None or frame.scan_id == node.relay_pending_scan_id
        )
        if matching_request:
            node.relay_pending_seq = None
            node.relay_pending_scan_id = None
        if node.relay_level == 1:
            node.relay_phase = "active"
            node.relay_deadline = time.monotonic() + MULTI_RELAY_STATE_TIMEOUT_S
        elif matching_request:
            node.relay_phase = "low"
            node.relay_deadline = 0.0
        # A low report for an earlier request is useful telemetry but cannot
        # complete the new command while its acknowledgement is outstanding.
        message = "继电器 PD7 高电平，正在放电" if node.relay_level else "继电器 PD7 低电平"
        self.log_event("设备", "device", message, identity, color)
        self.update_multi_relay_controls()

    def check_multi_relay_timeouts(self):
        now = time.monotonic()
        changed = False
        for node in self.multi_nodes.values():
            if node.relay_deadline and now >= node.relay_deadline:
                node.relay_level = None
                node.relay_phase = "timeout"
                node.relay_pending_seq = None
                node.relay_pending_scan_id = None
                node.relay_deadline = 0.0
                identity, color = self.multi_node_log_identity(node.node_id)
                self.log_event(
                    "警告", "problem", "继电器反馈超时，当前电平未知；可手动重试，未自动重发",
                    identity, color,
                )
                changed = True
        if changed:
            self.update_multi_relay_controls()

    def release_multi_link(self):
        if self.send_multi(MULTI_CMD_RELEASE_LINK):
            self.status_left.setText("已请求释放选中设备；收到实际断开事件后才算腾出位置")

    def start_multi_dc_scan(self):
        if self.multi_dc_min.value() > self.multi_dc_max.value():
            self.show_error("DAC 起点不能大于终点。")
            return
        payload = struct.pack(
            "<HHHHB", self.multi_dc_min.value(), self.multi_dc_max.value(),
            self.multi_dc_points.value(), self.multi_dc_settle.value(),
            self.multi_dc_average.value(),
        )
        self.multi_scan_id = (self.multi_scan_id + 1) & 0xFFFF
        if self.send_multi(MULTI_CMD_SCAN_START, payload):
            node = self.multi_nodes[self.multi_selected_node_id]
            node.scan_points.clear()
            node.scan_expected = self.multi_dc_points.value()
            node.scan_state = "waiting_ack"
            node.scan_id = self.multi_scan_id
            self.multi_scan_curve.setData([], [])
            self.multi_scan_info.setText(
                f"CH{node.color_index + 1} 扫描命令已发送：0 / {self.multi_dc_points.value()} 点"
            )
            self.multi_plot_tabs.setCurrentIndex(1)

    def abort_multi_dc_scan(self):
        if self.send_multi(MULTI_CMD_ABORT):
            self.status_left.setText("已向选中节点发送终止扫描命令")

    def set_multi_dc_dac(self, save):
        value = self.multi_dc_value.value()
        flags = MULTI_SET_DAC_FLAG_SAVE if save else 0
        if self.send_multi(MULTI_CMD_SET_DAC, struct.pack("<H", value), flags):
            self.status_left.setText(f"已向选中节点设置 DAC={value}" + ("（保存）" if save else ""))

    def refresh_multi_scan_plot(self):
        node = self.multi_nodes.get(self.multi_selected_node_id)
        if node is None or not node.scan_points:
            self.multi_scan_curve.setData([], [])
            if node is None:
                self.multi_scan_info.setText("选择节点后可执行 DAC 扫描")
            elif node.scan_state == "waiting_ack":
                self.multi_scan_info.setText(
                    f"CH{node.color_index + 1} 等待中继/传感器确认：0 / {node.scan_expected} 点"
                )
            elif node.scan_state == "accepted":
                self.multi_scan_info.setText(
                    f"CH{node.color_index + 1} 传感器已接受命令，等待扫描开始："
                    f"0 / {node.scan_expected} 点"
                )
            elif node.scan_state == "executing":
                self.multi_scan_info.setText(
                    f"CH{node.color_index + 1} 传感器正在执行：0 / {node.scan_expected} 点"
                )
            return
        x = np.asarray([point[1] for point in node.scan_points], dtype=np.float64)
        y = np.asarray([point[2] for point in node.scan_points], dtype=np.float64)
        color = MULTI_NODE_COLORS[node.color_index]
        self.multi_scan_curve.setPen(pg.mkPen(color, width=1.6))
        self.multi_scan_curve.setSymbolBrush(pg.mkBrush(color))
        self.multi_scan_curve.setData(x, y)
        x_pad = max(1.0, float(np.ptp(x)) * 0.04)
        y_pad = max(1.0, float(np.ptp(y)) * 0.08)
        self.multi_scan_plot.setXRange(float(np.min(x) - x_pad), float(np.max(x) + x_pad), padding=0)
        self.multi_scan_plot.setYRange(float(np.min(y) - y_pad), float(np.max(y) + y_pad), padding=0)
        self.multi_scan_info.setText(
            f"CH{node.color_index + 1} · NodeId 0x{node.node_id:04X} · "
            f"扫描进行中 {len(node.scan_points)} / {node.scan_expected or '?'} 点"
        )

    def export_multi_dc_scan(self):
        node = self.multi_nodes.get(self.multi_selected_node_id)
        if node is None or not node.scan_points:
            self.show_error("选中节点当前没有 DAC 扫描点可导出。")
            return
        default = f"node_{node.node_id:04X}_dac_scan_{datetime.now():%Y%m%d_%H%M%S}.csv"
        path, _ = QtWidgets.QFileDialog.getSaveFileName(
            self, "导出选中节点 DAC 扫描点", str(Path.home() / default), "CSV 文件 (*.csv)"
        )
        if not path:
            return
        with open(path, "w", newline="", encoding="utf-8-sig") as handle:
            writer = csv.writer(handle)
            writer.writerow(["node_id", "index", "dac", "lockin"])
            for index, dac, lockin in node.scan_points:
                writer.writerow([f"0x{node.node_id:04X}", index, dac, f"{lockin:.2f}"])
        self.statusBar().showMessage(f"扫描点已保存：{path}", 6000)

    def send_dc(self, frame_type, payload=b"", flags=0):
        if not self.connect_button.isChecked():
            self.show_error("请先连接直流传感器串口。")
            return False
        self.request_write.emit(dc_encode(frame_type, flags, 1, self.dc_seq, payload))
        self.log_event(
            "信息", "protocol", f"发送直流 V1 命令 0x{frame_type:02X}，Seq={self.dc_seq}",
            "一对一传感器",
        )
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
        self.dc_scan_text.setText(f"序号 = {scan_index}\nDAC = {dac:.0f}\nLock-in = {lockin:.2f}")
        self.dc_scan_text.setPos(dac, lockin)
        for item in (self.dc_scan_v, self.dc_scan_h, self.dc_scan_text):
            item.show()
        self.dc_scan_info.setText(f"已选择：序号 {scan_index}，DAC = {dac:.0f}，Lock-in = {lockin:.2f}")

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
            writer.writerows((index, dac, f"{lockin:.2f}") for index, dac, lockin in self.dc_scan_points)
        self.statusBar().showMessage(f"扫描点已保存：{path}", 6000)

    def trim_multi_buffers(self):
        if self.multi_hold_data.isChecked() or self.show_all_data():
            return
        duration = self.duration_seconds()
        if duration is None:
            return
        cutoff = time.time() - duration
        for node in self.multi_nodes.values():
            while node.rows and node.rows[0][0] < cutoff:
                node.rows.popleft()

    def refresh_multi_plot(self):
        if not self.multi_plot_dirty or self.multi_paused:
            return
        selected = self.multi_nodes.get(self.multi_selected_node_id)
        if self.multi_show_all.isChecked():
            visible_ids = {node.node_id for node in self.multi_nodes.values() if node.online}
        else:
            visible_ids = {selected.node_id} if selected is not None else set()
        duration = self.duration_seconds()
        cutoff = time.time() - duration if duration is not None else None
        base = self.multi_start_time
        time_scale, time_unit, _decimals = self.time_display_settings()
        self.multi_plot.setLabel("bottom", "相对采集时间", units=time_unit)
        minimum_x = None
        maximum_x = None
        selected_values = np.empty(0)
        is_dc = self.dc_mode.isChecked()
        for node_id, curve in self.multi_curves.items():
            node = self.multi_nodes.get(node_id)
            show = node is not None and node_id in visible_ids
            curve.setVisible(show)
            if not show:
                continue
            rows = list(node.rows)
            if cutoff is not None:
                rows = [row for row in rows if row[0] >= cutoff]
            if not rows:
                curve.setData([], [])
                continue
            if base is None:
                base = rows[0][0]
            x = np.asarray([(row[0] - base) * time_scale for row in rows], dtype=np.float64)
            values = np.asarray(
                [row[3] if is_dc else row[4] for row in rows], dtype=np.float64
            )
            curve.setData(x, values, skipFiniteCheck=True)
            minimum_x = float(x[0]) if minimum_x is None else min(minimum_x, float(x[0]))
            maximum_x = float(x[-1]) if maximum_x is None else max(maximum_x, float(x[-1]))
            if node_id == self.multi_selected_node_id:
                selected_values = values
        if minimum_x is not None:
            right = maximum_x if maximum_x > minimum_x else minimum_x + 1.0
            self.multi_plot.setXRange(minimum_x, right, padding=0)
            if self.multi_auto_y.isChecked():
                self.multi_plot.enableAutoRange(axis=pg.ViewBox.YAxis, enable=True)
        self.update_multi_stats(selected_values)
        self.multi_plot_dirty = False

    def update_multi_stats(self, values):
        values = np.asarray(values, dtype=np.float64)
        self.multi_last_stats_values = values.copy()
        if values.size == 0:
            latest = pp = rms = mean = minimum = maximum = "--"
        else:
            is_lockin = self.dc_mode.isChecked()
            decimals = 2 if is_lockin else 3
            latest = f"{values[-1]:.2f}" if is_lockin else f"{int(round(values[-1]))}"
            pp = f"{np.ptp(values):.2f}" if is_lockin else f"{int(np.ptp(values))}"
            rms = f"{np.sqrt(np.mean(np.square(values))):.{decimals}f}"
            mean = f"{np.mean(values):.{decimals}f}"
            minimum = f"{np.min(values):.2f}" if is_lockin else f"{int(np.min(values))}"
            maximum = f"{np.max(values):.2f}" if is_lockin else f"{int(np.max(values))}"
        self.multi_latest_label.setText(f"最新值：{latest}")
        self.multi_pp_label.setText(f"峰峰值：{pp}")
        self.multi_rms_label.setText(f"有效值：{rms}")
        self.multi_mean_label.setText(f"平均值：{mean}")
        self.multi_range_label.setText(f"最小/最大：{minimum} / {maximum}")
        self.multi_samples_label.setText(f"窗口点数：{values.size:,}")

    def mark_multi_dirty(self, _value=None):
        self.multi_plot_dirty = True

    def toggle_multi_pause(self, checked):
        self.multi_paused = checked
        self.multi_pause_button.setText("继续显示" if checked else "暂停显示")
        if not checked:
            self.multi_plot_dirty = True

    def on_multi_auto_y(self, checked):
        self.multi_plot.enableAutoRange(axis=pg.ViewBox.YAxis, enable=checked)
        if checked:
            self.multi_plot.autoRange()

    def clear_multi_data(self, reset_nodes=False):
        if not hasattr(self, "multi_plot"):
            return
        self.multi_parser.clear()
        self.multi_start_time = None
        self.multi_relay_stats = None
        if reset_nodes:
            for curve in self.multi_curves.values():
                self.multi_plot.removeItem(curve)
            legend = self.multi_plot.plotItem.legend
            if legend is not None:
                legend.clear()
            self.multi_nodes.clear()
            self.multi_archived_nodes.clear()
            self.multi_release_pending.clear()
            self.multi_curves.clear()
            self.multi_selected_node_id = None
        else:
            for node in (*self.multi_nodes.values(), *self.multi_archived_nodes.values()):
                node.rows.clear()
                node.scan_points.clear()
                node.scan_expected = 0
                node.scan_state = "idle"
                node.scan_id = 0
                curve = self.multi_curves.get(node.node_id)
                if curve is not None:
                    curve.setData([], [])
        self.multi_scan_curve.setData([], [])
        self.multi_scan_info.setText("选择节点后可执行 DAC 扫描")
        self.update_multi_stats(np.empty(0))
        self.update_multi_node_table()
        self.multi_plot_dirty = True
        if self.is_multi_mode():
            self.total_samples = 0
            self.total_bytes = 0
            self.update_status()

    def save_multi_data(self):
        rows = []
        for node in sorted((*self.multi_nodes.values(), *self.multi_archived_nodes.values()),
                           key=lambda item: (item.color_index, item.node_id)):
            rows.extend((node.node_id, node.color_index + 1, *row) for row in node.rows)
        if not rows:
            self.show_error("当前没有可保存的多节点数据。")
            return
        rows.sort(key=lambda row: row[2])
        mode = "dc" if self.dc_mode.isChecked() else "ac"
        default = f"multi_{mode}_sensor_data_{datetime.now():%Y%m%d_%H%M%S}.csv"
        path, _ = QtWidgets.QFileDialog.getSaveFileName(
            self, "导出多节点数据", str(Path.home() / default), "CSV 文件 (*.csv)"
        )
        if not path:
            return
        start = rows[0][2]
        try:
            with open(path, "w", newline="", encoding="utf-8-sig") as handle:
                writer = csv.writer(handle)
                writer.writerow([
                    "node_id", "channel", "timestamp", "elapsed_s", "seq",
                    "dac", "lockin", "adc", "display_field", "display_value",
                ])
                for node_id, channel, timestamp, seq, dac, lockin, adc in rows:
                    display_field = "lockin" if self.dc_mode.isChecked() else "adc"
                    display_value = f"{lockin:.2f}" if self.dc_mode.isChecked() else adc
                    writer.writerow([
                        f"0x{node_id:04X}", channel, f"{timestamp:.6f}",
                        f"{timestamp - start:.6f}", seq, dac, f"{lockin:.2f}", adc,
                        display_field, display_value,
                    ])
            self.statusBar().showMessage(f"多节点数据已保存：{path}", 6000)
        except Exception as exc:
            self.show_error(f"保存失败：{exc}")

    def trim_buffer(self):
        if self.is_multi_mode():
            self.trim_multi_buffers()
            return
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
        if self.is_multi_mode():
            self.refresh_multi_plot()
            return
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
            is_lockin = self.dc_mode.isChecked()
            decimals = 2 if is_lockin else 3
            pp = f"{np.ptp(values):.2f}" if is_lockin else f"{int(np.ptp(values))}"
            rms = f"{np.sqrt(np.mean(np.square(values))):.{decimals}f}"
            mean_value = np.mean(values)
            mean = f"{mean_value:.{decimals}f}"
            ac_values = values - mean_value
            ac_mean = f"{np.mean(ac_values):.{decimals}f}"
            ac_rms = f"{np.sqrt(np.mean(np.square(ac_values))):.{decimals}f}"
            minimum = f"{np.min(values):.2f}" if is_lockin else f"{int(np.min(values))}"
            maximum = f"{np.max(values):.2f}" if is_lockin else f"{int(np.max(values))}"
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
        value_text = f"{py:.2f}" if self.dc_mode.isChecked() else str(int(round(py)))
        self.cursor_text.setText(
            f"时间 = {px:.{decimals}f} {unit}\n十进制值 = {value_text}"
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
        if self.is_multi_mode():
            self.multi_plot_dirty = True
        else:
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
                        writer.writerow([f"{timestamp:.6f}", f"{timestamp-start:.6f}", seq, dac, f"{lockin:.2f}", adc])
                else:
                    writer.writerow(["sample_index", "time_s", "value"])
                    for offset, value in enumerate(data):
                        index = start_index + offset
                        writer.writerow([index, f"{index / rate:.9f}", str(int(round(value)))])
            self.statusBar().showMessage(f"数据已保存：{path}", 6000)
        except Exception as exc:
            self.show_error(f"保存失败：{exc}")

    def update_status(self):
        if self.is_multi_mode():
            online = sum(1 for node in self.multi_nodes.values() if node.online)
            readings = sum(len(node.rows) for node in self.multi_nodes.values())
            scan_points = sum(len(node.scan_points) for node in self.multi_nodes.values())
            self.status_right.setStyleSheet("")
            text = (
                f"V2 有效帧 {self.multi_parser.valid_frames} | 在线 {online}/{MULTI_MAX_NODES} | "
                f"接收 {self.total_bytes:,} B | 队列读数 {readings:,} | "
                f"扫描点 {scan_points:,} | 丢弃 {self.multi_parser.discarded_bytes:,} B"
            )
            if self.multi_relay_stats is not None:
                links, ble_depth, pc_free, duplicates = self.multi_relay_stats
                text += (
                    f" | 中继链路 {links} | BLE队列 {ble_depth} | PC空闲槽 {pc_free} | 去重 {duplicates}"
                )
            self.status_right.setText(text)
            return
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
        self.debug_log_window.hide()
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
