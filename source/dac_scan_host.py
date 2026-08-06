#!/usr/bin/env python3
"""PC host utility for the DAC wireless scan receiver.

The script can run a protocol self-test without external dependencies:

    python tools/dac_scan_host.py --self-test

Real serial I/O requires pyserial:

    python tools/dac_scan_host.py --port COM8 --scan
    python tools/dac_scan_host.py --port COM8 --set-dac 32768

When launched from VSCode without command-line arguments, the script starts an
interactive prompt.
"""

from __future__ import annotations

import argparse
import struct
import sys
import time
from dataclasses import dataclass
from typing import Iterable, Optional


MAGIC = b"\xA5\x5A"
VERSION = 0x01
MAX_PAYLOAD = 246

CMD_SCAN_START = 0x10
CMD_SET_DAC = 0x11
CMD_ABORT = 0x12
CMD_PD2_PULSE = 0x13

EVT_ACK = 0x80
EVT_NACK = 0x81
EVT_SCAN_BEGIN = 0x90
EVT_SCAN_POINTS = 0x91
EVT_SCAN_END = 0x92
EVT_SENSOR_READINGS = 0x93
EVT_BLE_RAW = 0xA0

STATUS = {
    0x00: "OK",
    0x01: "CRC_ERROR",
    0x02: "BAD_VERSION",
    0x03: "BAD_LENGTH",
    0x04: "UNKNOWN_TYPE",
    0x05: "NO_LINK",
    0x06: "BUSY",
    0x07: "UNSUPPORTED",
    0x08: "RX_OVERFLOW",
}

TYPE_NAMES = {
    CMD_SCAN_START: "CMD_SCAN_START",
    CMD_SET_DAC: "CMD_SET_DAC",
    CMD_ABORT: "CMD_ABORT",
    CMD_PD2_PULSE: "CMD_PD2_PULSE",
    EVT_ACK: "EVT_ACK",
    EVT_NACK: "EVT_NACK",
    EVT_SCAN_BEGIN: "EVT_SCAN_BEGIN",
    EVT_SCAN_POINTS: "EVT_SCAN_POINTS",
    EVT_SCAN_END: "EVT_SCAN_END",
    EVT_SENSOR_READINGS: "EVT_SENSOR_READINGS",
    EVT_BLE_RAW: "EVT_BLE_RAW",
}

FLAG_SAVE_DAC = 0x01


@dataclass
class Frame:
    frame_type: int
    flags: int
    scan_id: int
    seq: int
    payload: bytes


def crc16_ccitt(data: bytes) -> int:
    crc = 0xFFFF
    for byte in data:
        crc ^= byte << 8
        for _ in range(8):
            if crc & 0x8000:
                crc = ((crc << 1) ^ 0x1021) & 0xFFFF
            else:
                crc = (crc << 1) & 0xFFFF
    return crc


def encode_frame(frame_type: int, flags: int = 0, scan_id: int = 0, seq: int = 0, payload: bytes = b"") -> bytes:
    if len(payload) > MAX_PAYLOAD:
        raise ValueError(f"payload too large: {len(payload)} > {MAX_PAYLOAD}")
    header = struct.pack("<BBBHHB", VERSION, frame_type, flags, scan_id & 0xFFFF, seq & 0xFFFF, len(payload))
    crc = crc16_ccitt(header + payload)
    return MAGIC + header + payload + struct.pack("<H", crc)


class FrameParser:
    def __init__(self) -> None:
        self._buf = bytearray()

    def feed(self, data: bytes) -> Iterable[Frame]:
        self._buf.extend(data)
        frames = []
        while True:
            start = self._buf.find(MAGIC)
            if start < 0:
                if self._buf and self._buf[-1] == MAGIC[0]:
                    self._buf[:] = self._buf[-1:]
                    break
                self._buf.clear()
                break
            if start:
                del self._buf[:start]
            if len(self._buf) < 12:
                break
            version, frame_type, flags, scan_id, seq, payload_len = struct.unpack_from("<BBBHHB", self._buf, 2)
            total_len = 2 + 8 + payload_len + 2
            if payload_len > MAX_PAYLOAD:
                del self._buf[0]
                continue
            if len(self._buf) < total_len:
                break
            payload = bytes(self._buf[10 : 10 + payload_len])
            received_crc = struct.unpack_from("<H", self._buf, 10 + payload_len)[0]
            calc_crc = crc16_ccitt(bytes(self._buf[2 : 10 + payload_len]))
            del self._buf[:total_len]
            if version != VERSION or received_crc != calc_crc:
                continue
            frames.append(Frame(frame_type, flags, scan_id, seq, payload))
        return frames


def build_scan_start(scan_id: int, seq: int, dac_min: int, dac_max: int, points: int, settle_ms: int, avg_blocks: int) -> bytes:
    payload = struct.pack("<HHHHB", dac_min, dac_max, points, settle_ms, avg_blocks)
    return encode_frame(CMD_SCAN_START, 0, scan_id, seq, payload)


def build_set_dac(scan_id: int, seq: int, dac_code: int, save: bool = True) -> bytes:
    payload = struct.pack("<H", dac_code)
    flags = FLAG_SAVE_DAC if save else 0
    return encode_frame(CMD_SET_DAC, flags, scan_id, seq, payload)


def build_abort(scan_id: int, seq: int) -> bytes:
    return encode_frame(CMD_ABORT, 0, scan_id, seq)


def parse_sensor_readings(payload: bytes) -> list[tuple[int, int, int, int]]:
    """Return readings as (seq, dac, lockin, adc_raw)."""
    if not payload:
        return []
    count = payload[0]
    readings: list[tuple[int, int, int, int]] = []
    offset = 1
    for _ in range(count):
        if offset + 8 > len(payload):
            break
        sample_seq, dac_code, lockin, adc_raw = struct.unpack_from("<HHHH", payload, offset)
        readings.append((sample_seq, dac_code, lockin, adc_raw))
        offset += 8
    return readings


def describe_frame(frame: Frame) -> str:
    name = TYPE_NAMES.get(frame.frame_type, f"0x{frame.frame_type:02X}")
    if frame.frame_type in (EVT_ACK, EVT_NACK) and len(frame.payload) >= 2:
        target = TYPE_NAMES.get(frame.payload[0], f"0x{frame.payload[0]:02X}")
        status = STATUS.get(frame.payload[1], f"0x{frame.payload[1]:02X}")
        detail = ""
        if frame.payload[0] == CMD_SET_DAC and len(frame.payload) >= 5:
            dac_code = struct.unpack_from("<H", frame.payload, 2)[0]
            set_flags = frame.payload[4]
            save_text = "save=yes" if (set_flags & FLAG_SAVE_DAC) else "save=no"
            detail = f" dac={dac_code} {save_text}"
        return f"{name}: target={target} status={status}{detail} scan_id={frame.scan_id} seq={frame.seq}"
    if frame.frame_type == EVT_SCAN_POINTS and frame.payload:
        count = frame.payload[0]
        points = []
        offset = 1
        for _ in range(count):
            if offset + 5 > len(frame.payload):
                break
            index = frame.payload[offset]
            dac_code, lockin = struct.unpack_from("<HH", frame.payload, offset + 1)
            points.append((index, dac_code, lockin))
            offset += 5
        preview = ", ".join(f"#{i}:dac={d}:lockin={l}" for i, d, l in points[:5])
        suffix = " ..." if len(points) > 5 else ""
        return f"{name}: count={len(points)} {preview}{suffix}"
    if frame.frame_type == EVT_SENSOR_READINGS and frame.payload:
        readings = parse_sensor_readings(frame.payload)
        preview = ", ".join(
            f"seq={seq}:dac={dac}:lockin={lockin}:adc={adc}" for seq, dac, lockin, adc in readings[:3]
        )
        suffix = " ..." if len(readings) > 3 else ""
        return f"{name}: count={len(readings)} {preview}{suffix}"
    if frame.frame_type == EVT_BLE_RAW:
        if frame.payload and all((32 <= byte <= 126) or byte in (9, 10, 13) for byte in frame.payload):
            text = frame.payload.decode("ascii", errors="replace")
            return f"{name}: text={text}"
        return f"{name}: {len(frame.payload)} bytes raw={frame.payload[:16].hex(' ')}"
    return f"{name}: flags=0x{frame.flags:02X} scan_id={frame.scan_id} seq={frame.seq} payload={frame.payload.hex(' ')}"


def require_serial():
    try:
        import serial  # type: ignore
    except ImportError as exc:
        raise SystemExit("pyserial is required for --port mode. Install it with: python -m pip install pyserial") from exc
    return serial


def list_serial_ports() -> list[tuple[str, str]]:
    serial = require_serial()
    from serial.tools import list_ports  # type: ignore

    return [(port.device, port.description) for port in list_ports.comports()]


def prompt_int(prompt: str, default: int, minimum: int = 0, maximum: int = 0xFFFF) -> int:
    while True:
        value = input(f"{prompt} [{default}]: ").strip()
        if not value:
            return default
        try:
            number = int(value, 0)
        except ValueError:
            print("请输入十进制数，或 0x 开头的十六进制数。")
            continue
        if minimum <= number <= maximum:
            return number
        print(f"请输入 {minimum} 到 {maximum} 范围内的数值。")


def interactive_mode() -> int:
    print("DAC wireless scan host")
    print("无命令行参数启动，进入交互模式。")
    print()

    try:
        ports = list_serial_ports()
    except SystemExit:
        print("当前 Python 环境缺少 pyserial，暂时不能打开串口。")
        print("安装命令：D:\\anaconda\\envs\\dielectric_env\\python.exe -m pip install pyserial")
        answer = input("是否先运行协议自测？[Y/n]: ").strip().lower()
        if answer in ("", "y", "yes"):
            return self_test()
        return 2

    if ports:
        print("检测到串口：")
        for index, (device, description) in enumerate(ports, start=1):
            print(f"  {index}. {device}  {description}")
        default_port = ports[0][0]
    else:
        print("未自动检测到串口，可以手动输入，例如 COM8。")
        default_port = "COM8"

    port_input = input(f"串口号 [{default_port}]: ").strip()
    if port_input.isdigit() and ports:
        selected = int(port_input)
        if 1 <= selected <= len(ports):
            port = ports[selected - 1][0]
        else:
            print("串口序号超出范围。")
            return 2
    else:
        port = port_input or default_port

    print()
    print("操作模式：")
    print("  1. 只监听接收器输出")
    print("  2. 发送扫描开始命令")
    print("  3. 发送 DAC 设置命令")
    print("  4. 发送终止命令")
    print("  5. 协议自测")
    mode = input("请选择 [1]: ").strip() or "1"

    args = argparse.Namespace(
        port=port,
        baud=115200,
        timeout=3.0,
        listen=False,
        scan=False,
        set_dac=None,
        abort=False,
        scan_id=1,
        seq=1,
        dac_min=0,
        dac_max=0xFFFF,
        points=100,
        settle_ms=20,
        avg_blocks=1,
    )

    if mode == "1":
        args.listen = True
    elif mode == "2":
        args.scan = True
        args.dac_min = prompt_int("DAC 起点", args.dac_min)
        args.dac_max = prompt_int("DAC 终点", args.dac_max)
        args.points = prompt_int("扫描点数", args.points, 2, 255)
        args.settle_ms = prompt_int("每点等待时间 ms", args.settle_ms, 0, 1000)
        args.avg_blocks = prompt_int("平均次数", args.avg_blocks, 1, 255)
    elif mode == "3":
        args.set_dac = prompt_int("DAC 码值", 32768)
    elif mode == "4":
        args.abort = True
    elif mode == "5":
        return self_test()
    else:
        print("未知操作模式。")
        return 2

    return run_serial(args)


def run_serial(args: argparse.Namespace) -> int:
    serial = require_serial()
    parser = FrameParser()
    seq = args.seq

    with serial.Serial(args.port, args.baud, timeout=0.05) as ser:
        print(f"opened {args.port} at {args.baud} baud")
        if args.scan:
            frame = build_scan_start(args.scan_id, seq, args.dac_min, args.dac_max, args.points, args.settle_ms, args.avg_blocks)
            ser.write(frame)
            print(f"tx CMD_SCAN_START seq={seq} bytes={len(frame)}")
            seq += 1
        if args.set_dac is not None:
            frame = build_set_dac(args.scan_id, seq, args.set_dac)
            ser.write(frame)
            print(f"tx CMD_SET_DAC dac={args.set_dac} seq={seq} bytes={len(frame)}")
            seq += 1
        if args.abort:
            frame = build_abort(args.scan_id, seq)
            ser.write(frame)
            print(f"tx CMD_ABORT seq={seq} bytes={len(frame)}")

        deadline: Optional[float] = None if args.listen else time.time() + args.timeout
        while deadline is None or time.time() < deadline:
            data = ser.read(512)
            if not data:
                continue
            for frame in parser.feed(data):
                print("rx", describe_frame(frame))
    return 0


def self_test() -> int:
    parser = FrameParser()
    frames = [
        build_scan_start(1, 1, 0, 0xFFFF, 100, 20, 1),
        build_set_dac(1, 2, 32768),
        encode_frame(EVT_ACK, 0, 1, 2, bytes([CMD_SET_DAC, 0])),
        encode_frame(EVT_SENSOR_READINGS, 0, 0, 1, struct.pack("<BHHHH", 1, 1, 32768, 1234, 2048)),
    ]
    stream = b"\x00noise" + b"".join(frames)
    parsed = list(parser.feed(stream[:7])) + list(parser.feed(stream[7:]))
    assert len(parsed) == 4, len(parsed)
    assert parsed[0].frame_type == CMD_SCAN_START
    assert parsed[1].flags & FLAG_SAVE_DAC
    assert parsed[2].payload == bytes([CMD_SET_DAC, 0])
    assert parse_sensor_readings(parsed[3].payload) == [(1, 32768, 1234, 2048)]
    for frame in parsed:
        print("self-test", describe_frame(frame))
    print("self-test OK")
    return 0


def parse_args(argv: list[str]) -> argparse.Namespace:
    parser = argparse.ArgumentParser(description="DAC wireless scan PC host utility")
    parser.add_argument("--self-test", action="store_true", help="run protocol self-test without serial")
    parser.add_argument("--port", help="serial port, for example COM8")
    parser.add_argument("--baud", type=int, default=115200)
    parser.add_argument("--timeout", type=float, default=3.0)
    parser.add_argument("--listen", action="store_true", help="keep reading until interrupted")
    parser.add_argument("--scan", action="store_true", help="send CMD_SCAN_START")
    parser.add_argument("--set-dac", type=lambda value: int(value, 0), help="send CMD_SET_DAC with immediate save")
    parser.add_argument("--abort", action="store_true", help="send CMD_ABORT")
    parser.add_argument("--scan-id", type=lambda value: int(value, 0), default=1)
    parser.add_argument("--seq", type=lambda value: int(value, 0), default=1)
    parser.add_argument("--dac-min", type=lambda value: int(value, 0), default=0)
    parser.add_argument("--dac-max", type=lambda value: int(value, 0), default=0xFFFF)
    parser.add_argument("--points", type=int, default=100)
    parser.add_argument("--settle-ms", type=int, default=20)
    parser.add_argument("--avg-blocks", type=int, default=1)
    return parser.parse_args(argv)


def main(argv: list[str]) -> int:
    if not argv:
        from dac_scan_gui import main as gui_main

        return gui_main([])
    args = parse_args(argv)
    if args.self_test:
        return self_test()
    if not args.port:
        raise SystemExit("--port is required unless --self-test is used")
    return run_serial(args)


if __name__ == "__main__":
    raise SystemExit(main(sys.argv[1:]))
