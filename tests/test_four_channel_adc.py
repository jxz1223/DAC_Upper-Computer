"""Regression checks for the STM32WB four-channel ADC UART/BLE protocol."""

import csv
import io
import os
from pathlib import Path
import sys
import unittest
from unittest.mock import patch


os.environ.setdefault("QT_QPA_PLATFORM", "offscreen")
sys.path.insert(0, str(Path(__file__).resolve().parents[1]))
import sensor_waveform_viewer as viewer


class CsvBuffer(io.StringIO):
    def close(self):
        # Keep the text available after the viewer's context manager exits.
        pass


def make_wave(sequence=42, first_scan=1234):
    header = (
        b"\xA6\x6A\x02\x4C" + sequence.to_bytes(2, "big")
        + (2000).to_bytes(2, "big") + first_scan.to_bytes(4, "big")
    )
    samples = b"".join(
        (((index % 4) << 19) | ((index % 8) << 21) | (262144 + index)).to_bytes(3, "big")
        for index in range(76)
    )
    content = header + samples + bytes(4)
    return content + viewer.dc_crc16(content).to_bytes(2, "big")


def make_uart(payload):
    header = b"\x01\xA0\x00\x00\x00\x01\x00" + bytes([len(payload)])
    return b"\xA5\x5A" + header + payload + viewer.dc_crc16(header + payload).to_bytes(2, "little")


class FourChannelProtocolTests(unittest.TestCase):
    def test_crc_reference_vector(self):
        self.assertEqual(viewer.dc_crc16(b"123456789"), 0x29B1)

    def test_nested_frames_survive_split_reads(self):
        parser = viewer.FourChannelFrameParser()
        packet = make_uart(make_wave())
        frames = []
        for offset in range(0, len(packet), 17):
            frames.extend(parser.feed(packet[offset:offset + 17]))
        self.assertEqual(len(frames), 1)
        frame = frames[0]
        self.assertEqual((frame.sequence, frame.sample_rate, frame.first_scan), (42, 2000, 1234))
        self.assertEqual(frame.samples[:4], (
            (0, 262144, 0), (1, 262145, 1), (2, 262146, 2), (3, 262147, 3),
        ))
        self.assertEqual(len(frame.samples), 76)

    def test_outer_crc_error_resynchronizes_to_next_frame(self):
        bad = bytearray(make_uart(make_wave(42)))
        bad[40] ^= 1
        parser = viewer.FourChannelFrameParser()
        result = parser.feed(b"noise" + bad + make_uart(make_wave(43)))
        self.assertEqual([frame.sequence for frame in result], [43])
        self.assertEqual(parser.outer_crc_errors, 1)

    def test_inner_crc_and_channel_tag_are_rejected(self):
        bad_crc = bytearray(make_wave())
        bad_crc[20] ^= 1
        parser = viewer.FourChannelFrameParser()
        self.assertEqual(parser.feed(make_uart(bad_crc)), [])
        self.assertEqual(parser.wave_errors, 1)

        bad_tag = bytearray(make_wave())
        bad_tag[12] |= 0x08
        bad_tag[-2:] = viewer.dc_crc16(bad_tag[:-2]).to_bytes(2, "big")
        self.assertEqual(parser.feed(make_uart(bad_tag)), [])
        self.assertEqual(parser.wave_errors, 2)


class FourChannelUiTests(unittest.TestCase):
    @classmethod
    def setUpClass(cls):
        cls.app = viewer.QtWidgets.QApplication.instance() or viewer.QtWidgets.QApplication([])

    def setUp(self):
        self.window = viewer.MainWindow()
        self.window.plot_timer.stop()
        self.window.relay_timer.stop()
        self.window.four_channel_mode.setChecked(True)

    def tearDown(self):
        self.window.close()
        self.app.processEvents()

    def test_four_channel_mode_routes_samples_to_four_plots_and_status(self):
        self.assertTrue(self.window.is_four_channel_mode())
        self.assertEqual(self.window.baud_combo.currentText(), "2000000")
        self.assertEqual(self.window.realtime_stack.currentIndex(), 1)
        self.assertFalse(self.window.dc_mode.isEnabled())

        self.window.on_bytes(make_uart(make_wave(sequence=7, first_scan=100)))
        self.assertEqual([len(rows) for rows in self.window.four_channel_rows], [19, 19, 19, 19])
        self.assertEqual(self.window.four_channel_rate, 2000)
        self.assertEqual(self.window.four_channel_rows[2][0], (100, 262146, 2))
        self.assertIn("四路有效帧 1", self.window.status_right.text())

        self.window.refresh_four_channel_plot()
        x, y = self.window.four_channel_curves[0].getData()
        self.assertEqual(len(x), 19)
        self.assertEqual(int(y[-1]), 262144 + 72)
        self.assertIn("增益 ×1", self.window.four_channel_info_labels[0].text())

    def test_four_channel_csv_schema_keeps_channel_and_gain(self):
        self.window.on_bytes(make_uart(make_wave()))
        output = CsvBuffer()
        with patch.object(viewer.QtWidgets.QFileDialog, "getSaveFileName", return_value=("test.csv", "")):
            with patch("builtins.open", return_value=output):
                self.window.save_data()
        rows = list(csv.DictReader(io.StringIO(output.getvalue())))
        self.assertEqual(len(rows), 76)
        self.assertEqual(
            list(rows[0]),
            ["channel", "amplifier", "sample_index", "time_s", "normalized_value", "gain_code", "gain"],
        )
        self.assertEqual(rows[0], {
            "channel": "1", "amplifier": "U4", "sample_index": "1234", "time_s": "0.000000000",
            "normalized_value": "262144", "gain_code": "0", "gain": "1",
        })


if __name__ == "__main__":
    unittest.main()
