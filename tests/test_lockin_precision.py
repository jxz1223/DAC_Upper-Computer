"""Regression checks for fractional lock-in readings and long display windows.

Run: .build-venv-multi/Scripts/python.exe tests/test_lockin_precision.py
"""
import csv
import io
import os
from pathlib import Path
import struct
import sys
import unittest
from unittest.mock import patch

os.environ.setdefault("QT_QPA_PLATFORM", "offscreen")
sys.path.insert(0, str(Path(__file__).resolve().parents[1]))
import sensor_waveform_viewer as viewer


class CsvBuffer(io.StringIO):
    def close(self):
        # Keep exported text inspectable without writing files.
        pass


class LockinPrecisionTests(unittest.TestCase):
    @classmethod
    def setUpClass(cls):
        cls.app = viewer.QtWidgets.QApplication.instance() or viewer.QtWidgets.QApplication([])

    def setUp(self):
        self.window = viewer.MainWindow()
        self.window.multi_mode.setChecked(True)
        self.window.dc_mode.setChecked(True)
        self.window.plot_timer.stop()

    def tearDown(self):
        self.window.close()
        self.app.processEvents()

    def feed(self, kind, payload, flags=0, node=0x1234):
        frame = viewer.multi_encode(kind, node, flags=flags, payload=payload)
        # Check framing across arbitrary serial chunks too.
        for offset in range(0, len(frame), 3):
            self.window.on_multi_bytes(frame[offset:offset + 3])
        return self.window.multi_nodes[node]

    def export_rows(self, callback):
        data = CsvBuffer()
        with patch.object(viewer.QtWidgets.QFileDialog, "getSaveFileName", return_value=("test.csv", "")):
            with patch("builtins.open", return_value=data):
                callback()
        return list(csv.DictReader(io.StringIO(data.getvalue())))

    def test_high_precision_realtime_is_not_truncated_or_scaled_twice(self):
        values = [0, 1, 12345, 6553501, 0xFFFFFFFF]
        payload = bytes([len(values)]) + b"".join(
            struct.pack("<HHIH", i, 3000 + i, value, 2000 + i)
            for i, value in enumerate(values)
        )
        node = self.feed(viewer.MULTI_EVT_SENSOR_READINGS, payload, viewer.MULTI_FLAG_LOCKIN_CENTI)
        self.assertEqual([row[1] for row in node.rows], list(range(len(values))))
        self.assertEqual([row[2] for row in node.rows], list(range(3000, 3005)))
        self.assertEqual([row[4] for row in node.rows], list(range(2000, 2005)))
        for row, value in zip(node.rows, values):
            self.assertAlmostEqual(row[3], value / 100.0)
        self.window.refresh_multi_plot()
        self.assertEqual(self.window.multi_latest_label.text(), "最新值：42949672.95")
        self.assertEqual(self.window.multi_node_table.item(0, 4).text(), "42949672.95")
        self.assertEqual(self.window.total_samples, len(values))

    def test_old_and_new_v2_nodes_can_coexist(self):
        old = self.feed(viewer.MULTI_EVT_SENSOR_READINGS, struct.pack("<BHHHH", 1, 5, 300, 123, 456))
        new = self.feed(viewer.MULTI_EVT_SENSOR_READINGS, struct.pack("<BHHIH", 1, 6, 301, 12345, 457),
                        viewer.MULTI_FLAG_LOCKIN_CENTI, node=0x2345)
        self.assertEqual(old.rows[-1][1:], (5, 300, 123, 456))
        self.assertEqual(new.rows[-1][1:], (6, 301, 123.45, 457))
        self.assertEqual(self.window.multi_node_table.item(0, 4).text(), "123.00")
        self.assertEqual(self.window.multi_node_table.item(1, 4).text(), "123.45")

    def test_scans_keep_fractional_and_legacy_values(self):
        node = self.feed(viewer.MULTI_EVT_SCAN_POINTS,
                         b"\x02" + struct.pack("<BHI", 0, 100, 12345) + struct.pack("<BHI", 1, 200, 12346),
                         viewer.MULTI_FLAG_LOCKIN_CENTI)
        self.feed(viewer.MULTI_EVT_SCAN_POINTS, struct.pack("<BBHH", 1, 2, 300, 124))
        self.assertEqual(node.scan_points, [(0, 100, 123.45), (1, 200, 123.46), (2, 300, 124)])
        _, y = self.window.multi_scan_curve.getData()
        viewer.np.testing.assert_allclose(y, [123.45, 123.46, 124.00])
        rows = self.export_rows(self.window.export_multi_dc_scan)
        self.assertEqual([row["lockin"] for row in rows], ["123.45", "123.46", "124.00"])

    def test_rejects_entire_malformed_v2_batches_and_recovers(self):
        cases = (
            (viewer.MULTI_EVT_SENSOR_READINGS, 0, struct.pack("<HHHH", 1, 2, 123, 4)),
            (viewer.MULTI_EVT_SENSOR_READINGS, viewer.MULTI_FLAG_LOCKIN_CENTI, struct.pack("<HHIH", 1, 2, 12345, 4)),
            (viewer.MULTI_EVT_SCAN_POINTS, 0, struct.pack("<BHH", 1, 2, 123)),
            (viewer.MULTI_EVT_SCAN_POINTS, viewer.MULTI_FLAG_LOCKIN_CENTI, struct.pack("<BHI", 1, 2, 12345)),
        )
        for kind, flags, record in cases:
            with self.subTest(kind=kind, flags=flags):
                node = self.window.ensure_multi_node(0x1234)
                before = (len(node.rows), len(node.scan_points), self.window.total_samples)
                for payload in (b"", b"\x02" + record, b"\x01" + record[:-1],
                                b"\x01" + record + b"\x00", b"\x00" + record):
                    self.feed(kind, payload, flags)
                    self.assertEqual((len(node.rows), len(node.scan_points), self.window.total_samples), before)
                self.feed(kind, b"\x01" + record, flags)
                target = node.rows if kind == viewer.MULTI_EVT_SENSOR_READINGS else node.scan_points
                self.assertEqual(len(target), (before[0] if kind == viewer.MULTI_EVT_SENSOR_READINGS else before[1]) + 1)

    def test_rejects_mismatched_precision_flag(self):
        legacy = struct.pack("<BHHHH", 1, 1, 2, 123, 4)
        precise = struct.pack("<BHHIH", 1, 1, 2, 12345, 4)
        node = self.feed(viewer.MULTI_EVT_SENSOR_READINGS, legacy, viewer.MULTI_FLAG_LOCKIN_CENTI)
        self.feed(viewer.MULTI_EVT_SENSOR_READINGS, precise)
        self.assertEqual(len(node.rows), 0)

    def test_legacy_v1_realtime_and_scan_and_strict_batch_length(self):
        self.window.single_mode.setChecked(True)
        good = struct.pack("<BHHHH", 1, 8, 300, 123, 456)
        for invalid in (b"", b"\x02" + good[1:], good + b"\x00", good[:-1]):
            self.window.on_dc_bytes(viewer.dc_encode(viewer.DC_EVT_SENSOR_READINGS, payload=invalid))
        self.assertEqual(len(self.window.dc_rows), 0)
        self.window.on_dc_bytes(viewer.dc_encode(viewer.DC_EVT_SENSOR_READINGS, payload=good))
        self.assertEqual(self.window.dc_rows[-1][1:], (8, 300, 123, 456))
        scan = struct.pack("<BBHH", 1, 0, 300, 124)
        self.window.on_dc_bytes(viewer.dc_encode(viewer.DC_EVT_SCAN_POINTS, payload=scan[:-1]))
        self.assertFalse(self.window.dc_scan_points)
        self.window.on_dc_bytes(viewer.dc_encode(viewer.DC_EVT_SCAN_POINTS, payload=scan))
        self.assertEqual(self.window.dc_scan_points, [(0, 300, 124)])
        self.assertEqual(self.export_rows(self.window.save_data)[0]["lockin"], "123.00")
        self.assertEqual(self.export_rows(self.window.export_dc_scan)[0]["lockin"], "124.00")

    def test_fractional_stats_cursor_and_adc_semantics(self):
        values = viewer.np.asarray([123.45, 123.46])
        self.window.update_multi_stats(values)
        self.assertEqual(self.window.multi_latest_label.text(), "最新值：123.46")
        self.assertEqual(self.window.multi_pp_label.text(), "峰峰值：0.01")
        self.assertEqual(self.window.multi_range_label.text(), "最小/最大：123.45 / 123.46")
        self.window.update_stats(values, "test")
        self.assertEqual(self.window.pp_label.text(), "峰峰值：0.01")
        self.assertEqual(self.window.range_label.text(), "最小/最大：123.45 / 123.46")
        self.window.last_x = viewer.np.asarray([0., 1.])
        self.window.last_y = values
        self.window.show_coordinate(1., 0.)
        self.assertIn("123.46", self.window.cursor_text.toPlainText())
        self.window.ac_mode.setChecked(True)
        self.window.update_multi_stats(viewer.np.asarray([123, 124]))
        self.assertEqual(self.window.multi_latest_label.text(), "最新值：124")
        self.assertEqual(self.window.multi_pp_label.text(), "峰峰值：1")
        self.assertEqual(self.window.multi_range_label.text(), "最小/最大：123 / 124")
        self.assertEqual(self.window.multi_mean_label.text(), "平均值：123.500")

    def test_multi_csv_preserves_centi_values_in_both_display_modes(self):
        for dc in (True, False):
            (self.window.dc_mode if dc else self.window.ac_mode).setChecked(True)
            self.feed(viewer.MULTI_EVT_SENSOR_READINGS, struct.pack("<BHHIH", 1, 1, 300, 12340, 2048),
                      viewer.MULTI_FLAG_LOCKIN_CENTI)
            row = self.export_rows(self.window.save_multi_data)[0]
            self.assertEqual(row["lockin"], "123.40")
            self.assertEqual(row["dac"], "300")
            self.assertEqual(row["adc"], "2048")
            self.assertEqual(row["display_field"], "lockin" if dc else "adc")
            self.assertEqual(row["display_value"], "123.40" if dc else "2048")

    def test_duration_presets_custom_input_and_600_second_multi_window(self):
        combo = self.window.duration_combo
        items = [combo.itemText(i) for i in range(combo.count())]
        self.assertEqual(items.count("10 min"), 1)
        for item in items:
            combo.setCurrentText(item)
            seconds = self.window.duration_seconds()
            self.assertTrue(seconds is None or seconds >= 1, item)
        self.assertTrue(combo.isEditable())
        combo.setCurrentText("0.25 s")
        self.assertEqual(self.window.duration_seconds(), 0.25)
        combo.setCurrentText("10 min")
        self.assertEqual(self.window.duration_seconds(), 600)
        self.assertEqual(self.window.time_display_settings()[:2], (1 / 60, "min"))
        node = self.window.ensure_multi_node(0x1234)
        node.online = True
        node.rows.extend((timestamp, i, 1, 123.45 + i / 100, 1000 + i)
                         for i, timestamp in enumerate((399.99, 400., 700., 1000.)))
        self.window.multi_start_time = 400.
        with patch.object(viewer.time, "time", return_value=1000.):
            self.window.trim_multi_buffers()
            self.window.multi_plot_dirty = True
            self.window.refresh_multi_plot()
        self.assertEqual([row[0] for row in node.rows], [400., 700., 1000.])
        x, y = self.window.multi_curves[node.node_id].getData()
        viewer.np.testing.assert_allclose(x, [0., 5., 10.])
        self.assertEqual(len(y), 3)
        self.assertEqual(len(self.window.multi_last_stats_values), 3)

    def test_600_second_single_ac_and_dc_buffers(self):
        self.window.single_mode.setChecked(True)
        self.window.duration_combo.setCurrentText("10 min")
        self.window.dc_rows.extend((timestamp, i, 1, 123, 1000)
                                   for i, timestamp in enumerate((399.99, 400., 1000.)))
        with patch.object(viewer.time, "time", return_value=1000.):
            self.window.trim_buffer()
        self.assertEqual([row[0] for row in self.window.dc_rows], [400., 1000.])
        self.window.ac_mode.setChecked(True)
        self.window.sample_rate.setValue(10)
        self.window.sample_chunks.extend(viewer.np.arange(120, dtype=float) for _ in range(51))
        self.window.sample_count = self.window.total_samples = 6120
        self.window.trim_buffer()
        self.assertEqual(self.window.sample_count, 6000)
        self.window.plot_dirty = True
        self.window.refresh_plot()
        self.assertEqual(self.window.last_y.size, 6000)
        self.assertEqual(self.window.last_stats_values.size, 6000)
        self.assertTrue(self.window.curve.opts["autoDownsample"])
        self.assertEqual(self.window.curve.opts["downsampleMethod"], "peak")
        self.assertEqual(len(self.export_rows(self.window.save_data)), 6000)


if __name__ == "__main__":
    unittest.main()
