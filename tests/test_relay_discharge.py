"""Offline Qt regression tests for node-directed relay discharge and 30 min.

Run: .build-venv-multi/Scripts/python.exe tests/test_relay_discharge.py
No hardware or real serial connection is opened.
"""
import os
from pathlib import Path
import struct
import sys
import unittest
from unittest.mock import patch

os.environ.setdefault("QT_QPA_PLATFORM", "offscreen")
sys.path.insert(0, str(Path(__file__).resolve().parents[1]))
import sensor_waveform_viewer as viewer


class RelayDischargeTests(unittest.TestCase):
    @classmethod
    def setUpClass(cls):
        cls.app = viewer.QtWidgets.QApplication.instance() or viewer.QtWidgets.QApplication([])

    def setUp(self):
        self.window = viewer.MainWindow()
        self.window.multi_mode.setChecked(True)
        self.window.plot_timer.stop()
        self.window.relay_timer.stop()
        self.window.request_write.disconnect(self.window.worker.write)
        self.sent = []
        self.window.request_write.connect(self.sent.append)

    def tearDown(self):
        self.window.close()
        self.app.processEvents()

    def feed(self, kind, node_id=0x1234, payload=b"", scan_id=0, seq=1):
        packet = viewer.multi_encode(kind, node_id, scan_id=scan_id, seq=seq, payload=payload)
        # Exercise fragmented UART delivery, including the new one-byte event.
        for offset in range(0, len(packet), 5):
            self.window.on_multi_bytes(packet[offset:offset + 5])

    def link(self, node_id=0x1234, state=1):
        self.feed(viewer.MULTI_EVT_LINK_STATUS, node_id,
                  struct.pack("<BB6sBBBH", state, 1, b"\0" * 6, 0, 6, 0, 251))
        return self.window.multi_nodes[node_id]

    def connect(self):
        self.window.on_serial_state(True, "offline test")

    def select(self, node_id):
        for row in range(self.window.multi_node_table.rowCount()):
            item = self.window.multi_node_table.item(row, 0)
            if item.data(viewer.QtCore.Qt.UserRole) == node_id:
                self.window.multi_node_table.selectRow(row)
                return
        self.fail(f"Missing node {node_id:04X}")

    def command(self):
        before = len(self.sent)
        self.window.multi_relay_button.click()
        self.assertEqual(len(self.sent), before + 1)
        return viewer.MultiFrameParser().feed(self.sent[-1])[0]

    def reply(self, request, status=0, nack=False, **overrides):
        args = dict(node_id=request.node_id, scan_id=request.scan_id, seq=request.seq,
                    payload=bytes([viewer.MULTI_CMD_RELAY_DISCHARGE, status]))
        args.update(overrides)
        self.feed(viewer.MULTI_EVT_NACK if nack else viewer.MULTI_EVT_ACK, **args)

    def state(self, level, node_id=0x1234, scan_id=0, seq=100):
        self.feed(viewer.MULTI_EVT_RELAY_STATE, node_id, bytes([level]), scan_id, seq)

    def test_button_requires_connected_online_selected_v2_node_in_both_sensor_modes(self):
        for dc in (False, True):
            with self.subTest(dc=dc):
                self.window.on_serial_state(False, "reset")
                (self.window.dc_mode if dc else self.window.ac_mode).setChecked(True)
                self.window.clear_multi_data(True)
                self.assertFalse(self.window.multi_relay_button.isEnabled())
                self.link()
                self.assertFalse(self.window.multi_relay_button.isEnabled())
                self.connect()
                self.assertTrue(self.window.multi_relay_button.isEnabled())
                request = self.command()
                self.assertEqual(request.kind, viewer.MULTI_CMD_RELAY_DISCHARGE)
                self.assertEqual(request.node_id, 0x1234)
                self.assertEqual(request.payload, b"")
                self.assertEqual(request.flags, 0)
                self.assertEqual(len(self.sent[-1]), 14)
                self.assertFalse(self.window.multi_relay_button.isEnabled())
                before = len(self.sent)
                self.window.multi_relay_button.click()
                self.window.start_multi_relay_discharge()
                self.assertEqual(len(self.sent), before)
        self.window.on_serial_state(False, "reset")
        self.window.single_mode.setChecked(True)
        before = len(self.sent)
        self.assertFalse(self.window.start_multi_relay_discharge())
        self.assertEqual(len(self.sent), before)

    def test_ack_never_claims_pin_high_or_completion(self):
        node = self.link()
        self.connect()
        request = self.command()
        self.assertEqual(node.relay_phase, "waiting_ack")
        self.reply(request)
        self.assertEqual(node.relay_phase, "waiting_state")
        self.assertIsNone(node.relay_level)
        self.assertIn("已确认", self.window.multi_relay_status.text())
        self.assertFalse(self.window.multi_relay_button.isEnabled())
        self.state(1, scan_id=request.scan_id)
        self.assertEqual(node.relay_level, 1)
        self.assertIn("正在放电", self.window.multi_relay_status.text())
        self.assertFalse(self.window.multi_relay_button.isEnabled())
        self.state(0, scan_id=request.scan_id, seq=101)
        self.assertEqual(node.relay_level, 0)
        self.assertIn("PD7 低电平", self.window.multi_relay_status.text())
        self.assertTrue(self.window.multi_relay_button.isEnabled())

    def test_real_state_before_ack_and_late_ack_do_not_regress_status(self):
        node = self.link()
        self.connect()
        request = self.command()
        self.state(1, scan_id=request.scan_id)
        self.reply(request)
        self.assertEqual(node.relay_phase, "active")
        self.state(0, scan_id=request.scan_id, seq=101)
        self.reply(request)
        self.assertEqual(node.relay_phase, "low")
        self.assertTrue(self.window.multi_relay_button.isEnabled())

    def test_older_low_event_cannot_complete_a_new_pending_command(self):
        node = self.link()
        self.connect()
        request = self.command()
        self.state(0, scan_id=0)
        self.assertEqual(node.relay_phase, "waiting_ack")
        self.assertEqual(node.relay_pending_seq, request.seq)
        self.assertFalse(self.window.multi_relay_button.isEnabled())
        self.reply(request)
        self.assertEqual(node.relay_phase, "waiting_state")
        self.state(1, scan_id=request.scan_id)
        self.assertEqual(node.relay_phase, "active")

    def test_old_high_then_low_during_new_request_never_falsely_displays_high(self):
        node = self.link()
        self.connect()
        request = self.command()
        self.state(1, scan_id=0, seq=100)
        self.state(0, scan_id=0, seq=101)
        self.assertEqual(node.relay_level, 0)
        self.assertEqual(node.relay_pending_seq, request.seq)
        self.assertFalse(self.window.multi_relay_button.isEnabled())
        self.assertIn("PD7 低电平", self.window.multi_relay_status.text())
        self.assertNotIn("PD7 高电平", self.window.multi_relay_status.text())
        self.reply(request)
        self.state(1, scan_id=request.scan_id, seq=102)
        self.assertIn("PD7 高电平", self.window.multi_relay_status.text())

    def test_selected_node_changes_do_not_retarget_replies_or_commands(self):
        first = self.link(0x1234)
        second = self.link(0x2345)
        self.connect()
        self.select(first.node_id)
        request_first = self.command()
        self.select(second.node_id)
        request_second = self.command()
        self.assertNotEqual(request_first.scan_id, request_second.scan_id)
        self.assertNotEqual(request_first.seq, request_second.seq)
        self.reply(request_first)
        self.state(1, first.node_id, request_first.scan_id)
        self.assertEqual(first.relay_phase, "active")
        self.assertEqual(second.relay_phase, "waiting_ack")
        self.assertIn("0x2345", self.window.multi_relay_status.text())
        self.reply(request_second)
        self.state(1, second.node_id, request_second.scan_id)
        self.state(0, first.node_id, request_first.scan_id, seq=101)
        self.assertEqual(first.relay_phase, "low")
        self.assertEqual(second.relay_phase, "active")
        self.assertFalse(self.window.multi_relay_button.isEnabled())
        self.select(first.node_id)
        self.assertTrue(self.window.multi_relay_button.isEnabled())
        self.assertIn("PD7 低电平", self.window.multi_relay_status.text())

    def test_wrong_ack_sequence_scan_or_node_and_malformed_packets_are_ignored(self):
        node = self.link()
        self.link(0x2345)
        self.connect()
        self.select(node.node_id)
        request = self.command()
        for overrides in ({"seq": request.seq + 1}, {"scan_id": request.scan_id + 1},
                          {"node_id": 0x2345},
                          {"payload": bytes([viewer.MULTI_CMD_RELAY_DISCHARGE])},
                          {"payload": bytes([viewer.MULTI_CMD_RELAY_DISCHARGE, 0, 0])}):
            self.reply(request, **overrides)
            self.assertEqual(node.relay_phase, "waiting_ack")
        for payload in (b"", b"\x02", b"\x00\x00", b"\x01\x00"):
            self.feed(viewer.MULTI_EVT_RELAY_STATE, payload=payload, scan_id=request.scan_id)
            self.assertEqual(node.relay_phase, "waiting_ack")
            self.assertIsNone(node.relay_level)
        self.reply(request)
        self.state(1, scan_id=request.scan_id)
        self.assertEqual(node.relay_phase, "active")

    def test_nack_allows_retry_and_busy_waits_for_real_state_without_retrigger(self):
        node = self.link()
        self.connect()
        first = self.command()
        self.reply(first, status=0x07, nack=True)
        self.assertEqual(node.relay_phase, "rejected")
        self.assertTrue(self.window.multi_relay_button.isEnabled())
        second = self.command()
        self.reply(second, status=0x06, nack=True)
        self.assertEqual(node.relay_phase, "busy")
        self.assertIsNone(node.relay_level)
        self.assertFalse(self.window.multi_relay_button.isEnabled())
        self.state(1, scan_id=first.scan_id)
        self.assertEqual(node.relay_phase, "active")
        self.assertFalse(self.window.start_multi_relay_discharge())
        self.assertEqual(len(self.sent), 2)
        self.state(0, scan_id=first.scan_id, seq=101)
        self.assertTrue(self.window.multi_relay_button.isEnabled())

    def test_timeout_is_unknown_and_manual_retry_ignores_old_ack(self):
        node = self.link()
        self.connect()
        with patch.object(viewer.time, "monotonic", return_value=100.0):
            old = self.command()
        with patch.object(viewer.time, "monotonic", return_value=105.0):
            self.window.check_multi_relay_timeouts()
            self.assertEqual(node.relay_phase, "timeout")
            self.assertIsNone(node.relay_level)
            self.assertTrue(self.window.multi_relay_button.isEnabled())
            self.assertEqual(len(self.sent), 1)
            new = self.command()
        self.reply(old)
        self.assertEqual(node.relay_pending_seq, new.seq)
        self.assertEqual(node.relay_phase, "waiting_ack")
        with patch.object(viewer.time, "monotonic", return_value=106.0):
            self.reply(new)
            self.state(1, scan_id=new.scan_id)
        with patch.object(viewer.time, "monotonic", return_value=115.0):
            self.window.check_multi_relay_timeouts()
        self.assertIsNone(node.relay_level)
        self.assertEqual(node.relay_phase, "timeout")
        self.assertIn("状态未知", self.window.multi_relay_status.text())
        self.assertNotIn("低电平", self.window.multi_relay_status.text())
        self.assertEqual(len(self.sent), 2)

    def test_disconnect_reconnect_and_discovery_reset_state_without_stale_resurrection(self):
        node = self.link()
        self.connect()
        self.state(1)
        self.assertEqual(node.relay_phase, "active")
        self.link(state=0)
        self.assertIsNone(node.relay_level)
        self.assertFalse(self.window.multi_relay_button.isEnabled())
        self.state(1, seq=101)  # A delayed state cannot make a disconnected node ready.
        self.assertIsNone(node.relay_level)
        self.link(state=2)
        self.state(1, seq=102)  # Startup report before GATT readiness remains unknown.
        self.assertIsNone(node.relay_level)
        self.link(state=1)
        self.assertTrue(self.window.multi_relay_button.isEnabled())
        self.state(1, seq=103)
        self.link(state=1)  # Repeated unchanged readiness does not erase a current GPIO report.
        self.assertEqual(node.relay_level, 1)
        self.window.on_serial_state(False, "closed")
        self.assertIsNone(node.relay_level)
        self.assertFalse(self.window.multi_relay_button.isEnabled())
        self.connect()
        self.assertFalse(self.window.multi_relay_button.isEnabled())
        self.link(state=1)
        self.assertIsNone(node.relay_level)
        self.assertTrue(self.window.multi_relay_button.isEnabled())

    def test_unknown_and_archived_state_events_cannot_allocate_a_control_target(self):
        self.state(1, node_id=0x9999)
        self.assertNotIn(0x9999, self.window.multi_nodes)
        for i in range(8):
            self.link(0x5000 + i)
        self.link(0x5000, state=0)
        self.link(0x6000, state=1)
        self.assertIn(0x5000, self.window.multi_archived_nodes)
        self.state(1, node_id=0x5000)
        self.assertNotIn(0x5000, self.window.multi_nodes)
        self.assertEqual(len(self.window.multi_nodes), 8)

    def test_clear_waveforms_keeps_pending_control_and_unique_command_ids(self):
        node = self.link()
        self.link(0x2345)
        self.connect()
        self.select(node.node_id)
        request = self.command()
        self.window.clear_multi_data()
        self.assertEqual(node.relay_pending_seq, request.seq)
        self.assertEqual(node.relay_phase, "waiting_ack")
        self.select(0x2345)
        other = self.command()
        self.assertNotEqual(other.seq, request.seq)
        self.assertNotEqual(other.scan_id, request.scan_id)
        self.reply(request)
        self.state(0, node.node_id, request.scan_id)
        self.assertEqual(node.relay_phase, "low")

    def test_serial_reconnect_does_not_reuse_accepted_command_key(self):
        self.link()
        self.connect()
        first = self.command()
        sensor_accepted = {(first.scan_id, first.seq)}
        self.window.on_serial_state(False, "port closed only; BLE remains connected")
        self.window.clear_multi_data(True)
        self.connect()
        self.link()
        second = self.command()
        self.assertNotIn((second.scan_id, second.seq), sensor_accepted)

    def test_new_viewer_session_does_not_restart_the_sensor_deduplication_key(self):
        # Keep the simulated sensor running while an entirely new viewer is created.
        self.window.multi_seq = 1
        self.window.multi_scan_id = 1
        self.link()
        self.connect()
        first = self.command()
        sensor_accepted = {(first.scan_id, first.seq)}
        with patch.object(viewer.secrets, "randbelow", side_effect=(400, 800)):
            replacement = viewer.MainWindow()
        sent = []
        try:
            replacement.multi_mode.setChecked(True)
            replacement.plot_timer.stop()
            replacement.relay_timer.stop()
            replacement.request_write.disconnect(replacement.worker.write)
            replacement.request_write.connect(sent.append)
            replacement.on_serial_state(True, "same receiver and sensor")
            replacement.on_multi_bytes(viewer.multi_encode(
                viewer.MULTI_EVT_LINK_STATUS, first.node_id,
                payload=struct.pack("<BB6sBBBH", 1, 1, b"\0" * 6, 0, 6, 0, 251)))
            replacement.multi_relay_button.click()
            self.assertEqual(len(sent), 1)
            second = viewer.MultiFrameParser().feed(sent[0])[0]
            self.assertNotIn((second.scan_id, second.seq), sensor_accepted)
            self.assertNotEqual(second.scan_id, 0)
            self.assertNotEqual(second.seq, 0)
        finally:
            replacement.close()
            self.app.processEvents()

    def test_30_minute_preset_order_and_1800_second_multi_buffer(self):
        combo = self.window.duration_combo
        options = [combo.itemText(i) for i in range(combo.count())]
        self.assertEqual(options.count("30 min"), 1)
        self.assertEqual(options[options.index("10 min") + 1], "30 min")
        self.assertEqual(options[options.index("30 min") + 1], "1 h")
        self.assertFalse(any(option.endswith("ms") for option in options))
        combo.setCurrentText("30 min")
        self.assertEqual(self.window.duration_seconds(), 1800.0)
        self.assertEqual(self.window.time_display_settings()[:2], (1 / 60, "min"))
        node = self.link()
        node.rows.extend((timestamp, i, 123, 123.45 + i / 100, 456)
                         for i, timestamp in enumerate((199.99, 200., 1100., 2000.)))
        self.window.multi_start_time = 200.
        with patch.object(viewer.time, "time", return_value=2000.0):
            self.window.trim_multi_buffers()
            self.window.multi_plot_dirty = True
            self.window.refresh_multi_plot()
        self.assertEqual([row[0] for row in node.rows], [200., 1100., 2000.])
        x, _ = self.window.multi_curves[node.node_id].getData()
        viewer.np.testing.assert_allclose(x, [0., 15., 30.])
        self.assertEqual(self.window.multi_last_stats_values.size, 3)

    def test_30_minute_single_ac_and_dc_windows_preserve_full_statistics(self):
        self.window.single_mode.setChecked(True)
        self.window.dc_mode.setChecked(True)
        self.window.duration_combo.setCurrentText("30 min")
        self.window.dc_rows.extend((timestamp, i, 123, 123, 456)
                                   for i, timestamp in enumerate((199.99, 200., 2000.)))
        with patch.object(viewer.time, "time", return_value=2000.):
            self.window.trim_buffer()
        self.assertEqual([row[0] for row in self.window.dc_rows], [200., 2000.])
        self.window.ac_mode.setChecked(True)
        self.window.sample_rate.setValue(10)
        self.window.sample_chunks.extend(viewer.np.arange(120, dtype=float) for _ in range(151))
        self.window.sample_count = self.window.total_samples = 18120
        self.window.trim_buffer()
        self.assertEqual(self.window.sample_count, 18000)
        self.window.plot_dirty = True
        self.window.refresh_plot()
        self.assertEqual(self.window.last_y.size, 18000)
        self.assertEqual(self.window.last_stats_values.size, 18000)


if __name__ == "__main__":
    unittest.main()
