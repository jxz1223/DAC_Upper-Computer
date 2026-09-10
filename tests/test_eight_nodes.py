"""Exercise real Qt widgets and V2 parsing without connecting to a serial port.

Run: .build-venv-multi/Scripts/python.exe tests/test_eight_nodes.py
"""
import os
os.environ.setdefault("QT_QPA_PLATFORM", "offscreen")

from pathlib import Path
import struct
import sys
import unittest

sys.path.insert(0, str(Path(__file__).resolve().parents[1]))
import sensor_waveform_viewer as viewer


class EightNodeTests(unittest.TestCase):
    @classmethod
    def setUpClass(cls):
        cls.app = viewer.QtWidgets.QApplication.instance() or viewer.QtWidgets.QApplication([])

    def setUp(self):
        self.window = viewer.MainWindow()
        self.window.multi_mode.setChecked(True)
        self.window.plot_timer.stop()

    def tearDown(self):
        self.window.close()
        self.app.processEvents()

    def feed_nodes(self):
        packets = bytearray()
        for i in range(8):
            node_id = 0x5000 + i
            link = struct.pack("<BB6sBBBH", 1, 8, node_id.to_bytes(2, "little") + b"\0" * 4, 0, 6, 0, 251)
            packets += viewer.multi_encode(viewer.MULTI_EVT_LINK_STATUS, node_id, payload=link)
            readings = struct.pack("<BHHHH", 1, i, 100 + i, 1000 + i, 2000 + i)
            packets += viewer.multi_encode(viewer.MULTI_EVT_SENSOR_READINGS, node_id, payload=readings)
        # Deliberately split across headers, payloads and CRCs.
        for offset in range(0, len(packets), 7):
            self.window.on_multi_bytes(packets[offset:offset + 7])
        self.window.multi_show_all.setChecked(True)
        self.window.refresh_multi_plot()

    def test_ac_dc_eight_channels_and_reconnect(self):
        for dc in (False, True):
            with self.subTest(dc=dc):
                (self.window.dc_mode if dc else self.window.ac_mode).setChecked(True)
                self.feed_nodes()
                self.assertEqual(len(self.window.multi_nodes), 8)
                self.assertEqual(self.window.multi_node_table.rowCount(), 8)
                self.assertEqual(self.window.multi_online_badge.text(), "8 / 8 在线")
                self.assertEqual(len(set(viewer.MULTI_NODE_COLORS)), 8)
                for i, node in enumerate(self.window.multi_nodes.values()):
                    self.assertEqual(node.color_index, i)
                    _, values = self.window.multi_curves[node.node_id].getData()
                    self.assertEqual(values[-1], (1000 if dc else 2000) + i)
                self.assertIsNone(self.window.ensure_multi_node(0x9999))
                offline = struct.pack("<BB6sBBBH", 0, 7, b"\0" * 6, 8, 6, 0, 251)
                self.window.on_multi_bytes(viewer.multi_encode(viewer.MULTI_EVT_LINK_STATUS, 0x5007, payload=offline))
                self.assertEqual(self.window.multi_online_badge.text(), "7 / 8 在线")
                self.feed_nodes()
                self.assertEqual(self.window.multi_nodes[0x5007].color_index, 7)
                self.assertEqual(self.window.multi_online_badge.text(), "8 / 8 在线")
                self.window.multi_node_table.selectRow(7)
                self.assertEqual(self.window.multi_selected_node_id, 0x5007)
                self.assertIn("CH8", self.window.multi_target_label.text())
                self.assertEqual(self.window.multi_parser.discarded_bytes, 0)

    def test_ch8_dac_command_targets_only_selected_node(self):
        self.window.dc_mode.setChecked(True)
        self.feed_nodes()
        self.window.multi_node_table.selectRow(7)
        # Disconnect the actual worker write slot before pretending the port is open.
        self.window.request_write.disconnect(self.window.worker.write)
        sent = []
        self.window.request_write.connect(sent.append)
        self.window.connect_button.blockSignals(True)
        self.window.connect_button.setChecked(True)
        self.window.connect_button.blockSignals(False)
        self.window.start_multi_dc_scan()
        self.assertEqual(len(sent), 1)
        frames = viewer.MultiFrameParser().feed(sent[0])
        self.assertEqual(frames[0].node_id, 0x5007)
        self.assertEqual(frames[0].kind, viewer.MULTI_CMD_SCAN_START)
        self.assertEqual(len(sent[0]), 23)
        self.assertEqual(self.window.multi_nodes[0x5007].scan_state, "waiting_ack")
        self.assertTrue(all(n.scan_state != "waiting_ack" for k, n in self.window.multi_nodes.items() if k != 0x5007))

    def test_release_button_in_both_modes_waits_for_real_disconnect(self):
        self.window.request_write.disconnect(self.window.worker.write)
        sent = []
        self.window.request_write.connect(sent.append)
        for dc in (False, True):
            with self.subTest(dc=dc):
                self.window.on_serial_state(False, "test")
                (self.window.dc_mode if dc else self.window.ac_mode).setChecked(True)
                self.feed_nodes()
                self.window.multi_node_table.selectRow(7)
                self.window.on_serial_state(True, "test")
                self.assertTrue(self.window.multi_release_button.isEnabled())
                self.window.multi_release_button.click()
                frame = viewer.MultiFrameParser().feed(sent[-1])[0]
                self.assertEqual((frame.kind, frame.node_id, frame.payload),
                                 (viewer.MULTI_CMD_RELEASE_LINK, 0x5007, b""))
                self.assertEqual(len(sent[-1]), 14)
                self.window.on_multi_bytes(viewer.multi_encode(
                    viewer.MULTI_EVT_ACK, 0x5007, seq=frame.seq,
                    payload=bytes([viewer.MULTI_CMD_RELEASE_LINK, 0])))
                self.assertTrue(self.window.multi_nodes[0x5007].online)
                self.assertEqual(self.window.multi_online_badge.text(), "8 / 8 在线")
                self.window.on_multi_bytes(viewer.multi_encode(
                    viewer.MULTI_EVT_LINK_STATUS, 0x5007,
                    payload=struct.pack("<BB6sBBBH", 0, 7, b"\0" * 6, 0x16, 6, 0, 251)))
                self.assertFalse(self.window.multi_release_button.isEnabled())
                self.assertNotIn(0x5007, self.window.multi_release_pending)
                self.assertEqual(self.window.multi_online_badge.text(), "7 / 8 在线")
                self.assertFalse(any("允许重新连接" in b.text() for b in
                                     self.window.findChildren(viewer.QtWidgets.QPushButton)))

    def test_offline_slot_replacement_and_natural_return_keep_history(self):
        self.feed_nodes()
        old = self.window.multi_nodes[0x5007]
        old.scan_points.append((1, 23, 45))
        def link(node_id, state):
            self.window.on_multi_bytes(viewer.multi_encode(
                viewer.MULTI_EVT_LINK_STATUS, node_id,
                payload=struct.pack("<BB6sBBBH", state, 8, b"\0" * 6, 0, 6, 0, 251)))
        link(0x5007, 0)
        link(0x9999, 2)  # New peer reserves the offline slot while discovering.
        self.assertEqual(len(self.window.multi_nodes), 8)
        self.assertEqual(self.window.multi_nodes[0x9999].color_index, 7)
        self.assertIs(self.window.multi_archived_nodes[0x5007], old)
        self.assertTrue(old.rows)
        self.assertIsNone(self.window.ensure_multi_node(0x8888))
        link(0x9999, 1)
        # A late disconnect or ACK cannot resurrect an archived device.
        link(0x5007, 0)
        self.window.on_multi_bytes(viewer.multi_encode(
            viewer.MULTI_EVT_ACK, 0x5007, payload=bytes([viewer.MULTI_CMD_SET_DAC, 0])))
        self.assertNotIn(0x5007, self.window.multi_nodes)
        link(0x5002, 0)
        link(0x5007, 2)  # Automatic return after another slot becomes free.
        self.assertIs(self.window.multi_nodes[0x5007], old)
        self.assertEqual(old.color_index, 2)
        self.assertEqual(old.scan_points, [(1, 23, 45)])
        self.assertTrue(old.rows)
        link(0x5007, 1)
        self.assertEqual(self.window.multi_online_badge.text(), "8 / 8 在线")
        self.assertEqual(len(self.window.multi_curves), 8)
        self.assertEqual(len(self.window.multi_plot.plotItem.legend.items), 8)

    def test_release_nack_and_discovery_target(self):
        self.feed_nodes()
        self.window.request_write.disconnect(self.window.worker.write)
        sent = []
        self.window.request_write.connect(sent.append)
        self.window.multi_node_table.selectRow(0)
        self.window.on_serial_state(True, "test")
        node = self.window.multi_nodes[0x5000]
        node.online = False
        node.link_state = 2
        self.window.update_multi_target_label()
        self.assertTrue(self.window.multi_release_button.isEnabled())
        self.window.multi_release_button.click()
        frame = viewer.MultiFrameParser().feed(sent[-1])[0]
        self.window.on_multi_bytes(viewer.multi_encode(
            viewer.MULTI_EVT_NACK, node.node_id, seq=frame.seq,
            payload=bytes([viewer.MULTI_CMD_RELEASE_LINK, 3])))
        self.assertNotIn(node.node_id, self.window.multi_release_pending)
        self.assertEqual(node.link_state, 2)

    def test_single_modes_still_available(self):
        self.window.single_mode.setChecked(True)
        self.assertFalse(self.window.is_multi_mode())
        self.window.dc_mode.setChecked(True)
        self.assertFalse(self.window.is_multi_mode())
        self.window.ac_mode.setChecked(True)
        self.assertFalse(self.window.is_multi_mode())


if __name__ == "__main__":
    unittest.main()
