"""Run eight-node, lock-in precision, relay and duration widget tests against the code embedded in a release EXE.

Uses local Qt dependencies; checks packaged application code, not bootloader/DLL startup.
Usage: python tests/test_packaged_eight_nodes.py path/to/release.exe
"""
import marshal
from pathlib import Path
import sys
import types
import unittest

from PyInstaller.archive.readers import CArchiveReader
import test_eight_nodes
import test_lockin_precision
import test_relay_discharge

release = Path(sys.argv[1]).resolve()
archive = CArchiveReader(str(release))
module = types.ModuleType("packaged_sensor_waveform_viewer")
module.__file__ = str(release)
sys.modules[module.__name__] = module
exec(marshal.loads(archive.extract("sensor_waveform_viewer")), module.__dict__)
assert module.MULTI_MAX_NODES == 8
test_eight_nodes.viewer = module
test_lockin_precision.viewer = module
test_relay_discharge.viewer = module
suite = unittest.TestSuite([
    unittest.defaultTestLoader.loadTestsFromTestCase(test_eight_nodes.EightNodeTests),
    unittest.defaultTestLoader.loadTestsFromTestCase(test_lockin_precision.LockinPrecisionTests),
    unittest.defaultTestLoader.loadTestsFromTestCase(test_relay_discharge.RelayDischargeTests),
])
result = unittest.TextTestRunner(verbosity=2).run(suite)
sys.exit(0 if result.wasSuccessful() else 1)
