"""Capture lifecycle regression checks; no robot connection."""
import csv
import json
from neuro_final_teleop.nodes.rcm_diagnostics_logger import BasicRCMSessionWriter, BASIC_FIELDS
from neuro_final_teleop.control.kinematics import KDLKinModel


def test_capture_files_and_metadata(tmp_path):
    writer = BasicRCMSessionWriter(tmp_path, monotonic_clock=lambda: 10.)
    sample = dict(tcp_position_mm=[1, 2, 3], joints_deg=list(range(7)), lateral_error_mm=.125)
    assert not writer.write_sample(sample)
    path, metadata = writer.start(dict(captured_rcm_point_base_mm=[4,5,6],
                                      tcp_offset_mm_deg=[-36.87,-7.58,243.61,3.52,2.74,6.27]))
    assert writer.write_sample(sample)
    writer.stop()
    assert not writer.write_sample(sample)
    rows = list(csv.reader(path.open()))
    assert rows[0] == list(BASIC_FIELDS)
    assert len(rows) == 2 and len(rows[1]) == 12
    assert json.loads(metadata.read_text())['captured_rcm_point_base_mm'] == [4,5,6]
    second, _ = writer.start({})
    assert second != path
    writer.stop()


def test_controller_tcp_translation_only():
    tcp = [-36.87,-7.58,243.61,3.52,2.74,6.27]
    assert KDLKinModel.effective_tcp_translation_mm(tcp) == tcp[:3]
    assert KDLKinModel.effective_tcp_translation_mm(None) == [0.,0.,0.]
