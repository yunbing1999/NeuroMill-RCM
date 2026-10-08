"""Geometry and lifecycle checks without connecting to a robot or ROS graph."""
import json
import time
from collections import deque
from types import SimpleNamespace
from unittest.mock import Mock

import pytest
from builtin_interfaces.msg import Time
from std_msgs.msg import String
from visualization_msgs.msg import Marker

from neuro_final_teleop.nodes.rcm_visualizer import RCMVisualizer


def viewer():
    node = SimpleNamespace(
        trail=deque(maxlen=3), entry=None, length=0.10, diameter=0.005, timeout=1.0,
        show_shaft_marker=True, samples=0, error_sum=0.0, error_sq_sum=0.0, error_max=0.0,
        first_sample_s=None, latest_sample_s=None,
        visible=False, last_received=None, last_markers=None, ended=False, publisher=Mock(),
        get_clock=lambda: SimpleNamespace(now=lambda: SimpleNamespace(to_msg=lambda: Time())),
        get_logger=lambda: Mock(),
    )
    node.summary_text = lambda status: RCMVisualizer.summary_text(node, status)
    node.clear = lambda: RCMVisualizer.clear(node)
    node.check_stale = lambda: RCMVisualizer.check_stale(node)
    return node


def message():
    return String(data=json.dumps(dict(
        time_s=100.0, frame_id='link_base', tool_position_mm=[100, 200, 300],
        entry_point_mm=[100, 200, 400], shaft_point_mm=[100, 200, 300],
        shaft_axis_base=[1, 0, 0], lateral_error_mm=100,
    )))


def test_geometry_uses_base_meters_and_reported_axis():
    node = viewer()
    RCMVisualizer.on_diagnostics(node, message())
    markers = node.publisher.publish.call_args.args[0].markers
    assert len(markers) == 6
    assert all(m.header.frame_id == 'link_base' for m in markers)
    assert markers[0].pose.position.z == pytest.approx(0.4)
    assert markers[1].pose.position.x == pytest.approx(0.1)
    assert markers[2].type == Marker.CYLINDER
    assert markers[2].scale.z == pytest.approx(0.1)
    assert markers[2].pose.position.x == pytest.approx(0.05)
    assert markers[2].pose.position.z == pytest.approx(0.3)
    assert markers[4].points[0].z - markers[4].points[1].z == pytest.approx(0.1)


def test_pause_retains_snapshot_and_resume_extends_trail():
    node = viewer()
    RCMVisualizer.on_diagnostics(node, message())
    node.last_received = time.monotonic() - 2
    RCMVisualizer.check_stale(node)
    assert node.visible and len(node.trail) == 1
    markers = node.publisher.publish.call_args.args[0].markers
    assert all(m.action == Marker.ADD for m in markers)
    assert 'PAUSED' in markers[5].text
    RCMVisualizer.on_diagnostics(node, message())
    assert len(node.trail) == 2
    assert 'ACTIVE' in node.last_markers.markers[5].text


def test_stop_retains_snapshot_until_next_start():
    node = viewer()
    RCMVisualizer.on_diagnostics(node, message())
    RCMVisualizer.on_event(node, String(data='{"event":"stop"}'))
    assert node.visible and len(node.trail) == 1
    assert 'ENDED' in node.last_markers.markers[5].text
    RCMVisualizer.on_diagnostics(node, message())  # Ignore queued samples after stop.
    assert len(node.trail) == 1
    RCMVisualizer.on_event(node, String(data='{"event":"start"}'))
    assert not node.visible and not node.trail and not node.ended
    assert all(m.action == Marker.DELETE and m.ns == 'rcm'
               for m in node.publisher.publish.call_args.args[0].markers)
    RCMVisualizer.on_diagnostics(node, message())
    assert node.visible and len(node.trail) == 1


def test_trail_is_bounded_and_resets_on_new_capture():
    node = viewer()
    for _ in range(5):
        RCMVisualizer.on_diagnostics(node, message())
    assert len(node.trail) == 3
    data = json.loads(message().data)
    data['entry_point_mm'] = [101, 200, 400]
    RCMVisualizer.on_diagnostics(node, String(data=json.dumps(data)))
    assert len(node.trail) == 1


def test_summary_uses_received_samples_only_and_resets():
    node = viewer()
    for t, error in [(100, 3), (102, 4)]:
        d = json.loads(message().data)
        d.update(time_s=t, lateral_error_mm=error)
        RCMVisualizer.on_diagnostics(node, String(data=json.dumps(d)))
    RCMVisualizer.check_stale(node)
    assert node.samples == 2
    text = node.last_markers.markers[5].text
    assert 'Samples = 2' in text
    assert 'Mean error = 3.500000 mm' in text
    assert 'Max error = 4.000000 mm' in text
    assert 'RMS error = 3.535534 mm' in text
    assert 'Latest' not in text
    assert 'KDL MODEL ONLY' in text
    RCMVisualizer.on_event(node, String(data='{"event":"start"}'))
    assert node.samples == 0 and node.error_max == 0

def test_robot_model_shaft_disables_duplicate_marker():
    node = viewer()
    node.show_shaft_marker = False
    RCMVisualizer.on_diagnostics(node, message())
    assert node.last_markers.markers[2].action == Marker.DELETE
    assert node.last_markers.markers[0].action == Marker.ADD

def test_summary_shows_captured_entry_not_moving_tcp():
    node = viewer()
    RCMVisualizer.on_diagnostics(node, message())
    text = node.summary_text('ACTIVE')
    assert 'Captured RCM [link_base]' in text
    assert 'X = 100.000 mm' in text
    assert 'Y = 200.000 mm' in text
    assert 'Z = 400.000 mm' in text
