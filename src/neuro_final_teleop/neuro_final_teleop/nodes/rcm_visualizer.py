"""Display controller RCM geometry; never connect to or command the robot."""

from collections import deque
import json
import math
import time

import rclpy
from rclpy.node import Node
from geometry_msgs.msg import Point
from std_msgs.msg import String
from visualization_msgs.msg import Marker, MarkerArray


def vector(data, key, scale=1.0):
    values = [float(v) * scale for v in data[key]]
    if len(values) != 3 or not all(math.isfinite(v) for v in values):
        raise ValueError(f"Invalid vector: {key}")
    return values


def point(values):
    return Point(x=float(values[0]), y=float(values[1]), z=float(values[2]))


class RCMVisualizer(Node):
    def __init__(self):
        super().__init__('rcm_visualizer')
        self.declare_parameter('show_shaft_marker', False)
        self.show_shaft_marker = bool(self.get_parameter('show_shaft_marker').value)
        self.declare_parameter('shaft_length_m', 0.10)
        self.declare_parameter('shaft_diameter_m', 0.005)
        self.declare_parameter('max_trail_points', 1500)
        self.declare_parameter('stale_timeout_s', 1.0)
        self.length = float(self.get_parameter('shaft_length_m').value)
        self.diameter = float(self.get_parameter('shaft_diameter_m').value)
        self.timeout = float(self.get_parameter('stale_timeout_s').value)
        count = int(self.get_parameter('max_trail_points').value)
        if not all(math.isfinite(v) and v > 0 for v in (self.length, self.diameter, self.timeout)) or count < 2:
            raise ValueError('Positive length/timeout and at least two trail points required')
        self.trail = deque(maxlen=count)
        self.entry = None
        self.last_received = None
        self.visible = False
        self.last_markers = None
        self.ended = False
        self.samples = 0
        self.error_sum = self.error_sq_sum = self.error_max = 0.0
        self.first_sample_s = self.latest_sample_s = None
        self.publisher = self.create_publisher(MarkerArray, '/neuro_final/rcm_markers', 10)
        self.diag_sub = self.create_subscription(
            String, '/neuro_final/rcm_diagnostics', self.on_diagnostics, 10)
        self.event_sub = self.create_subscription(
            String, '/neuro_final/rcm_log_event', self.on_event, 10)
        self.timer = self.create_timer(0.2, self.check_stale)
        self.get_logger().info('Waiting for RCM diagnostics; markers: /neuro_final/rcm_markers')

    def clear(self):
        msg = MarkerArray()
        for index in range(6):
            marker = Marker()
            marker.ns = 'rcm'
            marker.id = index
            marker.action = Marker.DELETE
            msg.markers.append(marker)
        self.publisher.publish(msg)
        self.trail.clear()
        self.entry = None
        self.visible = False
        self.last_received = None
        self.last_markers = None
        self.ended = False
        self.samples = 0
        self.error_sum = self.error_sq_sum = self.error_max = 0.0
        self.first_sample_s = self.latest_sample_s = None

    def on_event(self, message):
        try:
            event = json.loads(message.data).get('event')
            if event == 'start':
                self.clear()
            elif event == 'stop':
                self.ended = True
                self.check_stale()
        except (ValueError, TypeError, AttributeError):
            self.get_logger().warning('Invalid RCM event')

    def check_stale(self):
        if not self.visible or self.last_markers is None:
            return
        if self.ended:
            status = 'ENDED'
        elif time.monotonic() - self.last_received > self.timeout:
            status = 'PAUSED/NO_DATA'
        else:
            status = 'LIVE'
        label = self.last_markers.markers[5]
        label.text = self.summary_text(status)
        if status == 'LIVE':
            label.color.r, label.color.g, label.color.b = 0.05, 0.15, 0.2
        else:
            label.color.r, label.color.g, label.color.b = 0.5, 0.22, 0.0
        stamp = self.get_clock().now().to_msg()
        for marker in self.last_markers.markers:
            marker.header.stamp = stamp
        # Refresh the frozen snapshot, without extending or clearing its trajectory.
        self.publisher.publish(self.last_markers)

    def summary_text(self, status):
        duration = max(0.0, self.latest_sample_s - self.first_sample_s)
        # Avoid spaces: some RViz/Ogre font configurations render spaces too wide.
        return (f'RCM:{status}\n'
                'Captured_RCM[link_base]\n'
                f'X={self.entry[0]*1000:.3f}mm\n'
                f'Y={self.entry[1]*1000:.3f}mm\n'
                f'Z={self.entry[2]*1000:.3f}mm\n'
                f'N={self.samples};T={duration:.1f}s\n'
                f'Latest={self.last_error:.6f}mm\n'
                f'Mean={self.error_sum / self.samples:.6f}mm\n'
                f'RMS={math.sqrt(self.error_sq_sum / self.samples):.6f}mm\n'
                f'Max={self.error_max:.6f}mm\n'
                'KDL_MODEL_ONLY')

    def on_diagnostics(self, message):
        if self.ended:
            return
        try:
            data = json.loads(message.data)
            if data['frame_id'] != 'link_base':
                raise ValueError('Expected controller geometry in link_base')
            tcp = vector(data, 'tool_position_mm', 0.001)
            entry = vector(data, 'entry_point_mm', 0.001)
            closest = vector(data, 'shaft_point_mm', 0.001)
            axis = vector(data, 'shaft_axis_base')
            norm = math.sqrt(sum(v*v for v in axis))
            if norm < 1e-9:
                raise ValueError('Zero shaft direction')
            axis = [v / norm for v in axis]
            error = float(data['lateral_error_mm'])
            sample_s = float(data['time_s'])
            if not math.isfinite(error) or error < 0 or not math.isfinite(sample_s):
                raise ValueError('Nonfinite RCM error')
        except (KeyError, ValueError, TypeError, AttributeError) as exc:
            self.get_logger().warning(f'Invalid geometry; rebuild/restart teleop if fields missing: {exc}')
            return

        # Entry is carried in every diagnostic, so starting this node mid-session works.
        if self.entry != entry:
            self.clear()
            self.entry = entry
        if self.first_sample_s is None:
            self.first_sample_s = sample_s
        self.latest_sample_s = sample_s
        self.samples += 1
        self.error_sum += error
        self.error_sq_sum += error * error
        self.error_max = max(self.error_max, error)
        self.last_error = error
        self.trail.append(point(tcp))
        stamp = self.get_clock().now().to_msg()
        msg = MarkerArray()

        def marker(index, kind, color, width):
            m = Marker()
            m.header.frame_id = 'link_base'
            m.header.stamp = stamp
            m.ns = 'rcm'
            m.id = index
            m.type = kind
            m.action = Marker.ADD
            m.pose.orientation.w = 1.0
            m.color.r, m.color.g, m.color.b, m.color.a = color
            m.scale.x = width
            # Expire even if this process is terminated without cleanup.
            m.lifetime.sec = int(self.timeout)
            m.lifetime.nanosec = int((self.timeout % 1) * 1e9)
            msg.markers.append(m)
            return m

        for index, xyz, color in [
            (0, entry, (1.0, 0.15, 0.15, 1.0)),
            (1, tcp, (0.1, 1.0, 0.25, 1.0)),
        ]:
            m = marker(index, Marker.SPHERE, color, 0.008)
            m.scale.y = m.scale.z = 0.008
            m.pose.position = point(xyz)

        # Approximate exposed drill shaft, not the complete EMAX2PLUS CAD.
        # Current shaft_axis_sign=+1 points outward; shaft extends behind TCP.
        m = marker(2, Marker.CYLINDER, (0.45, 0.48, 0.52, 1.0), self.diameter)
        if not self.show_shaft_marker:
            m.action = Marker.DELETE  # Permanent shaft is in RobotModel.
        m.scale.y = self.diameter
        m.scale.z = self.length
        m.pose.position = point([tcp[i] - self.length * 0.5 * axis[i] for i in range(3)])
        if axis[2] < -1.0 + 1e-9:
            m.pose.orientation.x = 1.0
            m.pose.orientation.w = 0.0
        else:
            qnorm = math.sqrt(2.0 * (1.0 + axis[2]))
            m.pose.orientation.x = -axis[1] / qnorm
            m.pose.orientation.y = axis[0] / qnorm
            m.pose.orientation.z = 0.0
            m.pose.orientation.w = (1.0 + axis[2]) / qnorm
        m = marker(3, Marker.LINE_STRIP, (0.0, 0.45, 0.15, 0.9), 0.001)
        m.points = list(self.trail) if len(self.trail) > 1 else [point(tcp), point(tcp)]
        m = marker(4, Marker.LINE_LIST, (0.75, 0.35, 0.0, 1.0), 0.001)
        m.points = [point(entry), point(closest)]
        m = marker(5, Marker.TEXT_VIEW_FACING, (0.05, 0.15, 0.2, 1.0), 0.014)
        m.scale.z = 0.014
        m.pose.position = point([entry[0] + 0.12, entry[1], entry[2] + 0.08])
        m.text = self.summary_text('LIVE')
        self.last_error = error
        self.last_markers = msg
        self.publisher.publish(msg)
        self.last_received = time.monotonic()
        self.visible = True


def main(args=None):
    rclpy.init(args=args)
    node = RCMVisualizer()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        if rclpy.ok():
            node.clear()
        node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()


if __name__ == '__main__':
    main()
