#!/usr/bin/env python3
import rclpy
from rclpy.node import Node
from geometry_msgs.msg import WrenchStamped
from std_srvs.srv import Trigger
from xarm.wrapper import XArmAPI
import time
import sys

# --- DEFAULT CONFIGURATION ---
ROBOT_IP = '192.168.1.243'
BAUDRATE = 2000000
PUBLISH_HZ = 100.0

class FtSensorBridge(Node):
    def __init__(self):
        super().__init__('ft_sensor_bridge')
        self.declare_parameter('robot_ip', ROBOT_IP)
        self.declare_parameter('ft_baudrate', BAUDRATE)
        self.declare_parameter('publish_hz', PUBLISH_HZ)
        self.declare_parameter('auto_zero_on_startup', True)

        self.robot_ip = str(self.get_parameter('robot_ip').value)
        self.ft_baudrate = int(self.get_parameter('ft_baudrate').value)
        self.publish_hz = max(1.0, float(self.get_parameter('publish_hz').value))
        self.auto_zero_on_startup = bool(self.get_parameter('auto_zero_on_startup').value)

        # Queue depth 1 keeps teleop/haptics on the latest FT sample.
        self.publisher_ = self.create_publisher(WrenchStamped, '/xarm/ft_data', 1)

        # Service for Zeroing (Tare) if I run ros2 service call /xarm/tare_sensor std_srvs/srv/Trigger {} it is gonna call tare_callback
        self.srv = self.create_service(Trigger, '/xarm/tare_sensor', self.tare_callback)

        self.get_logger().info(f'Connecting to robot at {self.robot_ip}...')
        self.arm = XArmAPI(self.robot_ip)

        # --- ROBUST INITIALIZATION ---
        self.initialize_sensor()

        period_s = 1.0 / self.publish_hz
        self.timer = self.create_timer(period_s, self.timer_callback)
        self.get_logger().info(f'Bridge Started. Publishing data at {self.publish_hz:.1f} Hz...')

    def initialize_sensor(self):
        """Fixes baudrate and resets sensor."""
        self.get_logger().info('Initializing sensor communication...')
        self.arm.clean_error()
        time.sleep(0.5)

        # Force Baudrate
        self.arm.set_tgpio_modbus_baudrate(self.ft_baudrate)
        time.sleep(0.5)

        # Reset Sensor Power
        self.arm.ft_sensor_enable(0)
        time.sleep(0.5)
        self.arm.ft_sensor_enable(1)
        time.sleep(1.0) # Wait for boot

        if self.auto_zero_on_startup:
            self.arm.ft_sensor_set_zero()
            self.get_logger().info('Sensor Initialized and Zeroed.')
        else:
            self.get_logger().info('Sensor Initialized.')

    def tare_callback(self, request, response):
        """ROS2 Service callback to zero the sensor."""
        self.get_logger().info('Received Tare Request...')
        self.arm.ft_sensor_set_zero()
        response.success = True
        response.message = "Sensor Zeroed (Tare) Successfully"
        return response

    def timer_callback(self):
        code, ft_data = self.arm.get_ft_sensor_data()

        if code == 0 and ft_data:
            msg = WrenchStamped()
            msg.header.stamp = self.get_clock().now().to_msg()
            msg.header.frame_id = "ft_sensor_link"

            # Data Mapping
            msg.wrench.force.x = float(ft_data[0])
            msg.wrench.force.y = float(ft_data[1])
            msg.wrench.force.z = float(ft_data[2])
            msg.wrench.torque.x = float(ft_data[3])
            msg.wrench.torque.y = float(ft_data[4])
            msg.wrench.torque.z = float(ft_data[5])

            self.publisher_.publish(msg)

    def shutdown(self):
        self.arm.disconnect()

def main(args=None):
    rclpy.init(args=args)
    node = FtSensorBridge()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.shutdown()
        node.destroy_node()
        rclpy.shutdown()

if __name__ == '__main__':
    main()
