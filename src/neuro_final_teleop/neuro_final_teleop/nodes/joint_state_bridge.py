#!/usr/bin/env python3
import rclpy
from rclpy.node import Node
from sensor_msgs.msg import JointState
from xarm.wrapper import XArmAPI

ROBOT_IP = "192.168.1.243"

class JointPublisher(Node):
    def __init__(self):
        super().__init__('joint_publisher')
        self.declare_parameter('robot_ip', ROBOT_IP)
        self.robot_ip = str(self.get_parameter('robot_ip').value)
        self.publisher_ = self.create_publisher(JointState, '/joint_states', 10)
        self.get_logger().info(f'Connecting to robot at {self.robot_ip}...')
        self.arm = XArmAPI(self.robot_ip)
        self.joint_names = ['joint1', 'joint2', 'joint3', 'joint4', 'joint5', 'joint6', 'joint7']
        self.timer = self.create_timer(0.05, self.timer_callback) # 20Hz

    def timer_callback(self):
        code, angles = self.arm.get_servo_angle(is_radian=True)
        if code == 0 and angles:
            msg = JointState()
            msg.header.stamp = self.get_clock().now().to_msg()
            msg.name = self.joint_names
            msg.position = angles[:7]
            self.publisher_.publish(msg)

    def shutdown(self):
        self.arm.disconnect()

def main(args=None):
    rclpy.init(args=args)
    node = JointPublisher()
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
