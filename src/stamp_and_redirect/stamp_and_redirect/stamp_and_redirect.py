#!/usr/bin/env python3
import rclpy
from rclpy.node import Node
from geometry_msgs.msg import Twist, TwistStamped

class TwistConverterNode(Node):
    def __init__(self):
        super().__init__('twist_converter')
        # Subscriber to receive Twist messages from /cmd_vel_nav
        self.subscription = self.create_subscription(
            Twist,
            '/cmd_vel_nav',
            self.cmd_vel_callback,
            10)
        # Publisher to publish TwistStamped messages to /diff_cont/cmd_vel
        self.publisher = self.create_publisher(TwistStamped, '/diff_cont/cmd_vel', 10)
        self.get_logger().info('Twist Converter Node has been started.')

    def cmd_vel_callback(self, msg: Twist):
        # Create a new TwistStamped message
        twist_stamped = TwistStamped()
        # Stamp the message with the current time
        twist_stamped.header.stamp = self.get_clock().now().to_msg()
        # Set a frame_id (adjust as necessary for your system)
        twist_stamped.header.frame_id = 'base_link'
        # Copy the twist data from the received message
        twist_stamped.twist = msg
        # Publish the stamped message
        self.publisher.publish(twist_stamped)
        self.get_logger().debug('Published a TwistStamped message.')

def main(args=None):
    rclpy.init(args=args)
    node = TwistConverterNode()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()

if __name__ == '__main__':
    main()
