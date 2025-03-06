import rclpy
from rclpy.node import Node
from sensor_msgs.msg import Joy
from geometry_msgs.msg import TwistStamped

class InputFromJoy(Node):
    def __init__(self):
        super().__init__('input_from_joystick')
        
        # Create a publisher for the /diff_cont/cmd_vel topic (now using TwistStamped)
        self.cmd_vel_publisher = self.create_publisher(TwistStamped, 'diff_cont/cmd_vel', 10)
        
        # Create a subscriber to the /joy topic
        self.joy_subscriber = self.create_subscription(
            Joy,
            '/joy',  # Joystick input topic
            self.joy_callback,
            10
        )
    
    def joy_callback(self, msg: Joy):
        # Create a TwistStamped message instead of Twist
        twist_msg = TwistStamped()

        # Add a timestamp
        twist_msg.header.stamp = self.get_clock().now().to_msg()
        twist_msg.header.frame_id = "base_link"  # Optional frame reference

        # Map the axes from the joystick to the Twist message
        twist_msg.twist.linear.x = msg.axes[1]  # Forward/backward (left stick vertical)
        twist_msg.twist.linear.y = msg.axes[0]  # Left/right (left stick horizontal)
        twist_msg.twist.angular.z = msg.axes[3]  # Rotation (right stick horizontal)

        # Publish the TwistStamped message to /diff_cont/cmd_vel
        self.cmd_vel_publisher.publish(twist_msg)

def main(args=None):
    rclpy.init(args=args)
    node = InputFromJoy()
    
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()

if __name__ == '__main__':
    main()
