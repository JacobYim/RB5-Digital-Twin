from rclpy.node import Node
from sensor_msgs.msg import JointState
from std_msgs.msg import Header
import rclpy
import math

class JointStatePublisher(Node):
    def __init__(self):
        super().__init__('joint_state_publisher')
        self.publisher_ = self.create_publisher(JointState, 'joint_command', 10)
        self.timer_period = 0.01  # seconds
        self.timer = self.create_timer(self.timer_period, self.timer_callback)
        self.i = 0

    def timer_callback(self):
        msg = JointState()
        msg.header = Header()
        msg.header.stamp = self.get_clock().now().to_msg()
        msg.header.frame_id = ''

        # Calculate the sine wave for each joint index
        amplitude = 1.0  # amplitude of the sine wave
        frequency = 1.0  # frequency of the sine wave

        # Example joint names and generating sine wave positions
        msg.name = ['base', 'shoulder', 'elbow', 'wrist1', 'wrist2', 'wrist3']
        msg.position = [
            amplitude * math.sin(frequency * self.i * self.timer_period),
            amplitude * math.sin(frequency * self.i * self.timer_period + 1),
            amplitude * math.sin(frequency * self.i * self.timer_period + 2),
            amplitude * math.sin(frequency * self.i * self.timer_period + 3),
            amplitude * math.sin(frequency * self.i * self.timer_period + 4),
            amplitude * math.sin(frequency * self.i * self.timer_period + 5)
        ]
        msg.velocity = [0.0001, -0.0001, 0.0, 0.0, 0.0, 0.0]
        msg.effort = [0.0, 0.0, 0.0, 0.0, 0.0, 0.0]

        self.publisher_.publish(msg)
        self.get_logger().info('Publishing: "%s"' % msg)

        self.i += 1  # Increment to change the sine argument over time

def main(args=None):
    rclpy.init(args=args)
    joint_state_publisher = JointStatePublisher()

    try:
        rclpy.spin(joint_state_publisher)
    except KeyboardInterrupt:
        print('Stopping the publisher.')
    finally:
        joint_state_publisher.destroy_node()
        rclpy.shutdown()

if __name__ == '__main__':
    main()
