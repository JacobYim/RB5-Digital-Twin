import rclpy
from rclpy.node import Node
from sensor_msgs.msg import JointState
from std_msgs.msg import Header

class JointStatePublisher(Node):
    def __init__(self):
        super().__init__('joint_state_publisher')
        self.publisher_ = self.create_publisher(JointState, '/joint_command', 10)
        timer_period = 0.1  # seconds
        self.timer = self.create_timer(timer_period, self.timer_callback)
        self.i = 0

    def timer_callback(self):
        msg = JointState()
        msg.header = Header()
        msg.header.stamp = self.get_clock().now().to_msg()
        msg.header.frame_id = ''

        # Example joint names and corresponding positions and velocities
        msg.name = ['base', 'shoulder', 'elbow', 'wrist1', 'wrist2', 'wrist3']
        msg.position = [0.0, 0.0, 0.0, 0.0, 0.0, 0.0]
        msg.velocity = [0.0001, -0.0001, 0.0, 0.0, 0.0, 0.0]
        msg.effort = [0.0, 0.0, 0.0, 0.0, 0.0, 0.0]

        self.publisher_.publish(msg)
        self.get_logger().info('Publishing: "%s"' % msg)

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
