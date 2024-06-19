import rclpy
from rclpy.node import Node
from sensor_msgs.msg import JointState
import csv
import os

class JointStateSubscriber(Node):
    def __init__(self):
        super().__init__('joint_state_subscriber')
        self.subscription = self.create_subscription(
            JointState,
            '/joint_states',
            self.listener_callback,
            10)
        self.subscription  # prevent unused variable warning

        # Setting up CSV file to save joint state data
        self.filename = "joint_states.csv"
        # Check if file exists to avoid overwriting it
        if not os.path.exists(self.filename):
            with open(self.filename, mode='w', newline='') as file:
                writer = csv.writer(file)
                writer.writerow(["time_sec", "time_nanosec", "frame_id", "name", "position", "velocity", "effort"])

    def listener_callback(self, msg):
        # Open file in append mode and write joint state data
        with open(self.filename, mode='a', newline='') as file:
            writer = csv.writer(file)
            for i in range(len(msg.name)):
                writer.writerow([
                    msg.header.stamp.sec,
                    msg.header.stamp.nanosec,
                    msg.header.frame_id,
                    msg.name[i],
                    msg.position[i] if i < len(msg.position) else 'n/a',
                    msg.velocity[i] if i < len(msg.velocity) else 'n/a',
                    msg.effort[i] if i < len(msg.effort) else 'n/a'
                ])

def main(args=None):
    rclpy.init(args=args)
    joint_state_subscriber = JointStateSubscriber()
    try:
        rclpy.spin(joint_state_subscriber)
    except KeyboardInterrupt:
        print('Joint State Subscriber stopped cleanly')
    except BaseException:
        print('Exception in Joint State Subscriber:', file=sys.stderr)
        raise
    finally:
        # Destroy the node explicitly
        joint_state_subscriber.destroy_node()
        rclpy.shutdown()

if __name__ == '__main__':
    main()
