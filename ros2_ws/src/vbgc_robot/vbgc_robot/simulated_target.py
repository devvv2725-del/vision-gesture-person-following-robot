import rclpy
from rclpy.node import Node
from geometry_msgs.msg import Pose


class SimulatedTarget(Node):
    def __init__(self):
        super().__init__("simulated_target")

        self.publisher = self.create_publisher(
            Pose, "/target_pose", 10
        )

        self.timer = self.create_timer(0.05, self.publish_target)
        self.get_logger().info("Stationary target publisher started.")

    def publish_target(self):
        msg = Pose()

        # Temporary rover-relative target for testing.
        msg.position.x = 2.0
        msg.position.y = 0.0
        msg.position.z = 0.0
        msg.orientation.w = 1.0

        self.publisher.publish(msg)


def main(args=None):
    rclpy.init(args=args)
    node = SimulatedTarget()

    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == "__main__":
    main()
