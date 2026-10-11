import math

import rclpy
from rclpy.node import Node
from geometry_msgs.msg import Pose
from nav_msgs.msg import Odometry


class SimulatedTarget(Node):
    def __init__(self):
        super().__init__("simulated_target")

        # Person's fixed position in the Gazebo world.
        self.person_x = 2.0
        self.person_y = 0.0

        self.rover_x = None
        self.rover_y = None
        self.rover_yaw = 0.0

        self.publisher = self.create_publisher(
            Pose, "/target_pose", 10
        )

        self.odom_subscription = self.create_subscription(
            Odometry, "/odom", self.odom_callback, 10
        )

        self.timer = self.create_timer(0.05, self.publish_target)

        self.get_logger().info(
            "Simulated target active; waiting for rover odometry."
        )

    def odom_callback(self, msg):
        self.rover_x = msg.pose.pose.position.x
        self.rover_y = msg.pose.pose.position.y

        q = msg.pose.pose.orientation
        self.rover_yaw = math.atan2(
            2.0 * (q.w * q.z + q.x * q.y),
            1.0 - 2.0 * (q.y * q.y + q.z * q.z)
        )

    def publish_target(self):
        # Never publish a fabricated relative position before odometry.
        if self.rover_x is None or self.rover_y is None:
            return

        dx = self.person_x - self.rover_x
        dy = self.person_y - self.rover_y

        # Transform world coordinates into the rover's local frame.
        x_relative = (
            math.cos(self.rover_yaw) * dx
            + math.sin(self.rover_yaw) * dy
        )
        y_relative = (
            -math.sin(self.rover_yaw) * dx
            + math.cos(self.rover_yaw) * dy
        )

        msg = Pose()
        msg.position.x = x_relative
        msg.position.y = y_relative
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
