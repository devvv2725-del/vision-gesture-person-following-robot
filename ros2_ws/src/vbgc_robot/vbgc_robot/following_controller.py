import math

import rclpy
from rclpy.node import Node
from geometry_msgs.msg import Pose, Twist
from std_msgs.msg import String


class FollowingController(Node):
    def __init__(self):
        super().__init__("following_controller")

        self.desired_distance = 1.2
        self.distance_tolerance = 0.10

        self.max_linear_speed = 0.12
        self.max_angular_speed = 0.6
        self.target_timeout = 0.5

        self.follow_enabled = False
        self.target_x = None
        self.target_y = None
        self.last_target_time = None
        self.target_lost_logged = False

        self.cmd_vel_publisher = self.create_publisher(
            Twist, "/cmd_vel", 10
        )

        self.gesture_subscription = self.create_subscription(
            String, "/gesture_command", self.gesture_callback, 10
        )

        self.target_subscription = self.create_subscription(
            Pose, "/target_pose", self.target_callback, 10
        )

        self.control_timer = self.create_timer(0.05, self.control_loop)

        self.get_logger().info(
            "Following controller started; desired distance 1.2 m."
        )

    def gesture_callback(self, msg):
        command = msg.data.strip().upper()

        if command == "FOLLOW":
            self.follow_enabled = True
            self.get_logger().info("FOLLOW enabled.")
        elif command in ("STOP", "DOCK"):
            self.follow_enabled = False
            self.publish_stop()
            self.get_logger().info(f"{command} received. Rover stopped.")

    def target_callback(self, msg):
        self.target_x = msg.position.x
        self.target_y = msg.position.y
        self.last_target_time = self.get_clock().now()
        self.target_lost_logged = False

    def publish_stop(self):
        self.cmd_vel_publisher.publish(Twist())

    def control_loop(self):
        if not self.follow_enabled:
            self.publish_stop()
            return

        if self.target_x is None or self.target_y is None:
            self.publish_stop()
            return

        if self.last_target_time is None:
            self.publish_stop()
            return

        target_age = (
            self.get_clock().now() - self.last_target_time
        ).nanoseconds / 1e9

        if target_age > self.target_timeout:
            self.publish_stop()
            if not self.target_lost_logged:
                self.get_logger().warning(
                    "Target data stale. Rover stopped."
                )
                self.target_lost_logged = True
            return

        distance = math.hypot(self.target_x, self.target_y)
        target_angle = math.atan2(self.target_y, self.target_x)

        twist = Twist()

        twist.angular.z = max(
            -self.max_angular_speed,
            min(
                self.max_angular_speed,
                1.2 * target_angle
            )
        )

        # Only move forward when the target is ahead and outside
        # the desired following distance plus tolerance.
        if (
            self.target_x > 0.0
            and distance > self.desired_distance + self.distance_tolerance
            and abs(target_angle) < 0.6
        ):
            distance_error = distance - self.desired_distance
            twist.linear.x = min(
                self.max_linear_speed,
                0.6 * distance_error
            )
        else:
            twist.linear.x = 0.0

        self.cmd_vel_publisher.publish(twist)

    def destroy_node(self):
        self.publish_stop()
        super().destroy_node()


def main(args=None):
    rclpy.init(args=args)
    node = FollowingController()

    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == "__main__":
    main()
