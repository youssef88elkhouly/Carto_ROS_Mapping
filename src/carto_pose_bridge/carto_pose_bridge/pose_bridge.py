import math
import requests

import rclpy
from rclpy.node import Node
from geometry_msgs.msg import PoseWithCovarianceStamped


def yaw_from_quaternion(q):
    siny_cosp = 2.0 * (q.w * q.z + q.x * q.y)
    cosy_cosp = 1.0 - 2.0 * (q.y * q.y + q.z * q.z)
    return math.atan2(siny_cosp, cosy_cosp)


class CartoPoseBridge(Node):
    def __init__(self):
        super().__init__("carto_pose_bridge")

        self.declare_parameter("edge_url", "http://127.0.0.1:4000/dev/pose")
        self.declare_parameter("topic", "/amcl_pose")
        self.declare_parameter("timeout_sec", 0.5)
        self.declare_parameter("send_rate_hz", 5.0)

        self.edge_url = self.get_parameter("edge_url").value
        self.topic = self.get_parameter("topic").value
        self.timeout_sec = float(self.get_parameter("timeout_sec").value)
        self.send_period = 1.0 / float(self.get_parameter("send_rate_hz").value)

        self.latest_msg = None
        self.last_send_time = self.get_clock().now()

        self.sub = self.create_subscription(
            PoseWithCovarianceStamped,
            self.topic,
            self.pose_callback,
            10
        )

        self.timer = self.create_timer(self.send_period, self.send_latest_pose)

        self.get_logger().info(f"Listening to {self.topic}")
        self.get_logger().info(f"Sending pose to {self.edge_url}")

    def pose_callback(self, msg):
        self.latest_msg = msg

    def send_latest_pose(self):
        if self.latest_msg is None:
            return

        msg = self.latest_msg

        x = msg.pose.pose.position.x
        y = msg.pose.pose.position.y
        yaw = yaw_from_quaternion(msg.pose.pose.orientation)

        payload = {
            "x": x,
            "y": y,
            "yaw": yaw
        }

        try:
            response = requests.post(
                self.edge_url,
                json=payload,
                timeout=self.timeout_sec
            )

            if response.status_code >= 300:
                self.get_logger().warn(
                    f"cart-edge returned {response.status_code}: {response.text}"
                )

        except Exception as e:
            self.get_logger().warn(f"Failed to send pose: {e}")


def main(args=None):
    rclpy.init(args=args)
    node = CartoPoseBridge()

    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass

    node.destroy_node()
    rclpy.shutdown()


if __name__ == "__main__":
    main()
