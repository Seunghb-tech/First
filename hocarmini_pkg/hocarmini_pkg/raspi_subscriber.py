#!/usr/bin/env python3
# This node should be run in Raspberry Pi
import rclpy
from rclpy.node import Node
from rclpy.qos import QoSProfile
from geometry_msgs.msg import Twist

from .hocar import Hocar
h_mini = Hocar(16, 20, 21, 14, 0, 90)  # (ENA, IN1, IN2, ENS, 0, 90)

# Motor direction
STOP  = 0; FORWARD  = 1; BACKWARD = 2

class HocarminiSubacriber(Node):
    def __init__(self):
        super().__init__("hocarmini_subscriber")
        qos_profile = QoSProfile(depth=10)
        self.hocarmini_subscriber_ = self.create_subscription(
            Twist, "hocar/cmd_vel", self.subscribe_cmdvel_topic, qos_profile)
        self.speed_ = 0
        self.angle_ = 90
        self.get_logger().info("Hocarmini subscriber has been started.")

    def subscribe_cmdvel_topic(self, msg):
        # speed, angle control
        self.speed_ = int(msg.linear.x)  # 80(go) or 0(stop)
        h_mini.setMotorControl(self.speed_, FORWARD)
        self.angle_ = int(msg.angular.z * 100 + 90)
        if self.angle_ > 120: self.angle_ = 120
        elif self.angle_ < 60: self.angle_ = 60 
        h_mini.setServoAngle(self.angle_)
        self.get_logger().info("Speed: {0}, Angle: {1}".format(self.speed_, self.angle_))


def main(args=None):
    rclpy.init(args=args)
    node = HocarminiSubacriber()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
            node.get_logger().info("Keyboard Interupt happeded")
    node.destroy_node()
    rclpy.shutdown()


if __name__ == "__main__":
    main()
