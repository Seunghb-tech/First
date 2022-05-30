#!/usr/bin/env python3

import rclpy
from rclpy.node import Node
from geometry_msgs.msg import Twist
from sensor_msgs.msg import LaserScan
from rclpy.qos import ReliabilityPolicy, QoSProfile


class WallFollowing(Node):
    def __init__(self):
        super().__init__('WallFollowing')
        # self.publisher = self.create_publisher(Twist, 'cmd_vel', 10)
        self.subscriber = self.create_subscription(LaserScan, '/scan', self.move_turtlebot,
                                                   QoSProfile(depth=10, reliability=ReliabilityPolicy.BEST_EFFORT))
        self.subscriber # prevent unused variable warning
        # self.timer_period = 0.2
        # define the variable to save the received info
        self.Laser_forward_left = 0
        self.Laser_forward_right = 0
        self.Laser_right = 0
        # self.cmd = Twist()
        # self.timer = self.create_timer(self.timer_period, self.motion)

    def move_turtlebot(self, msg):
        # self.Laser_forward_right = (msg.ranges[179])
        # self.obsAtRight = min(msg.ranges[130:165])
        # self.obsAtLeft = min(msg.ranges[190:250])
        # self.Laser_right = msg.ranges[89]
        print('length of ranges = ', len(msg.ranges))

    def motion(self):
        # print the data
        self.get_logger().info("I receive in front %s" % str(self.Laser_forward_right))
        self.get_logger().info("I receive in right wall %s" % str(self.Laser_right))

        # Logic to move
        if self.Laser_forward_right < 0.5 or (self.obsAtRight < 0.2 and self.obsAtLeft > 0.3):
            self.cmd.linear.x = 0.02
            self.cmd.angular.z = 0.2
            self.get_logger().info("obstacle detected or front near")
        else:
            if self.Laser_right > 0.3 or self.obsAtLeft < 0.3:
                self.cmd.linear.x = 0.02
                self.cmd.angular.z = -0.1
            elif self.Laser_right < 0.2:
                self.cmd.linear.x = 0.02
                self.cmd.angular.z = 0.1
            else:
                self.cmd.linear.x = 0.02
                self.cmd.angular.z = 0.0
        # publishing the cmd_val to the topics
        self.publisher.publish(self.cmd)


def main(args=None):
    rclpy.init(args=args)
    wall_following = WallFollowing()
    try:
        rclpy.spin(wall_following)
    except KeyboardInterrupt:
        pass
    wall_following.destroy_node()  # explicity destroy the node
    rclpy.shutdown()


if __name__ == '__main__':
    main()

