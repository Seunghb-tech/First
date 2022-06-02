#!/usr/bin/env python3

import rclpy
from rclpy.node import Node
import numpy as np
import cv2
from cv_bridge import CvBridgeError, CvBridge
from sensor_msgs.msg import Image, LaserScan
from rclpy.qos import ReliabilityPolicy, QoSProfile
from geometry_msgs.msg import Twist


def img_process(img):
    height, width = img.shape[:2]  # 480, 640

    x_margin, y_min, y_max= int(0.4 * width), int(0.6 * height), int(0.7 * height)
    y_middle = int((y_min + y_max)/2)

    _, binary_img = cv2.threshold(img, 100, 255, cv2.THRESH_BINARY)

    img = cv2.rectangle(img, (0, y_min), (x_margin, y_max), 125, 3) # draw rectangle
    img = cv2.rectangle(img, (width-x_margin, y_min), (width, y_max), 125, 3)

    l_y, l_x = binary_img[y_min : y_max, 0 : x_margin].nonzero()
    r_y, r_x = binary_img[y_min : y_max, width-x_margin : width].nonzero()

    leftx = np.average(l_x) if len(l_x) > 500 else 0
    rightx = np.average(r_x) + (width - x_margin) if len(r_x) > 500 else width
    midx = int((leftx + rightx)/2)

    cv2.circle(img, (int(leftx), y_middle), 10, 0, -1)
    cv2.circle(img, (int(rightx), y_middle), 10, 0, -1)
    cv2.circle(img, (int(midx), y_middle), 10, 0, -1)

    steer = np.arctan2(midx - (width/2), height-y_middle)
    steer = round(steer,2)
    img = cv2.line(img, (int(width/2), height), (midx, y_middle), 0, 5)  # draw line
    img = cv2.putText(img, str(steer), (150, height), cv2.FONT_HERSHEY_SIMPLEX, 1.5, 125, 3) # write text

    return img, steer


class HocarminiController(Node):
    def __init__(self):
        super().__init__('hocarmini_controller')
        self.steer_ = 0.0  
        self.declare_parameter("hocar_speed", 60.0) 
        self.speed_ = self.get_parameter("hocar_speed").value

        self.front_distance_ = 0.0
        self.image_subscriber_ = self.create_subscription(Image, "image", self.image_callback, 10)
        self.lidar_subscriber_ = self.create_subscription(
            LaserScan, '/scan', self.lidar_callback, QoSProfile(depth=10, reliability=ReliabilityPolicy.BEST_EFFORT))
        self.image_subscriber_  # prevent unused variable warning
        self.lidar_subscriber_  # prevent unused variable warning
        self.br_ = CvBridge()
        self.publisher_ = self.create_publisher(Twist, 'hocar/cmd_vel', 10)
        self.cmd_ = Twist()
        self.timer_period = 0.25
        self.timer = self.create_timer(self.timer_period, self.hocarmini_control)

    def image_callback(self, img):
        # self.get_logger().info('Receiving video frame')
        try:
            cv2_frame = self.br_.imgmsg_to_cv2(img, 'mono8').astype('uint8')  # import image in gray scale
        except CvBridgeError as e:
            self.get_logger().error("CvBridge Error : %r" % (e,))

        processed_img, self.steer_ = img_process(cv2_frame)
        cv2.imshow("camera", processed_img)  # cv2_frame
        cv2.waitKey(1)

    def lidar_callback(self, msg):
        ## print('length of ranges = ', len(msg.ranges)) # 719
        av_dist = msg.ranges[360] # 180 degree, sum(msg.ranges[357:363]) / 6 
        if av_dist != 0.0:
            self.front_distance_ = round(av_dist, 3) # meter unit
            print('Receiving Lidar data: ' + str(self.front_distance_))     

    def hocarmini_control(self):
        # Logic to move
        if self.front_distance_ < 0.3:
            self.cmd_.linear.x = 5.0 # almost stop          
            self.get_logger().info("obstacle is detected at front")
        else:
            self.cmd_.linear.x = self.speed_
        self.cmd_.angular.z = self.steer_
        self.publisher_.publish(self.cmd_)


def main(args=None):
    rclpy.init(args=args)
    hocarmini = HocarminiController()
    try:
        rclpy.spin(hocarmini)
    except KeyboardInterrupt:
        pass
    hocarmini.destroy_node()  # explicity destroy the node
    rclpy.shutdown()

if __name__ == '__main__':
    main()
