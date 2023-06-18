import rclpy
from rclpy.node import Node
from sensor_msgs.msg import Image
from geometry_msgs.msg import Twist
import cv2
import numpy as np
from cv_bridge import CvBridge


class LineFollower(Node):
    def __init__(self) -> None:
        super().__init__('line_follower')

        self.cv_bridge = CvBridge()

        self.image_subscriber = self.create_subscription(
            Image,
            '/camera/image_raw',
            self.image_callback,
            10)
        self.cmd_vel_publisher = self.create_publisher(
            Twist,
            '/cmd_vel',
            10)

        self.movement = {"STOP": (0, 0),
                         "LEFT": (0.5, 0),
                         "RIGHT": (-0.5, 0),
                         "FORWARD": (0, 0.5)}

    def image_callback(self, msg) -> None:
        try:
            image = self.cv_bridge.imgmsg_to_cv2(msg, 'mono8')

        except Exception as e:
            self.get_logger().error('Failed to convert imgmsg to cv2: ' + str(e))
            return

        _, width = image.shape
        _, binary = cv2.threshold(image, 30, 255, cv2.THRESH_BINARY_INV)
        points = find_center(binary)

        move = control_nanosaur(points, width)

        dz, dx = self.movement[move]
        twist_msg = Twist()
        twist_msg.angular.z = dz
        twist_msg.linear.x = dx
        self.cmd_vel_publisher.publish(twist_msg)


def find_center(binary_image) -> list:
    contours, _ = cv2.findContours(binary_image, cv2.RETR_LIST, cv2.CHAIN_APPROX_SIMPLE)

    center_points = []
    for contour in contours:
        M = cv2.moments(contour)

        if M["m00"] != 0:
            cx = int(M["m10"] / M["m00"])
            cy = int(M["m01"] / M["m00"])
            center_points.append((cx, cy))

    return center_points


def control_nanosaur(center_points, frame_width) -> str:
    action = "STOP"
    if center_points:
        for point in center_points:
            if point[0] < frame_width // 2 - 100:
                action = "LEFT"
            elif point[0] > frame_width // 2 + 100:
                action = "RIGHT"
            else:
                action = "FORWARD"

    return action


def main(args=None):
    rclpy.init(args=args)
    line_follower = LineFollower()
    rclpy.spin(line_follower)

    line_follower.stop_moving()
    line_follower.destroy_node()
    rclpy.shutdown()


if __name__ == '__main__':
    main()
