import rclpy
from rclpy.node import Node
from sensor_msgs.msg import Image
from geometry_msgs.msg import Twist
import cv2
import numpy as np
from cv_bridge import CvBridge

class LineFollower(Node):
    def __init__(self):
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

    def image_callback(self, msg):
        try:
            image = self.cv_bridge.imgmsg_to_cv2(msg, 'mono8')
            height, width = image.shape

            # 이미지 크기 축소
            scale_percent = 50
            new_width = int(width * scale_percent / 100)
            new_height = int(height * scale_percent / 100)
            image = cv2.resize(image, (new_width, new_height), interpolation=cv2.INTER_AREA)

        except Exception as e:
            self.get_logger().error('Failed to convert imgmsg to cv2: ' + str(e))
            return

        roi = image[new_height//2:, :]

        centerX = new_width//2
        leftX = centerX//2
        rightX = centerX + leftX

        center_roi = roi[:, centerX - leftX:centerX + leftX]
        left_roi = roi[:, :leftX]
        right_roi = roi[:, rightX:]

        _, center_threshold = cv2.threshold(center_roi, 60, 255, cv2.THRESH_BINARY_INV)
        center_contours, _ = cv2.findContours(center_threshold, cv2.RETR_EXTERNAL, cv2.CHAIN_APPROX_SIMPLE)

        _, left_threshold = cv2.threshold(left_roi, 60, 255, cv2.THRESH_BINARY_INV)
        left_contours, _ = cv2.findContours(left_threshold, cv2.RETR_EXTERNAL, cv2.CHAIN_APPROX_SIMPLE)

        _, right_threshold = cv2.threshold(right_roi, 60, 255, cv2.THRESH_BINARY_INV)
        right_contours, _ = cv2.findContours(right_threshold, cv2.RETR_EXTERNAL, cv2.CHAIN_APPROX_SIMPLE)

        # 직진을 위한 코드
        if len(center_contours) > 0:
            twist_msg = Twist()
            twist_msg.linear.x = 0.5
            self.cmd_vel_publisher.publish(twist_msg)

        # 회전을 위한 코드
        else:
            if len(left_contours) > 0:
                twist_msg = Twist()
                twist_msg.angular.z = 0.5
                twist_msg.linear.x = 0.0
                self.cmd_vel_publisher.publish(twist_msg)
            elif len(right_contours) > 0:
                twist_msg = Twist()
                twist_msg.angular.z = -0.5
                twist_msg.linear.x = 0.0
                self.cmd_vel_publisher.publish(twist_msg)
            else:
                self.stop_moving()

    def stop_moving(self):
        twist_msg = Twist()
        twist_msg.angular.z = 0.0
        twist_msg.linear.x = 0.0
        self.cmd_vel_publisher.publish(twist_msg)

def main(args=None):
    rclpy.init(args=args)
    line_follower = LineFollower()
    rclpy.spin(line_follower)

    line_follower.stop_moving()
    line_follower.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()
