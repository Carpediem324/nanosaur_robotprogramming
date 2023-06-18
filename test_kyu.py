import rclpy
from rclpy.node import Node
from sensor_msgs.msg import Image
from cv_bridge import CvBridge
import cv2
import numpy as np
from geometry_msgs.msg import Twist
import time

class LineFollower(Node):
    def __init__(self):
        super().__init__('line_follower')
        self.subscription = self.create_subscription(
            Image,
            '/camera/color/image_raw',
            self.timer_callback,
            10)
        self.subscription  # Prevent unused variable warning

        self.publisher = self.create_publisher(Twist, '/cmd_vel', 10)

        self.bridge = CvBridge()
        
        self.direction = None  # None for uninitialized, 0 for center, 1 for left, -1 for right
        self.threshold = 100  # Set a suitable threshold value
        self.delay = 1  # Delay in seconds between direction changes

    def timer_callback(self, msg):
        try:
            img = self.bridge.imgmsg_to_cv2(msg, desired_encoding='bgr8')
        except Exception as e:
            self.get_logger().info(str(e))
            return

        linear_velocity, angular_velocity, new_direction = self.process_image_and_move(img)

        # Check if direction change is required and a sufficient delay has passed
        if self.direction != new_direction and (self.direction is None or time.time() - self.direction_timestamp >= self.delay):
            self.direction = new_direction
            self.direction_timestamp = time.time()

        # Control the motors
        self.control_motors(linear_velocity, self.direction * angular_velocity)

    def process_image_and_move(self, img):
        lower_yellow = np.array([10, 10, 10])
        upper_yellow = np.array([255, 255, 80])
        mask = cv2.inRange(img, lower_yellow, upper_yellow)

        height, width, _ = img.shape
        top = int(2 * height / 5)
        bottom = int(2 * height / 3)

        left_mask = mask[top:bottom, 0:int(width / 3)]
        center_mask = mask[top:bottom, int(width / 3):int(2 * width / 3)]
        right_mask = mask[top:bottom, int(2 * width / 3):width]

        left_area = cv2.countNonZero(left_mask)
        center_area = cv2.countNonZero(center_mask)
        right_area = cv2.countNonZero(right_mask)

        linear_velocity = 0.2
        angular_velocity = 0.25

        # Determine the new direction based on the conditions
        if center_area > left_area and center_area > right_area:
            new_direction = 0
        elif left_area > center_area and left_area > right_area:
            new_direction = 1
        elif right_area > center_area and right_area > left_area:
            new_direction = -1
        else:
            new_direction = 0

        return linear_velocity, angular_velocity, new_direction

    def control_motors(self, linear, angular):
        twist = Twist()
        twist.linear.x = linear
        twist.angular.z = angular
        self.publisher.publish(twist)

def main(args=None):
    rclpy.init(args=args)

    line_follower = LineFollower()

    rclpy.spin(line_follower)

    # Destroy the node explicitly
    line_follower.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()
