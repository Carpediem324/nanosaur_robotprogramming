import cv2
import rclpy
import time
from rclpy.node import Node
from sensor_msgs.msg import CameraInfo
from geometry_msgs.msg import Twist
import numpy as np

def gstreamer_pipeline(capture_width=640, capture_height=480, display_width=640, display_height=480, framerate=30, flip_method=0):
    return (
        "nvarguscamerasrc ! "
        "video/x-raw(memory:NVMM), "
        "width=(int)%d, height=(int)%d, "
        "format=(string)NV12, framerate=(fraction)%d/1 ! "
        "nvvidconv flip-method=%d ! "
        "video/x-raw, width=(int)%d, height=(int)%d, format=(string)BGRx ! "
        "videoconvert ! "
        "video/x-raw, format=(string)BGR ! appsink"
        % (
            capture_width,
            capture_height,
            framerate,
            flip_method,
            display_width,
            display_height,
        )
    )

def find_black_regions(img):
    hsv_img = cv2.cvtColor(img, cv2.COLOR_BGR2HSV)
    lower_black = np.array([0, 0, 0])
    upper_black = np.array([180, 255, 100]) # Increase the value range to include gray tones
    mask = cv2.inRange(hsv_img, lower_black, upper_black)
    return mask

class LineFollower(Node):
    def __init__(self):
        super().__init__('line_follower')
        self.cmd_vel_publisher_ = self.create_publisher(Twist, 'cmd_vel', 10)
        self.cap = cv2.VideoCapture(gstreamer_pipeline(flip_method=2), cv2.CAP_GSTREAMER)
        self.timer_period = 0.01
        self.timer = self.create_timer(self.timer_period, self.timer_callback)
        self.last_twist = None

    def timer_callback(self):
        ret_val, img = self.cap.read()
        if ret_val:
            self.process_image_and_move(img)

    def process_image_and_move(self, img):
        black_regions = find_black_regions(img)
        height, width = black_regions.shape
        center_bottom = black_regions[int(height * 0.75):, int(width * 0.45):int(width * 0.55)]
        center_left = black_regions[int(height * 0.4):int(height * 0.6), :int(width * 0.4)]
        center_right = black_regions[int(height * 0.4):int(height * 0.6), int(width * 0.6):]

        twist = Twist()

        left_value = center_left.sum()
        right_value = center_right.sum()

        if center_bottom.sum() > left_value and center_bottom.sum() > right_value:
            # Move straight when the center-bottom is black
            twist.linear.x = 0.5
            twist.angular.z = 0.0
        elif left_value > center_bottom.sum() and left_value > right_value:
            # Turn left when the center-left is black
            # Update the angular velocity based on the difference between right and left values
            twist.linear.x = 0.25
            twist.angular.z = min((left_value - right_value) / 10000, 0.5) # 수정된 부분
        elif right_value > center_bottom.sum() and right_value > left_value:
            # Turn right when the center-right is black
            # Update the angular velocity based on the difference between left and right values
            twist.linear.x = 0.25
            twist.angular.z = -min((right_value - left_value) / 10000, 0.5) # 수정된 부분
        else:
            # Stop if no black region is dominant
            twist.linear.x = 0.0
            twist.angular.z = 0.0

        if self.last_twist is None or (twist.linear.x != self.last_twist.linear.x or twist.angular.z != self.last_twist.angular.z):
            self.cmd_vel_publisher_.publish(twist)
            self.last_twist = twist
            time.sleep(0.1)

        cv2.imshow("Processed Image", black_regions)
        cv2.waitKey(1)

def main(args=None):
    rclpy.init(args=args)
    line_follower = LineFollower()

    rclpy.spin(line_follower)
    
    line_follower.cap.release()
    line_follower.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()
