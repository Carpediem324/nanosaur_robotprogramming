import cv2
import rclpy
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

def find_black_lines(img):
    hsv_img = cv2.cvtColor(img, cv2.COLOR_BGR2HSV)
    lower_black = np.array([0, 0, 0])
    upper_black = np.array([180, 255, 75])
    mask = cv2.inRange(hsv_img, lower_black, upper_black)
    return mask

class LineFollower(Node):
    def __init__(self):
        super().__init__('line_follower')
        self.cmd_vel_publisher_ = self.create_publisher(Twist, 'cmd_vel', 10)
        self.cap = cv2.VideoCapture(gstreamer_pipeline(flip_method=2), cv2.CAP_GSTREAMER)
        self.line_speed = 0.2
        self.angular_speed = 0.5
        self.timer_period = 0.01
        self.timer = self.create_timer(self.timer_period, self.timer_callback)

    def timer_callback(self):
        ret_val, img = self.cap.read()

        if ret_val:
            self.process_image_and_move(img)

    def process_image_and_move(self, img):
        black_lines = find_black_lines(img)
        edges = cv2.Canny(black_lines, 50, 150, apertureSize=3)
        lines = cv2.HoughLines(edges, 1, np.pi / 180, 100)

        if lines is not None:
            self.follow_line(lines)
        else:
            self.stop_robot()

        cv2.imshow("Processed Image", black_lines)
        cv2.waitKey(1)

    def follow_line(self, lines):
        error = self.calculate_error_from_line(lines)
        twist = Twist()
        twist.linear.x = float(self.line_speed)
        twist.angular.z = -float(error) * float(self.angular_speed)
        self.cmd_vel_publisher_.publish(twist)

    def calculate_error_from_line(self, lines):
        x_middle = self.cap.get(cv2.CAP_PROP_FRAME_WIDTH) / 2
        x_mean = np.mean([line[0][0] * np.cos(line[0][1]) for line in lines])

        error = (x_middle - x_mean) / x_middle
        return error

    def stop_robot(self):
        twist = Twist()
        twist.linear.x = float(0)
        twist.angular.z = float(0)
        self.cmd_vel_publisher_.publish(twist)

def main(args=None):
    rclpy.init(args=args)
    line_follower = LineFollower()

    rclpy.spin(line_follower)
    
    line_follower.cap.release()
    line_follower.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()
