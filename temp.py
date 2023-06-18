import cv2
import rclpy
import time
import numpy as np
from rclpy.node import Node
from .motor import Motor

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
    upper_black = np.array([180, 255, 100])
    mask = cv2.inRange(hsv_img, lower_black, upper_black)
    return mask

class LineFollower(Node):
    def __init__(self):
        super().__init__('line_follower')
        self.cap = cv2.VideoCapture(gstreamer_pipeline(flip_method=2), cv2.CAP_GSTREAMER)
        self.timer_period = 0.01
        self.timer = self.create_timer(self.timer_period, self.timer_callback)
 
        self.radius = 0.0105
        self.wheel_separation = 0.086

        left_id = 1
        right_id = 2
        self.rpm = 150
        self.mright = Motor(right_id, self.rpm)
        self.mleft = Motor(left_id, self.rpm)

    def timer_callback(self):
        ret_val, img = self.cap.read()
        if ret_val:
            linear_velocity, angular_velocity = self.process_image_and_move(img)
            self.control_motors(linear_velocity, angular_velocity)
      
    def process_image_and_move(self, img):
        black_regions = find_black_regions(img)
        height, width = black_regions.shape
        left_area = black_regions[int(height * 0.6):int(height * 0.8), :int(width * 1/3)]
        center_area = black_regions[int(height * 0.3):, int(width * 1/3):int(width * 2/3)]
        right_area = black_regions[int(height * 0.6):int(height * 0.8), int(width * 2/3):]

        edge_weight = 15

        left_edge = black_regions[int(height * 0.6):int(height * 0.8), int(width * 0.1):int(width * 1/3)]
        right_edge = black_regions[int(height * 0.6):int(height * 0.8), int(width * 2/3):int(width * 0.9)]

        linear_velocity = 0.0
        angular_velocity = 0.0

        if center_area.sum() > left_area.sum() + left_edge.sum() * edge_weight and center_area.sum() > right_area.sum() + right_edge.sum() * edge_weight:
            linear_velocity = 0.5
            angular_velocity = 0.0
        elif left_area.sum() + left_edge.sum() * edge_weight > center_area.sum() and left_area.sum() + left_edge.sum() * edge_weight > right_area.sum() + right_edge.sum() * edge_weight:
            z_compensation = (right_area.sum() - left_area.sum()) / (left_area.sum() + right_area.sum())
            linear_velocity = 0.15
            angular_velocity = 0.5 + z_compensation
        elif right_area.sum() + right_edge.sum() * edge_weight > center_area.sum() and right_area.sum() + right_edge.sum() * edge_weight > left_area.sum() + left_edge.sum() * edge_weight:
            z_compensation = (left_area.sum() - right_area.sum()) / (left_area.sum() + right_area.sum())
            linear_velocity = 0.15
            angular_velocity = -0.5 - z_compensation
        else:
            linear_velocity = 0.0
            angular_velocity = 0.0

        cv2.imshow("Processed Image", black_regions)
        cv2.waitKey(1)

        return linear_velocity, angular_velocity

    def control_motors(self, linear_velocity, angular_velocity):
        v = linear_velocity
        w = angular_velocity
        half_wheel_separation = self.wheel_separation / 2.
        vr = v + half_wheel_separation * w
        vl = v - half_wheel_separation * w
        rr = vr / self.radius
        rl = vl / self.radius
        max_speed = self.rpm / 60
        r = [max(-max_speed, min(max_speed, rr)), max(-max_speed, min(max_speed, rl))]
        rpmr = r[0] * 60
        rpml = r[1] * 60
        self.mright.set_speed(rpmr)
        self.mleft.set_speed(rpml)

def main(args=None):
    rclpy.init(args=args)
    line_follower = LineFollower()

    rclpy.spin(line_follower)

    line_follower.cap.release()
    line_follower.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()
