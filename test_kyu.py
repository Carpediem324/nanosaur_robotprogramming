import cv2
import rclpy
import time
import numpy as np
from rclpy.node import Node
from .motor import Motor
import asyncio

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
        
        self.corner_detected = False

    def timer_callback(self):
        ret_val, img = self.cap.read()
        if ret_val:
            linear_velocity, angular_velocity = self.process_image_and_move(img)
            self.control_motors(linear_velocity, angular_velocity)
            
    async def rotate_90_degrees(self):
        # 로봇을 제자리에서 90도 회전하게 하는 코드 (시계방향 회전)
        rotation_duration = 1.5  # 실행할 회전 시간(초)
        target_rpm = 100       # 회전 도중 각 바퀴에서 필요한 RPM
        self.mright.set_speed(-target_rpm)
        self.mleft.set_speed(target_rpm)

        await asyncio.sleep(rotation_duration)

        self.mright.set_speed(0)
        self.mleft.set_speed(0)
        self.corner_detected = False

    def process_image_and_move(self, img):
        left_mask, right_mask = self.generate_masks()
        img_points = cv2.cvtColor(img, cv2.COLOR_BGR2GRAY)

        left_points = cv2.bitwise_and(img_points, left_mask)
        right_points = cv2.bitwise_and(img_points, right_mask)

        left_edge = cv2.Canny(left_points, 100, 120)
        right_edge = cv2.Canny(right_points, 100, 120)

        edge_weight = 1.0 / (len(img_points)*len(img_points[0]))
        threshold = 0.015

        if not self.corner_detected and right_edge.sum()*edge_weight > threshold and left_edge.sum()*edge_weight > threshold:
            self.corner_detected = True
            self.get_logger().info("Corner detected")
            asyncio.run(self.rotate_90_degrees())

        if self.corner_detected:
            linear_velocity = 0.0
            angular_velocity = 0.0
        else:
            left_intensity = left_edge.sum()*edge_weight
            right_intensity = right_edge.sum()*edge_weight

            difference = right_intensity - left_intensity

            linear_velocity = self.base_speed
            angular_velocity = self.base_speed * difference * self.gain

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
