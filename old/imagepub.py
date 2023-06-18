import cv2
import rclpy
from rclpy.node import Node
from sensor_msgs.msg import Image
import numpy as np

def gstreamer_pipeline(
    capture_width=640,
    capture_height=480,
    display_width=640,
    display_height=480,
    framerate=20,
    flip_method=0,
):
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

class ImagePublisher(Node):
    def __init__(self):
        super().__init__('image_publisher')
        self.publisher_ = self.create_publisher(Image, '/camera/image_raw', 10)
        self.timer_period = 1.0/30  # 30 Hz
        self.timer = self.create_timer(self.timer_period, self.timer_callback)
        self.img_pipeline = gstreamer_pipeline(flip_method=2)
        self.cap = cv2.VideoCapture(self.img_pipeline, cv2.CAP_GSTREAMER)

    def timer_callback(self):
        ret_val, img = self.cap.read()
        if ret_val:
            image = Image()
            image.encoding = 'bgr8'
            image.width = img.shape[1]
            image.height = img.shape[0]
            image.step = img.strides[0]
            image.data = img.reshape(-1).tobytes()
            self.publisher_.publish(image)



def main(args=None):
    rclpy.init(args=args)
    image_publisher = ImagePublisher()
    rclpy.spin(image_publisher)
    #clean shutdown
    image_publisher.cap.release()
    cv2.destroyAllWindows()

    image_publisher.destroy_node()
    rclpy.shutdown()


if __name__ == '__main__':
    main()
