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
            '/nanosaur/camera/image_raw',
            self.image_callback,
            10)
        self.cmd_vel_publisher = self.create_publisher(
            Twist,
            '/nanosaur/cmd_vel',
            10)
        # 데스크탑용
        cv2.namedWindow('line follower')
        cv2.imshow('line follower', np.zeros((300,300), dtype=np.uint8))

    def image_callback(self, msg):

        try:
            image = self.cv_bridge.imgmsg_to_cv2(msg, 'bgr8')
        except Exception as e:
            self.get_logger().error('Failed to convert imgmsg to cv2: ' + str(e))
            return
        
        height, width, _ = image.shape
        roi = image[int(height/2):, :]
        gray_image = cv2.cvtColor(roi, cv2.COLOR_BGR2GRAY)
        _, threshold = cv2.threshold(gray_image, 60, 255, cv2.THRESH_BINARY_INV)
        contours, _ = cv2.findContours(
            threshold, cv2.RETR_EXTERNAL, cv2.CHAIN_APPROX_SIMPLE)

        if len(contours) > 0:
            largest_contour = max(contours, key=cv2.contourArea)
            moments = cv2.moments(largest_contour)
            cX = int(moments["m10"] / moments["m00"])
            error = cX - int(width/2)

            twist_msg = Twist()
            twist_msg.angular.z = -float(error) * 0.005
            twist_msg.linear.x = 0.5
            self.cmd_vel_publisher.publish(twist_msg)
        else:
            self.stop_moving()
        
        # 데스크탑용
        cv2.drawContours(roi, [largest_contour], 0, (0,0,255), 3)
        cv2.imshow('line follower', roi)
        
        key = cv2.waitKey(1)
        if key == 27: # ESC
            cv2.destroyAllWindows()


    def stop_moving(self):
        twist_msg = Twist()
        twist_msg.angular.z = 0.0
        twist_msg.linear.x = 0.0
        self.cmd_vel_publisher.publish(twist_msg)

def main(args=None):
    rclpy.init(args=args)

    try:
        line_follower = LineFollower()
        rclpy.spin(line_follower)

    except KeyboardInterrupt:
        pass

    line_follower.stop_moving()
    line_follower.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()
