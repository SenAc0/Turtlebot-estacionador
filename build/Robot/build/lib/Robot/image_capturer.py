import rclpy
from rclpy.node import Node
from sensor_msgs.msg import Image
from cv_bridge import CvBridge
import cv2

class ImageCapturer(Node):
    def __init__(self):
        super().__init__('image_capturer')
        self.subscription = self.create_subscription(
            Image,
            '/camera/image_raw',  # Asegúrate de que este sea el tema correcto
            self.listener_callback,
            10)
        self.br = CvBridge()

    def listener_callback(self, msg):
        # Conversión de imagen de ROS a OpenCV
        cv_image = self.br.imgmsg_to_cv2(msg, "bgr8")
        cv2.imshow("Camera Feed", cv_image)
        cv2.waitKey(1)

def main(args=None):
    rclpy.init(args=args)
    image_capturer = ImageCapturer()
    rclpy.spin(image_capturer)
    image_capturer.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()
