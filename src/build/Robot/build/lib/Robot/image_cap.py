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
            '/camera/image_raw',  # Cambia esto si tu tema es diferente
            self.image_callback,
            10)
        self.bridge = CvBridge()

    def image_callback(self, msg):
        # Convertir el mensaje de imagen a una imagen de OpenCV
        cv_image = self.bridge.imgmsg_to_cv2(msg, desired_encoding='bgr8')

        # Mostrar la imagen
        cv2.imshow("Camera Feed", cv_image)
        cv2.waitKey(1)  # Esperar un pequeño tiempo para que se muestre la imagen

def main(args=None):
    rclpy.init(args=args)
    image_capturer = ImageCapturer()
    rclpy.spin(image_capturer)
    image_capturer.destroy_node()
    cv2.destroyAllWindows()
    rclpy.shutdown()

if __name__ == '__main__':
    main()
