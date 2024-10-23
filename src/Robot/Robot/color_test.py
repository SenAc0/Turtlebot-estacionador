import rclpy
from rclpy.node import Node
from sensor_msgs.msg import Image
from cv_bridge import CvBridge
import cv2
import numpy as np

class ColorDetector(Node):
    def __init__(self):
        super().__init__('color_detector')
        self.subscription = self.create_subscription(
            Image,
            '/camera/image_raw',  # Asegúrate de que este sea el tema correcto
            self.listener_callback,
            10)
        self.br = CvBridge()

    def listener_callback(self, msg):
        # Conversión de imagen de ROS a OpenCV
        cv_image = self.br.imgmsg_to_cv2(msg, "bgr8")
        
        # Definir el rango de color a detectar (ejemplo: verde)
        lower_green = np.array([35, 100, 100])  # Valores en HSV para el verde
        upper_green = np.array([85, 255, 255])

        # Convertir la imagen a espacio de color HSV
        hsv_image = cv2.cvtColor(cv_image, cv2.COLOR_BGR2HSV)

        # Crear una máscara para el color verde
        mask = cv2.inRange(hsv_image, lower_green, upper_green)

        # Verificar si hay píxeles del color verde en la máscara
        if cv2.countNonZero(mask) > 0:
            print("COLOR ESPECÍFICO DETECTADO")
        else: print(" NO COLOR ESPECÍFICO DETECTADO")

        # Mostrar la imagen original y la máscara (opcional)
        cv2.imshow("Camera Feed", cv_image)
        cv2.imshow("Mask", mask)
        cv2.waitKey(1)

def main(args=None):
    rclpy.init(args=args)
    color_detector = ColorDetector()
    rclpy.spin(color_detector)  # Asegúrate de que esté correctamente llamado
    color_detector.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()
