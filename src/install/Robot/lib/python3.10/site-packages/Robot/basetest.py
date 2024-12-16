import rclpy
from rclpy.node import Node
from sensor_msgs.msg import Image
from geometry_msgs.msg import Twist
from cv_bridge import CvBridge
import cv2
import numpy as np

class ColorFollowingRobot(Node):
    def __init__(self):
        super().__init__('color_following_robot')
        
        # Publicador para el movimiento
        self.cmd_vel_publisher = self.create_publisher(Twist, '/cmd_vel', 10)
        
        # Suscriptor para la cámara
        self.image_subscription = self.create_subscription(
            Image,
            '/camera/image_raw',
            self.process_image,
            10
        )
        
        self.br = CvBridge()
        
        # Definir el rango de color (verde por ejemplo)
        self.lower_color = np.array([35, 100, 100])
        self.upper_color = np.array([85, 255, 255])

    def process_image(self, msg):
        # Convertir imagen ROS a OpenCV
        cv_image = self.br.imgmsg_to_cv2(msg, "bgr8")
        
        # Convertir la imagen al espacio de color HSV
        hsv_image = cv2.cvtColor(cv_image, cv2.COLOR_BGR2HSV)
        
        # Crear una máscara para el color objetivo
        mask = cv2.inRange(hsv_image, self.lower_color, self.upper_color)
        
        # Encontrar los momentos de la máscara
        moments = cv2.moments(mask)
        if moments['m00'] > 0:
            # Calcular el centroide del color detectado
            cx = int(moments['m10'] / moments['m00'])
            cy = int(moments['m01'] / moments['m00'])
            
            # Obtener el ancho de la imagen
            height, width, _ = cv_image.shape
            
            # Desviación del centro de la imagen
            error = cx - width // 2
            
            # Control del robot
            twist = Twist()
            twist.linear.x = 0.1  # Velocidad hacia adelante
            twist.angular.z = -float(error) / 100  # Ajuste basado en el error
            
            # Publicar comando
            self.cmd_vel_publisher.publish(twist)
            self.get_logger().info(f"Moviendo hacia el color. Error: {error}")
        else:
            # Si no se detecta el color, detenerse
            twist = Twist()
            twist.linear.x = 0.0
            twist.angular.z = 0.0
            self.cmd_vel_publisher.publish(twist)
            self.get_logger().info("Color no detectado. Detenido.")

        # Mostrar las imágenes para depuración
        cv2.imshow("Camera Feed", cv_image)
        cv2.imshow("Mask", mask)
        cv2.waitKey(1)

def main(args=None):
    rclpy.init(args=args)
    robot = ColorFollowingRobot()
    rclpy.spin(robot)
    robot.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()
