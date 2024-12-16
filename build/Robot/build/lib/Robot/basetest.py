import rclpy
from rclpy.node import Node
from sensor_msgs.msg import Image
from geometry_msgs.msg import Twist
from cv_bridge import CvBridge
import cv2
import numpy as np
import time

class ColorFollowingRobot(Node):
    def __init__(self):
        super().__init__('color_following_robot')

        # Publicador para el movimiento
        self.cmd_vel_publisher = self.create_publisher(Twist, '/cmd_vel', 10)

        # Suscriptor para la cámara
        self.image_subscription = self.create_subscription(
            Image,
            '/image_raw',
            self.process_image,
            10
        )

        self.br = CvBridge()

        # Definir el rango de color (verde por ejemplo)
        self.lower_color = np.array([35, 100, 100])
        self.upper_color = np.array([85, 255, 255])

        # Variables para optimizar el procesamiento
        self.last_processed_time = time.time()
        self.processing_interval = 0.3  # Procesa imágenes cada 1.0 segundos

        # Bandera para saber si está buscando o avanzando
        self.is_searching = False

        self.target_reached = False
        self.last_seen_color_time = time.time()


    def process_image(self, msg):
        # Limitar la frecuencia de procesamiento
        if time.time() - self.last_processed_time < self.processing_interval:
            return
        self.last_processed_time = time.time()

        if self.target_reached:
            self.get_logger().info("Objetivo alcanzado. Deteniendo robot.")
            self.stop_robot()
            return

        try:
            # Convertir imagen ROS a OpenCV
            cv_image = self.br.imgmsg_to_cv2(msg, "bgr8")

            # Reducir la resolución para optimizar el procesamiento
            resized_image = cv2.resize(cv_image, (80, 60))

            # Convertir la imagen al espacio de color HSV
            hsv_image = cv2.cvtColor(resized_image, cv2.COLOR_BGR2HSV)

            # Crear una máscara para el color objetivo
            mask = cv2.inRange(hsv_image, self.lower_color, self.upper_color)

            # Verificar si hay detección del color verde
            if np.sum(mask) > 0:
                self.is_searching = False
                self.last_seen_color_time = time.time()

                self.move_towards_color()
            else:
                self.is_searching = True
                self.rotate_to_search()


            # Verificar si no ha visto el color en 5 segundos
            if time.time() - self.last_seen_color_time > 20.94:
                self.target_reached = True
                self.get_logger().info("No se detecta el color. Deteniendo robot.")


        except Exception as e:
            self.get_logger().error(f"Error al procesar la imagen: {e}")

    def move_towards_color(self):
        # Crear un mensaje Twist para moverse lentamente
        msg = Twist()
        msg.linear.x = 0.05  # Velocidad lineal hacia adelante (ajusta según desees)
        msg.angular.z = 0.0  # Sin rotación
        
        # Publicar el mensaje
        self.cmd_vel_publisher.publish(msg)
        self.get_logger().info("El robot avanza lentamente.")

    def rotate_to_search(self):
        if self.is_searching:
            # Rotar para buscar el color
            twist = Twist()
            twist.linear.x = 0.0
            twist.angular.z = 0.3  # Velocidad de rotación

            # Publicar comando
            self.cmd_vel_publisher.publish(twist)
            self.get_logger().info("Buscando el color...")

    def stop_robot(self):
        # Crear un mensaje Twist para detener el robot
        twist = Twist()
        twist.angular.z = 0.0

        # Publicar el comando para detener
        self.cmd_vel_publisher.publish(twist)
        self.get_logger().info("Robot detenido.")


def main(args=None):
    rclpy.init(args=args)
    robot = ColorFollowingRobot()
    rclpy.spin(robot)
    robot.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()
