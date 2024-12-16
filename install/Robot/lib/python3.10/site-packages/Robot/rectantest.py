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


        # Definir el rango de color (azul por ejemplo)
        #self.lower_color = np.array([100, 150, 0])
        #self.upper_color = np.array([140, 255, 255])

        # Variables para optimizar el procesamiento
        self.last_processed_time = time.time()
        self.processing_interval = 0.5  # Procesa imágenes cada 0.5 segundos

        # Bandera para saber si está buscando o avanzando
        self.is_searching = False
        self.target_reached = False  # Nueva bandera para condición de término
        self.last_seen_color_time = time.time()  # Marca de tiempo de la última detección

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
            small_image = cv2.resize(cv_image, (320, 240))  # Resolución moderada

            # Convertir la imagen al espacio de color HSV
            hsv_image = cv2.cvtColor(small_image, cv2.COLOR_BGR2HSV)

            # Crear una máscara para el color objetivo
            mask = cv2.inRange(hsv_image, self.lower_color, self.upper_color)

            # Encontrar contornos en la máscara
            contours, _ = cv2.findContours(mask, cv2.RETR_EXTERNAL, cv2.CHAIN_APPROX_SIMPLE)

            if contours:
                # Encontrar el contorno más grande (rectángulo verde)
                largest_contour = max(contours, key=cv2.contourArea)
                area = cv2.contourArea(largest_contour)

                # Si el área del rectángulo es suficientemente grande, considera alcanzado
                if area > 5000:  # Ajusta este valor según el tamaño esperado del rectángulo
                    self.target_reached = True
                    self.get_logger().info("Rectángulo verde alcanzado. Deteniendo robot.")
                    self.stop_robot()
                    return

                # Obtener el centro del rectángulo
                M = cv2.moments(largest_contour)
                if M['m00'] > 0:
                    cx = int(M['m10'] / M['m00'])
                    cy = int(M['m01'] / M['m00'])

                    # Mover el robot hacia el centro del rectángulo
                    self.move_towards_rectangle(cx, small_image.shape[1])
                else:
                    self.rotate_to_search()
            else:
                self.rotate_to_search()

            # Verificar si no ha visto el rectángulo en 5 segundos
            if time.time() - self.last_seen_color_time > 5.0:
                self.get_logger().info("No se detecta el rectángulo por 5 segundos. Deteniendo robot.")
                self.stop_robot()

        except Exception as e:
            self.get_logger().error(f"Error al procesar la imagen: {e}")

    def move_towards_rectangle(self, cx, image_width):
        # Crear un mensaje Twist para moverse hacia el rectángulo
        msg = Twist()

        # Control proporcional para centrar el robot en el rectángulo
        error = cx - (image_width // 2)
        k_p = 0.002  # Ganancia proporcional para la corrección angular
        msg.angular.z = -k_p * error
        msg.linear.x = 0.05  # Velocidad lineal constante hacia adelante

        # Publicar el mensaje
        self.cmd_vel_publisher.publish(msg)
        self.get_logger().info(f"Moviéndose hacia el rectángulo: error={error}, angular.z={msg.angular.z}")

    def rotate_to_search(self):
        if self.is_searching:
            # Rotar para buscar el rectángulo
            twist = Twist()
            twist.linear.x = 0.0
            twist.angular.z = 0.3  # Velocidad de rotación

            # Publicar comando
            self.cmd_vel_publisher.publish(twist)
            self.get_logger().info("Buscando el rectángulo...")

    def stop_robot(self):
        # Crear un mensaje Twist para detener el robot
        twist = Twist()
        twist.linear.x = 0.0
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
