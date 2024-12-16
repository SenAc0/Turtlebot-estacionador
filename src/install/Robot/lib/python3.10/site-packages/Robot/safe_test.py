import rclpy
from rclpy.node import Node
from sensor_msgs.msg import LaserScan
from geometry_msgs.msg import Twist

class SafetyStop(Node):
    def __init__(self):
        super().__init__('safety_stop')
        
        # Crear suscriptor para los datos del LiDAR
        self.scan_subscriber = self.create_subscription(
            LaserScan,
            '/scan',  # Tema del LiDAR en TurtleBot3
            self.scan_callback,
            10
        )
        
        # Crear publicador para controlar el movimiento
        self.cmd_vel_publisher = self.create_publisher(Twist, '/cmd_vel', 10)
        
        # Variable para almacenar el estado de seguridad
        self.safe_to_move = True
        # Distancia de seguridad en metros
        self.safety_distance = 0.3  # Ajusta según sea necesario

    def scan_callback(self, scan_msg):
        # Verificar si hay un obstáculo dentro de la distancia de seguridad
        min_distance = min(scan_msg.ranges)  # Obtiene la distancia más cercana
        if min_distance < self.safety_distance:
            # Si la distancia es menor a la distancia de seguridad, detiene el movimiento
            self.safe_to_move = False
            self.stop_robot()
        else:
            self.safe_to_move = True

    def stop_robot(self):
        # Publicar un mensaje Twist con velocidades en cero para detener el robot
        stop_msg = Twist()
        stop_msg.linear.x = 0.0
        stop_msg.angular.z = 0.0
        self.cmd_vel_publisher.publish(stop_msg)
        self.get_logger().info("Obstáculo detectado: Movimiento detenido.")

def main(args=None):
    rclpy.init(args=args)
    safety_stop = SafetyStop()
    rclpy.spin(safety_stop)
    safety_stop.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()
