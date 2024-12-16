import rclpy
from rclpy.node import Node
from geometry_msgs.msg import Twist

class SlowForward(Node):
    def __init__(self):
        super().__init__('slow_forward')
        
        # Publicador para enviar comandos de velocidad
        self.cmd_vel_publisher = self.create_publisher(Twist, '/cmd_vel', 10)
        
        # Crear un temporizador para enviar comandos de velocidad continuamente
        self.timer = self.create_timer(0.1, self.move_forward)  # Publicar cada 0.1 segundos

    def move_forward(self):
        # Crear un mensaje Twist para moverse lentamente
        msg = Twist()
        msg.linear.x = 0.05  # Velocidad lineal hacia adelante (ajusta según desees)
        msg.angular.z = 0.0  # Sin rotación
        
        # Publicar el mensaje
        self.cmd_vel_publisher.publish(msg)
        self.get_logger().info("El robot avanza lentamente.")

def main(args=None):
    rclpy.init(args=args)
    slow_forward = SlowForward()
    rclpy.spin(slow_forward)
    slow_forward.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()
