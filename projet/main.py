import rclpy
from rclpy.node import Node
from geometry_msgs.msg import Twist
from irobot_create_msgs.msg import HazardDetectionVector, IrIntensityVector  # Import IrIntensityVector
from rclpy.qos import qos_profile_sensor_data

class ICreate3Basic(Node):
    def __init__(self):
        super().__init__('icreate3_basic')
        
        self.cmd_vel_publisher = self.create_publisher(Twist, '/Robot4/cmd_vel', 10)
        
        self.hazard_subscriber = self.create_subscription(
            HazardDetectionVector,
            '/Robot4/hazard_detection',
            self.hazard_callback,
            qos_profile_sensor_data)
        
        self.timer = self.create_timer(0.4, self.move_forward)
        self.get_logger().info("Nœud iCreate3 démarré.")

    def move_forward(self):
        """ Envoie une commande pour avancer tout droit """
        twist = Twist()
        twist.linear.x = 0.4  # Avancer à 0.2 m/s
        twist.angular.z = 0.0  # Pas de rotation
        self.cmd_vel_publisher.publish(twist)
        self.get_logger().info("Commande envoyée : avancer")

    def hazard_callback(self, msg):
        """ Callback pour les alertes des capteurs """
        if msg.detections:
            self.get_logger().info(f"Détection de danger : {msg.detections}")
            self.get_logger().warn("Obstacle détecté ! Arrêt du robot.")
            twist = Twist()  # Stopper le robot
            twist.linear.x = -0.5
            self.cmd_vel_publisher.publish(twist)
            twist = Twist()
            twist.angular.z = 1.8
            self.cmd_vel_publisher.publish(twist)

def main(args=None):
    rclpy.init(args=args)
    node = ICreate3Basic()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()
