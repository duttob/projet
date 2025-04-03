import rclpy
from rclpy.node import Node
from geometry_msgs.msg import Twist
from irobot_create_msgs.msg import HazardDetectionVector, IrIntensityVector  # Import IrIntensityVector

class ICreate3Basic(Node):
    def __init__(self):
        super().__init__('icreate3_basic')
        
        # Publisher pour envoyer des commandes de mouvement
        self.cmd_vel_publisher = self.create_publisher(Twist, '/Robot4/cmd_vel', 10)
        
        # Subscriber pour écouter les capteurs (ex: détection d'obstacles)
        self.hazard_subscriber = self.create_subscription(
            HazardDetectionVector,
            '/Robot4/hazard_detection',
            self.hazard_callback,
            10)
        
        # Subscriber pour écouter les données d'intensité IR
        self.ir_intensity_subscriber = self.create_subscription(
            IrIntensityVector,
            '/Robot4/ir_intensity',
            self.ir_intensity_callback,
            10)
        
        # Timer pour envoyer des commandes de mouvement toutes les 2 secondes
        self.timer = self.create_timer(2.0, self.move_forward)
        self.get_logger().info("Nœud iCreate3 démarré.")

    def move_forward(self):
        """ Envoie une commande pour avancer tout droit """
        twist = Twist()
        twist.linear.x = 0.2  # Avancer à 0.2 m/s
        twist.angular.z = 0.0  # Pas de rotation
        self.cmd_vel_publisher.publish(twist)
        self.get_logger().info("Commande envoyée : avancer")

    def hazard_callback(self, msg):
        """ Callback pour les alertes des capteurs """
        if msg.detections:
            self.get_logger().info(f"Détection de danger : {msg.detections}")
            self.get_logger().warn("Obstacle détecté ! Arrêt du robot.")
            twist = Twist()  # Stopper le robot
            self.cmd_vel_publisher.publish(twist)

    def ir_intensity_callback(self, msg):
        """ Callback pour les données d'intensité IR """
        for reading in msg.readings:
            self.get_logger().info(f"Intensité IR : {reading.value}")
            if reading.value > 1000:  
                self.get_logger().warn(f"Obstacle détecté par IR ! Intensité : {reading.value}")
                twist = Twist() 
                self.cmd_vel_publisher.publish(twist)
                return  

def main(args=None):
    rclpy.init(args=args)
    node = ICreate3Basic()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()