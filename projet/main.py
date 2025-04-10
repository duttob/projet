import tkinter as tk
from tkinter import ttk
import threading
import time
import rclpy
from rclpy.node import Node
from geometry_msgs.msg import Twist
from std_msgs.msg import String
from enum import Enum
from irobot_create_msgs.action import Dock
from rclpy.action import ActionClient

class States(Enum):
    STOP = 0
    FORWARD = 1
    BACKWARD = 2
    ROTATE_RIGHT = 3
    ROTATE_LEFT = 4
    DOCKING = 5
    UNDOCKING = 6

class RobotControlNode(Node):
    def __init__(self):
        super().__init__('robot_control_node')
        
        self.cmd_vel_pub = self.create_publisher(Twist, 'Robot4/cmd_vel', 10)
        
        self.speed_factor = 0.5
        self.state = States.STOP
        self.timer = self.create_timer(0.25, self.control_cycle)
        
        self.dock_client = ActionClient(self, Dock, 'Robot4/dock')
        
        self.setup_gui()
        
    def setup_gui(self):
        self.root = tk.Tk()
        self.root.title("ROS2 Robot Control")
        self.root.geometry("600x400")
        self.root.configure(bg="#2c3e50")
        self.root.protocol("WM_DELETE_WINDOW", self.on_closing)
        
        self.main_frame = tk.Frame(self.root, bg="#34495e")
        self.main_frame.pack(fill=tk.BOTH, expand=True, padx=10, pady=10)
        
        self.setup_control_frame()
        self.setup_programmable_buttons()
        
        self.status_bar = tk.Label(self.root, text="Statut: ROS2 Node prêt", bd=1, relief=tk.SUNKEN, anchor=tk.W, bg="#7f8c8d", fg="white")
        self.status_bar.pack(side=tk.BOTTOM, fill=tk.X)
        
    def setup_control_frame(self):
        control_frame = tk.LabelFrame(self.main_frame, text="Contrôles", padx=5, pady=5, bg="#34495e", fg="white")
        control_frame.pack(fill=tk.BOTH, expand=True, padx=5, pady=5)
        
        dir_frame = tk.Frame(control_frame, bg="#34495e")
        dir_frame.pack(expand=True, pady=20)
        
        fwd_btn = ttk.Button(dir_frame, text="▲", width=8, command=lambda: self.move("forward"))
        fwd_btn.grid(row=0, column=1, padx=10, pady=5)
        
        mid_frame = tk.Frame(dir_frame, bg="#34495e")
        mid_frame.grid(row=1, column=0, columnspan=3)
        
        left_btn = ttk.Button(mid_frame, text="◄", width=8, command=lambda: self.move("rotate_left"))
        left_btn.grid(row=0, column=0, padx=10, pady=5)
        
        stop_btn = ttk.Button(mid_frame, text="■", width=8, command=lambda: self.move("stop"))
        stop_btn.grid(row=0, column=1, padx=10, pady=5)
        
        right_btn = ttk.Button(mid_frame, text="►", width=8, command=lambda: self.move("rotate_right"))
        right_btn.grid(row=0, column=2, padx=10, pady=5)
        
        back_btn = ttk.Button(dir_frame, text="▼", width=8, command=lambda: self.move("backward"))
        back_btn.grid(row=2, column=1, padx=10, pady=5)
        
        speed_frame = tk.Frame(control_frame, bg="#34495e")
        speed_frame.pack(fill=tk.X, pady=10)
        
        tk.Label(speed_frame, text="Vitesse:", bg="#34495e", fg="white").pack(side=tk.LEFT, padx=5)
        
        self.speed_var = tk.IntVar(value=50)
        speed_slider = ttk.Scale(speed_frame, from_=0, to=100, orient=tk.HORIZONTAL, 
                               variable=self.speed_var, command=self.update_speed)
        speed_slider.pack(side=tk.LEFT, fill=tk.X, expand=True, padx=5)
        
        self.speed_label = tk.Label(speed_frame, text="50%", bg="#34495e", fg="white", width=5)
        self.speed_label.pack(side=tk.LEFT, padx=5)
        
    def setup_programmable_buttons(self):
        prog_frame = tk.LabelFrame(self.main_frame, text="Fonctions programmables", padx=5, pady=5, bg="#34495e", fg="white")
        prog_frame.pack(side=tk.BOTTOM, fill=tk.X, padx=5, pady=5)
        
        for i in range(6):
            btn = ttk.Button(prog_frame, text=f"F{i+1}", command=lambda i=i: self.execute_function(i+1))
            btn.grid(row=i//3, column=i%3, padx=10, pady=5, sticky="ew")
            
        for i in range(3):
            prog_frame.columnconfigure(i, weight=1)
    
    def control_cycle(self):
        msg = Twist()
        msg.linear.x = 0.0
        msg.linear.y = 0.0
        msg.linear.z = 0.0
        msg.angular.x = 0.0
        msg.angular.y = 0.0
        msg.angular.z = 0.0

        speed = self.speed_factor*0.5
        if (self.state == States.STOP):
            self.status_bar.config(text="Statut: Arrêt")
            return

        if (self.state == States.FORWARD):
            self.get_logger().info("Moving forward ...")
            msg.linear.x = speed
            msg.angular.z = 0.0
            self.cmd_vel_pub.publish(msg)
            self.status_bar.config(text=f"Statut: Avancer à {self.speed_var.get()}%")
            return
        
        if (self.state == States.BACKWARD):
            self.get_logger().info("Moving backward ...")
            msg.linear.x = -speed
            msg.angular.z = 0.0
            self.status_bar.config(text=f"Statut: Reculer à {self.speed_var.get()}%")
            self.cmd_vel_pub.publish(msg)
            return

        if (self.state == States.ROTATE_LEFT):
            self.get_logger().info("Rotating left ...")
            msg.linear.x = 0.0
            msg.angular.z = 1.0
            self.status_bar.config(text=f"Statut: Tourner à gauche à {self.speed_var.get()}%")
            self.cmd_vel_pub.publish(msg)
            return

        if (self.state == States.ROTATE_RIGHT):
            self.get_logger().info("Rotating right ...")
            msg.linear.x = 0.0
            msg.angular.z = -1.0
            self.cmd_vel_pub.publish(msg)
            self.status_bar.config(text=f"Statut: Tourner à droite à {self.speed_var.get()}%")
            return
        if (self.state == States.DOCKING):
            self.get_logger().info("Docking ...")
            
            if not self.dock_client.wait_for_server(timeout_sec=5.0):
                self.get_logger().error("Dock action server not available!")
                self.state = States.STOP
                return

            goal_msg = Dock.Goal()
            self.get_logger().info("Sending docking request...")
            goal= self.dock_client.send_goal_async(goal_msg)

            rclpy.spin_until_future_complete(self, goal)
            
            self.state = States.STOP
            return
            

    def move(self, direction):
        if direction == "forward":
            self.state = States.FORWARD
        elif direction == "backward":
            self.state = States.BACKWARD
        elif direction == "rotate_left":
            self.state = States.ROTATE_LEFT
        elif direction == "rotate_right":
            self.state = States.ROTATE_RIGHT
        elif direction == "stop":
            self.state = States.STOP
    
    def update_speed(self, *args):
        self.speed_factor = self.speed_var.get() / 100.0
        self.speed_label.config(text=f"{self.speed_var.get()}%")
    
    def execute_function(self, function_num):
        """ TODO """
        self.status_bar.config(text=f"Statut: Exécution de la fonction F{function_num}")
        
        "Dock"
        if function_num == 1:
            self.state = States.DOCKING
    
    def on_closing(self):
        stop_msg = Twist()
        self.cmd_vel_pub.publish(stop_msg)
        self.root.destroy()
    
    def spin(self):
        def process_ros():
            while rclpy.ok():
                rclpy.spin_once(self, timeout_sec=0.01)
                time.sleep(0.01)
        
        ros_thread = threading.Thread(target=process_ros, daemon=True)
        ros_thread.start()
        
        self.root.mainloop()
        
        if hasattr(self, 'ros_thread') and self.ros_thread.is_alive():
            self.ros_thread.join(timeout=1.0)

def main(args=None):
    rclpy.init(args=args)
    
    robot_control = RobotControlNode()
    
    try:
        robot_control.spin()
    except KeyboardInterrupt:
        pass
    finally:
        robot_control.destroy_node()
        rclpy.shutdown()

if __name__ == '__main__':
    main()
