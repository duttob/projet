import tkinter as tk
from tkinter import ttk
import random
import threading
import time
import rclpy
from rclpy.node import Node
from rclpy.action import ActionClient
from geometry_msgs.msg import Twist
from irobot_create_msgs.msg import HazardDetectionVector, IrIntensityVector
from irobot_create_msgs.action import Dock, Undock
from rclpy.qos import qos_profile_sensor_data
from enum import Enum


class RobotControlNode(Node):
    def __init__(self):
        super().__init__('robot_control_node')
        self.get_logger().info("Robot control node started")

        # Publishers and Actions
        self.cmd_vel_pub = self.create_publisher(Twist, '/Robot4/cmd_vel', 10)
        self.dock_client = ActionClient(self, Dock, '/Robot4/dock')
        self.undock_client = ActionClient(self, Undock, '/Robot4/undock')

        # Subscriptions for obstacle/hazard detection
        self.hazard_sub = self.create_subscription(
            HazardDetectionVector, '/Robot4/hazard_detection', self.hazard_callback, qos_profile_sensor_data)
        self.ir_sub = self.create_subscription(
            IrIntensityVector, '/Robot4/ir_intensity', self.ir_callback, qos_profile_sensor_data)

        # State variables
        self.fsm_state = 'undock'
        self.fsm_start_time = time.time()
        self.avoid_timer = 0
        self.manual_override = False
        self.manual_command = 'stop'
        self.speed_factor = 0.5
        self.is_undocking = False
        self.blocked = False

        self.timer = self.create_timer(0.25, self.timer_callback)

        # GUI setup
        self.setup_gui()

    def setup_gui(self):
        self.root = tk.Tk()
        self.root.title("ROS2 Robot Hybrid Control")
        self.root.geometry("600x400")
        self.root.configure(bg="#2c3e50")
        self.root.protocol("WM_DELETE_WINDOW", self.on_closing)

        frame = tk.Frame(self.root, bg="#34495e")
        frame.pack(fill=tk.BOTH, expand=True, padx=10, pady=10)

        # Mode toggle
        self.mode_button = ttk.Button(frame, text="Passer en manuel", command=self.toggle_mode)
        self.mode_button.pack(pady=10)

        # Manual controls
        control_frame = tk.LabelFrame(frame, text="Contrôles manuels", bg="#34495e", fg="white")
        control_frame.pack(fill=tk.BOTH, expand=True, padx=5, pady=5)

        dir_frame = tk.Frame(control_frame, bg="#34495e")
        dir_frame.pack(pady=10)

        ttk.Button(dir_frame, text="▲", width=8, command=lambda: self.set_manual_command("forward")).grid(row=0, column=1)
        ttk.Button(dir_frame, text="◄", width=8, command=lambda: self.set_manual_command("rotate_left")).grid(row=1, column=0)
        ttk.Button(dir_frame, text="■", width=8, command=lambda: self.set_manual_command("stop")).grid(row=1, column=1)
        ttk.Button(dir_frame, text="►", width=8, command=lambda: self.set_manual_command("rotate_right")).grid(row=1, column=2)
        ttk.Button(dir_frame, text="▼", width=8, command=lambda: self.set_manual_command("backward")).grid(row=2, column=1)

        # Speed control
        speed_frame = tk.Frame(control_frame, bg="#34495e")
        speed_frame.pack(fill=tk.X, pady=10)

        tk.Label(speed_frame, text="Vitesse:", bg="#34495e", fg="white").pack(side=tk.LEFT, padx=5)
        self.speed_var = tk.IntVar(value=50)
        speed_slider = ttk.Scale(speed_frame, from_=0, to=100, variable=self.speed_var,
                                 command=self.update_speed, orient=tk.HORIZONTAL)
        speed_slider.pack(side=tk.LEFT, fill=tk.X, expand=True, padx=5)

        self.speed_label = tk.Label(speed_frame, text="50%", bg="#34495e", fg="white")
        self.speed_label.pack(side=tk.LEFT, padx=5)

        # Programmable buttons
        prog_frame = tk.LabelFrame(frame, text="Fonctions", bg="#34495e", fg="white")
        prog_frame.pack(fill=tk.X, padx=5, pady=5)

        ttk.Button(prog_frame, text="Dock", command=self.dock).pack(side=tk.LEFT, padx=10, pady=5)
        ttk.Button(prog_frame, text="Undock", command=self.undock).pack(side=tk.LEFT, padx=10, pady=5)

        self.status_bar = tk.Label(self.root, text="Statut: Initialisation", bg="#7f8c8d", fg="white", anchor=tk.W)
        self.status_bar.pack(side=tk.BOTTOM, fill=tk.X)

    def toggle_mode(self):
        self.manual_override = not self.manual_override
        mode = "manuel" if self.manual_override else "autonome"
        self.status_bar.config(text=f"Mode: {mode}")
        self.mode_button.config(text="Passer en auto" if self.manual_override else "Passer en manuel")

    def set_manual_command(self, direction):
        self.manual_command = direction
        self.status_bar.config(text=f"Commande manuelle: {direction}")

    def update_speed(self, *args):
        self.speed_factor = self.speed_var.get() / 100.0
        self.speed_label.config(text=f"{self.speed_var.get()}%")

    def timer_callback(self):
        if self.manual_override:
            self.handle_manual_control()
        else:
            if self.blocked:
                self.stop_robot()
                return
            self.handle_fsm()

    def handle_manual_control(self):
        msg = Twist()
        speed = self.speed_factor * 0.5

        if self.manual_command == "forward":
            msg.linear.x = speed
        elif self.manual_command == "backward":
            msg.linear.x = -speed
        elif self.manual_command == "rotate_left":
            msg.angular.z = speed
        elif self.manual_command == "rotate_right":
            msg.angular.z = -speed

        self.cmd_vel_pub.publish(msg)

    def handle_fsm(self):
        twist = Twist()
        speed = self.speed_factor * 0.5

        if self.fsm_state == 'undock' and not self.is_undocking:
            self.send_undock_goal()
            self.is_undocking = True

        elif self.fsm_state == 'wander':
            if time.time() - self.fsm_start_time > 20:  # Wander for 20s
                self.fsm_state = 'dock'
                self.status_bar.config(text="Statut: FSM → dock")
                return
            twist.linear.x = speed
            self.cmd_vel_pub.publish(twist)
            self.status_bar.config(text="Statut: FSM → avancer")

        elif self.fsm_state == 'avoid':
            if time.time() < self.avoid_timer:
                twist.angular.z = random.choice([-1.0, 1.0]) * speed
                self.cmd_vel_pub.publish(twist)
                self.status_bar.config(text="Statut: FSM → évitement")
            else:
                self.fsm_state = 'wander'
                self.fsm_start_time = time.time()  # resume wander

        elif self.fsm_state == 'dock':
            self.send_dock_goal()
            self.fsm_state = 'done'

        elif self.fsm_state == 'done':
            self.status_bar.config(text="Statut: FSM terminé")


    def send_dock_goal(self):
        if not self.dock_client.wait_for_server(timeout_sec=2.0):
            self.get_logger().error("Dock server not available")
            self.status_bar.config(text="Dock échoué")
            return
        self.get_logger().info("Sending dock goal")
        goal = Dock.Goal()
        self.dock_client.send_goal_async(goal).add_done_callback(lambda f: self.status_bar.config(text="Dock envoyé"))


    def send_undock_goal(self):
        if not self.undock_client.wait_for_server(timeout_sec=2.0):
            self.get_logger().error("Undock server not available")
            self.status_bar.config(text="Undock échoué")
            return

        self.get_logger().info("Sending undock goal")
        goal = Undock.Goal()
        self.undock_client.send_goal_async(goal).add_done_callback(self.undock_done)


    def undock_done(self, future):
        result = future.result()
        self.get_logger().info("Undock terminé")
        self.status_bar.config(text="Statut: Undock terminé")
        self.fsm_state = 'wander'


    def dock(self):
        if not self.dock_client.wait_for_server(timeout_sec=2.0):
            self.get_logger().error("Dock server not available")
            self.status_bar.config(text="Dock échoué")
            return

        self.get_logger().info("Sending dock goal")
        goal = Dock.Goal()
        self.dock_client.send_goal_async(goal).add_done_callback(lambda f: self.status_bar.config(text="Dock envoyé"))


    def undock(self):
        self.send_undock_goal()


    def hazard_callback(self, msg):
        if self.manual_override:
            return
        if msg.detections and self.fsm_state == 'wander':
            self.get_logger().warn("Hazard detected! Avoiding.")
            self.stop_robot()
            self.blocked = False  # Clear blocked
            self.fsm_state = 'avoid'
            self.avoid_timer = time.time() + 2.0


    def ir_callback(self, msg):
        if self.manual_override:
            return
        for reading in msg.readings:
            if reading.value > 1000 and self.fsm_state == 'wander':
                self.get_logger().warn("High IR detected! Avoiding.")
                self.stop_robot()
                self.blocked = False
                self.fsm_state = 'avoid'
                self.avoid_timer = time.time() + 2.0
                return


    def stop_robot(self):
        self.cmd_vel_pub.publish(Twist())


    def on_closing(self):
        self.stop_robot()
        self.root.destroy()


    def spin(self):
        def ros_spin():
            while rclpy.ok():
                rclpy.spin_once(self, timeout_sec=0.01)
                time.sleep(0.01)

        threading.Thread(target=ros_spin, daemon=True).start()
        self.root.mainloop()


def main(args=None):
    rclpy.init(args=args)
    node = RobotControlNode()
    try:
        node.spin()
    except KeyboardInterrupt:
        node.get_logger().info("Shutting down.")
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()
