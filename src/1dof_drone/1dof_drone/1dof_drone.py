#!/usr/bin/env python3
import rclpy
from rclpy.node import Node
from std_msgs.msg import Float32
import numpy as np
import matplotlib.pyplot as plt
import math

class DroneNode(Node):
    def __init__(self):
        super().__init__('drone_node')
        self.get_logger().info("Drone simülasyonu başlatıldı")

        # Fiziksel parametreler
        self.m = 1.0
        self.m1 = 0.5
        self.m2 = 0.5
        self.L = 0.5
        self.I = 0.02
        self.g = 9.81
        self.dt = 0.01

        # Başlangıç durumu
        self.theta = 0.0
        self.omega = 0.0

        # Motor kuvvetleri
        self.F1 = 5.0
        self.F2 = 5.0

        # Publisher
        self.theta_pub = self.create_publisher(Float32, '/theta', 10)

        # Subscriber
        self.create_subscription(Float32, '/motor1/F1', self.f1_callback, 10)
        self.create_subscription(Float32, '/motor2/F2', self.f2_callback, 10)

        # ROS timer ile simülasyon ve publish
        self.create_timer(self.dt, self.timer_callback)

        # =====================
        # Matplotlib interaktif animasyon
        # =====================
        plt.ion()  # interactive mode
        self.fig, self.ax = plt.subplots()
        self.line, = self.ax.plot([], [], 'o-', lw=4)
        self.ax.set_xlim(-1, 1)
        self.ax.set_ylim(-0.5, 0.5)
        self.ax.set_aspect('equal')
        self.fig.show()
        self.fig.canvas.draw()

    # =====================
    # Subscriber callback
    # =====================
    def f1_callback(self, msg):
        self.F1 = msg.data

    def f2_callback(self, msg):
        self.F2 = msg.data

    # =====================
    # Simülasyon
    # =====================
    def update_state(self):
        torque = (self.F2 - self.F1 + self.m1*self.g - self.m2*self.g) * self.L * math.cos(self.theta)
        alpha = torque / self.I
        self.omega += alpha * self.dt
        self.theta += self.omega * self.dt

    # =====================
    # Timer callback (simülasyon + publish + animasyon)
    # =====================
    def timer_callback(self):
        self.update_state()

        # Theta publish
        msg = Float32()
        msg.data = float(self.theta)
        self.theta_pub.publish(msg)

        # Animasyon güncelle
        x = np.array([-self.L, self.L])
        y = np.array([0, 0])
        x_rot = x * np.cos(self.theta) - y * np.sin(self.theta)
        y_rot = x * np.sin(self.theta) + y * np.cos(self.theta)
        self.line.set_data(x_rot, y_rot)
        self.fig.canvas.draw()
        self.fig.canvas.flush_events()

def main(args=None):
    rclpy.init(args=args)
    node = DroneNode()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    node.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()
