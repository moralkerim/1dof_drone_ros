#!/usr/bin/env python3
import rclpy
from rclpy.node import Node
import numpy as np
import matplotlib.pyplot as plt
from matplotlib.animation import FuncAnimation
import math

class DroneNode(Node):
    def __init__(self):
        super().__init__('drone_node')
        self.get_logger().info("Drone simulasyonu baslatildi.")

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

        # Animasyon hazırlığı
        self.fig, self.ax = plt.subplots()
        self.ax.set_xlim(-1, 1)
        self.ax.set_ylim(-0.5, 0.5)
        self.ax.set_aspect('equal')
        self.line, = self.ax.plot([], [], 'o-', lw=4)

        # Animasyon başlat
        self.ani = FuncAnimation(self.fig, self.animate, frames=1000, interval=10, blit=True, init_func=self.init)
        plt.show()

    def update_state(self):
        torque = (self.F2 - self.F1 + self.m1*self.g - self.m2*self.g) * self.L * math.cos(self.theta)
        alpha = torque / self.I
        self.omega += alpha * self.dt
        self.theta += self.omega * self.dt

    def init(self):
        self.line.set_data([], [])
        return self.line,

    def animate(self, frame):
        self.update_state()
        x = np.array([-self.L, self.L])
        y = np.array([0, 0])
        x_rot = x * np.cos(self.theta) - y * np.sin(self.theta)
        y_rot = x * np.sin(self.theta) + y * np.cos(self.theta)
        self.line.set_data(x_rot, y_rot)
        return self.line,

def main(args=None):
    rclpy.init(args=args)
    node = DroneNode()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()
