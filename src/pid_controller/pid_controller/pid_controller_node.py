#!/usr/bin/env python3
import rclpy
from rclpy.node import Node
from std_msgs.msg import Float32

from pid_controller.PIDController import PIDController

class PIDControllerNode(Node):
    def __init__(self):
        super().__init__('pid_controller')
        self.get_logger().info("PID Controller başlatıldı")

        # PID parametreleri
        self.Kp = 3.0
        self.Ki = 0.0
        self.Kd = 1.0

        self.dt = 0.01

        self.pid = PIDController(self.Kp, self.Ki, self.Kd, self.dt)

        # Hedef açı
        self.theta_setpoint = 0.2

        # Publisher
        self.pub_F1 = self.create_publisher(Float32, '/motor1/F1', 10)
        self.pub_F2 = self.create_publisher(Float32, '/motor2/F2', 10)

        # Subscriber
        self.create_subscription(Float32, '/theta', self.theta_callback, 10)
        self.create_subscription(Float32, '/cmd', self.cmd_callback, 10)

    def theta_callback(self, msg):
        theta = msg.data

        # PID hesaplama
        u = self.pid.compute(self.theta_setpoint,theta)

        # PID çıktısını motor kuvvetlerine dönüştür
        F_total = 10.0  # Toplam kuvvet, drone ağırlığını karşılamak için baz
        F1 = F_total - u/2
        F2 = F_total + u/2

        # Limit koy (opsiyonel)
        F1 = max(0.0, F1)
        F2 = max(0.0, F2)

        # Publish et
        msg_F1 = Float32()
        msg_F1.data = F1
        self.pub_F1.publish(msg_F1)

        msg_F2 = Float32()
        msg_F2.data = F2
        self.pub_F2.publish(msg_F2)

    def cmd_callback(self, msg):
        self.theta_setpoint = msg.data

def main(args=None):
    rclpy.init(args=args)
    node = PIDControllerNode()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()
