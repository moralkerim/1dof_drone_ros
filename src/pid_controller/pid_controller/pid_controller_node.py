#!/usr/bin/env python3
import rclpy
from rclpy.node import Node
from std_msgs.msg import Float32
from rclpy.parameter import Parameter
from rcl_interfaces.msg import SetParametersResult

from pid_controller.PIDController import PIDController

class PIDControllerNode(Node):
    def __init__(self):
        super().__init__('pid_controller')
        self.get_logger().info("PID Controller başlatıldı")

        # ======================
        # Parametreleri declare et
        # ======================
        self.declare_parameter('Kp', 3.0)
        self.declare_parameter('Ki', 0.0)
        self.declare_parameter('Kd', 1.0)

        # Parametreleri al
        self.Kp = self.get_parameter('Kp').value
        self.Ki = self.get_parameter('Ki').value
        self.Kd = self.get_parameter('Kd').value

        # Hedef açı
        self.theta_setpoint = 0.0

        #Simulasyon güncelleme süresi
        self.dt = 0.01

        self.pid = PIDController(self.Kp, self.Ki, self.Kd, self.dt)

        # Publisher
        self.pub_F1 = self.create_publisher(Float32, '/motor1/F1', 10)
        self.pub_F2 = self.create_publisher(Float32, '/motor2/F2', 10)

        # Subscriber
        self.create_subscription(Float32, '/theta', self.theta_callback, 10)
        self.create_subscription(Float32, '/cmd', self.cmd_callback, 10)

        self.add_on_set_parameters_callback(self.parameter_update_callback)

    # =====================================
    # ROS2 parameter değişince çağrılır
    # =====================================
    def parameter_update_callback(self, params):
        for param in params:
            if param.name == 'Kp' and param.type_ == Parameter.Type.DOUBLE:
                self.pid.Kp = param.value
                self.get_logger().info(f"Kp güncellendi: {param.value}")
            elif param.name == 'Ki' and param.type_ == Parameter.Type.DOUBLE:
                self.pid.Ki = param.value
                self.get_logger().info(f"Ki güncellendi: {param.value}")
            elif param.name == 'Kd' and param.type_ == Parameter.Type.DOUBLE:
                self.pid.Kd = param.value
                self.get_logger().info(f"Kd güncellendi: {param.value}")
            elif param.name == 'setpoint' and param.type_ == Parameter.Type.DOUBLE:
                self.theta_setpoint = param.value
                self.get_logger().info(f"Setpoint güncellendi: {param.value}")
        return SetParametersResult(successful=True)

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
