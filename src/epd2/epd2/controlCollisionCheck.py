#!/usr/bin/python3
# This Python file uses the following encoding: utf-8

import sys
import math
import rclpy
from rclpy.node import Node
import tf2_ros
import tf2_geometry_msgs  # Registra tipos de geometry_msgs con tf2
from geometry_msgs.msg import TwistStamped, PointStamped, PoseStamped
from sensor_msgs.msg import LaserScan

class TurtlebotControlCollisionCheck(Node):
    def __init__(self):
        super().__init__('controlCollisionCheck')
        
        self.goal = None
        self.laser = None
        
        # 1. Declarar y leer parámetros de velocidad máxima
        self.declare_parameter('max_lin_vel', 0.5)
        self.declare_parameter('max_ang_vel', 1.0)

        self.max_lin_vel = self.get_parameter('max_lin_vel').value
        self.max_ang_vel = self.get_parameter('max_ang_vel').value
        
        self.get_logger().info(f'Parámetros: v_max={self.max_lin_vel}, w_max={self.max_ang_vel}')
        
        # Configuración de TF2
        self.tf_buffer = tf2_ros.Buffer()
        self.tf_listener = tf2_ros.TransformListener(self.tf_buffer, self)
        
        # Publisher para comandos de velocidad
        self.cmd_vel = self.create_publisher(TwistStamped, 'cmd_vel', 10)

        # Suscripción al láser (scan)
        self.scan_sub = self.create_subscription(LaserScan, 'scan', self.scan_callback, 10)
        
        # Suscripción al objetivo (goal_pose) desde RViz o terminal
        self.goal_sub = self.create_subscription(PoseStamped, 'goal_pose', self.goal_callback, 10)
        
        # Bucle de control a 10Hz
        timer_period = 0.1
        self.timer = self.create_timer(timer_period, self.timer_callback)
    
    def scan_callback(self, data):
        self.laser = data
        
    def goal_callback(self, goal):
        self.get_logger().info("¡Objetivo recibido! x: %.2f, y:%.2f" % (goal.pose.position.x, goal.pose.position.y))
        self.goal = goal
    
    def timer_callback(self):
        self.command()
        
    def command(self):
        if self.goal is None:
            self.publish(0.0, 0.0)
            return
        
        g = self.goal
        g.header.stamp = rclpy.time.Time()
        
        try:
            # Transformar el objetivo al sistema de referencia del robot (base_footprint)
            base_goal = self.tf_buffer.transform(g, 'base_footprint', timeout=rclpy.duration.Duration(seconds=1.0))
        except tf2_ros.TransformException as e:
            self.get_logger().warn(f"Fallo en la transformación: {e}")
            return
            
        x = base_goal.pose.position.x
        y = base_goal.pose.position.y
        
        # --- LEY DE CONTROL (Basada en EPD 1) ---
        rho = math.sqrt(x**2 + y**2) # Distancia
        alpha = math.atan2(y, x)     # Ángulo
        
        kp_lin = 0.4
        kp_ang = 0.8
        
        linear = kp_lin * rho
        angular = kp_ang * alpha
        
        # Si llegamos al destino, paramos
        if rho < 0.15:
            linear = 0.0
            angular = 0.0
            self.goal = None # Reseteamos el objetivo para dejar de movernos
            self.get_logger().info("¡Objetivo alcanzado!")

        # --- DETECTOR DE COLISIONES ---
        if self.checkCollision():
            self.get_logger().warn('¡COLISIÓN POSIBLE! Deteniendo el robot...')
            linear = 0.0
            angular = 0.0
        
        self.publish(linear, angular)

    def checkCollision(self):
        if self.laser is None:
            return False
            
        # Revisamos las distancias del láser para detectar obstáculos cercanos
        for distance in self.laser.ranges:
            # Si hay un obstáculo a menos de 40cm y la medida es válida
            if distance < 0.4 and distance > self.laser.range_min:
                return True
        return False

    def publish(self, lin_vel, ang_vel):
        move_cmd = TwistStamped()
        move_cmd.header.stamp = self.get_clock().now().to_msg()
        move_cmd.twist.linear.x = self.constrain_vel(lin_vel, -self.max_lin_vel, self.max_lin_vel)
        move_cmd.twist.angular.z = self.constrain_vel(ang_vel, -self.max_ang_vel, self.max_ang_vel)
        self.cmd_vel.publish(move_cmd)

    def constrain_vel(self, val, low, high):
        return max(low, min(val, high))

def main():
    rclpy.init()
    try:
        robot = TurtlebotControlCollisionCheck()
        robot.get_logger().info("Para parar: CTRL + C")
        rclpy.spin(robot)
    except KeyboardInterrupt:
        pass
    finally:
        rclpy.shutdown()

if __name__ == '__main__':
    main()