#!/usr/bin/env python3
# This Python file uses the following encoding: utf-8

import sys
import math
import rclpy
from rclpy.node import Node
from geometry_msgs.msg import Twist, PointStamped, PoseStamped, TwistStamped
from sensor_msgs.msg import LaserScan
from nav_msgs.msg import Path, Odometry
from tf2_ros import TransformException
from tf2_ros.buffer import Buffer
from tf2_ros.transform_listener import TransformListener
from robot_controller.robot_utils import Utils
import tf2_geometry_msgs 


class TurtlebotController(Node):

    def __init__(self):
        super().__init__('robot_controller')

        # Declare parameters
        self.declare_parameter('robot_vel_topic', 'cmd_vel')
        self.declare_parameter('robot_scan_topic', 'scan')
        self.declare_parameter('max_lin_vel', 0.25)
        self.declare_parameter('max_ang_vel', 1.2)
        self.declare_parameter('control_rate', 10)

        # Get parameters
        robot_vel_topic = self.get_parameter('robot_vel_topic').value
        robot_scan_topic = self.get_parameter('robot_scan_topic').value
        self.max_lin_vel = self.get_parameter('max_lin_vel').value
        self.max_ang_vel = self.get_parameter('max_ang_vel').value
        control_rate = self.get_parameter('control_rate').value

        # Store the received path here
        self.path = Path()
        self.path_received = False
        self.laser = LaserScan()
        self.laser_received = False

        # Declare the velocity command publisher
        self.cmd_vel = self.create_publisher(TwistStamped, robot_vel_topic, 10)

        # Create tf2 buffer and listener
        self.tf_buffer = Buffer()
        self.tf_listener = TransformListener(self.tf_buffer, self)

        self.utils = Utils(self.tf_buffer)

        # Subscription to the scan topic [sensor_msgs/LaserScan]
        self.create_subscription(LaserScan, robot_scan_topic, self.laser_callback, 10)

        # Subscription to the path topic [nav_msgs/Path]
        self.create_subscription(Path, 'path', self.path_callback, 10)

        # Subscription to odometry
        self.create_subscription(Odometry, 'odom', self.odom_callback, 10)

        # Create a timer for the control loop
        timer_period = 1.0 / control_rate  # seconds
        self.timer = self.create_timer(timer_period, self.control_loop)

    def odom_callback(self, msg):
        """Callback to receive and store odometry messages"""
        self.utils.set_odom(msg)

    def control_loop(self):
        """Main control loop executed at fixed rate"""
        end = self.command()
        if end:
            self.get_logger().info("Goal reached, stopping")
            # self.destroy_timer(self.timer) # he eliminado esto del codigo y he añadido lo siguiente
            self.path_received = False
            self.path = Path()

    def command(self):
        """
        Command the robot to follow the path
        Returns True if goal reached, False otherwise
        """
        # Check if the final goal has been reached
        # TODO: Exercise 1:implement goal reached check

        # Si no hay ruta, esperamos
        if not self.path_received:
            return False

        # Comprobar llegada: Solo si goal_reached devuelve True
        if self.goal_reached():
            self.get_logger().info("GOAL REACHED!!! Stopping!")
            self.publish(0.0, 0.0)
            return True

        # Determine the local path point to be reached
        # TODO: Exercise 1: fill the method get_sub_goal
        current_goal = self.get_sub_goal() 

        # TODO: use current_goal
        # Put your control law here (copy from EPD1)
        # Transformamos el punto que nos da get_sub_goal al sistema del robot
        goal_in_robot = self.utils.transform_pose(current_goal, 'base_footprint', self.get_logger())

        if goal_in_robot is not None:
            x = goal_in_robot.pose.position.x
            y = goal_in_robot.pose.position.y

            # Calculamos rho (distancia) y alpha (ángulo)
            rho = math.sqrt(x**2 + y**2)
            alpha = math.atan2(y, x)

            # Velocidades: Ley de control
            linear = 0.2 * rho
            angular = 0.4 * alpha
        else:
            linear = 0.0
            angular = 0.0

        # Check the maximum speed values allowed
        angular = self.constrain_vel(angular, -self.max_ang_vel, self.max_ang_vel)
        linear = self.constrain_vel(linear, 0.0, self.max_lin_vel)

        # If the computed commands does not provoke a collision,
        # send the commands to the robot
        # TODO: fill the check_collision function (copy from EPD2)
        if not self.check_collision(linear, angular):
            self.publish(linear, angular)
            return False

        # If a possible collision is detected,
        # try to find an alternative command to avoid the collision
        # TODO: Exercise 2: fill the collision_avoidance function
        linear, angular = self.collision_avoidance() 
        self.publish(linear, angular)
        return False

    def goal_reached(self):
        """
        Exercise 1.2: Check if the final goal has been reached
        TODO: use the last point of the path to check if the robot
        has reached the final goal (the robot is in a close position).
        Returns True if the FINAL goal was reached, False otherwise
        """

        if not self.path_received or len(self.path.poses) == 0:
            return False

        # Cogemos el ultimo punto de la ruta
        final_goal = self.path.poses[-1]

        # Transformamos al sistema de referencia del robot
        final_in_robot = self.utils.transform_pose(final_goal, 'base_footprint', self.get_logger())

        # Si la transformación falla, devuelve un PoseStamped vacío (x=0, y=0)
        # Comprobamos que el frame_id no esté vacío para saber si fue válida
        if final_in_robot.header.frame_id == '':
            return False

        # Si la transformación es buena, calculamos distancia
        x = final_in_robot.pose.position.x
        y = final_in_robot.pose.position.y
        dist_to_final = math.sqrt(x**2 + y**2)

        # Solo paramos si la distancia es real y pequeña
        if dist_to_final < 0.08:
            return True

        return False

    def get_sub_goal(self):
        """
        Exercise 1.1: Get the next sub-goal from the path
        TODO: use self.path.poses to find the subgoal to be reached
        You could transform the path points to the robot reference
        to find the closest point:
        path_pose = self.path.poses[index]
        path_pose_in_robot_frame = self.utils.transform_pose(
            path_pose, 'base_footprint', self.get_logger())
        """
        subgoal = PoseStamped()
        if not self.path_received or len(self.path.poses) == 0:
            return subgoal

        L = 0.25  # Distancia de lookahead (25 cm es ideal para pasillos estrechos)

        # 1. Encontrar el punto de la ruta más cercano al robot para saber dónde estamos
        closest_idx = 0
        min_dist = float("inf")

        for i, path_pose in enumerate(self.path.poses):
            path_pose_in_robot = self.utils.transform_pose(
                path_pose, "base_footprint", self.get_logger()
            )

            if path_pose_in_robot is not None:
                x = path_pose_in_robot.pose.position.x
                y = path_pose_in_robot.pose.position.y
                dist = math.sqrt(x**2 + y**2)
                if dist < min_dist:
                    min_dist = dist
                    closest_idx = i

        # 2. A partir de esa posición actual en la lista, buscar el primer punto hacia adelante a distancia L
        for i in range(closest_idx, len(self.path.poses)):
            path_pose = self.path.poses[i]
            path_pose_in_robot = self.utils.transform_pose(
                path_pose, "base_footprint", self.get_logger()
            )

            if path_pose_in_robot is not None:
                x = path_pose_in_robot.pose.position.x
                y = path_pose_in_robot.pose.position.y
                dist = math.sqrt(x**2 + y**2)
                if dist > L:
                    return path_pose

        # 3. Si estamos llegando al final y ningún punto supera L, el objetivo es el final
        return self.path.poses[-1]

    def check_collision(self, linear, angular):
        """
        Copy from EPD2: Check for possible collisions
        TODO: use self.laser to check possible collisions
        Optionally, you can also use the velocity commands
        Returns True if possible collision, False otherwise
        """
        if not self.laser_received:
            return False

        num_puntos = len(self.laser.ranges)
        limite_seguridad = 0.35

        for i in range(num_puntos):
            # Solo nos interesan los puntos del frente (aprox. entre 0-45 y 315-360)
            # i < 45 es la izquierda, i > (num_puntos - 45) es la derecha
            if i < 45 or i > (num_puntos - 45):
                distancia = self.laser.ranges[i]

                # Si detectamos algo cerca, devolvemos True (Peligro)
                if 0.05 < distancia < limite_seguridad:
                    return True
        return False

    def collision_avoidance(self):
        """
        Exercise 2: Try to find an alternative command to avoid collision
        TODO: try to find an alternative command to avoid the collision
        Here you must try to implement one of the reactive methods
        seen in T4: bug algorithm, potential fields, velocity obstacles,
        Dynamic Window Approach, others...
        Feel free to add the new variables and methods that you may need
        Returns (lin_vel, ang_vel)
        """
        lin_vel = 0.0 
        ang_vel = 0.0

        # Si todavía no hay datos, no nos movemos
        if not self.laser_received:
            return 0.0, 0.0

        # Numero de puntos que tiene el láser en total (normalmente 360)
        num_puntos = len(self.laser.ranges)

        # Miramos los primeros 45 puntos (izquierda) y los últimos 45 (derecha)
        # y calculamos la distancia media que está libre en cada lado
        dist_izq = sum(self.laser.ranges[0:45]) / 45
        dist_der = sum(self.laser.ranges[num_puntos-45: num_puntos]) / 45

        # Añadimos un pequeño margen (0.1)
        if dist_izq > (dist_der + 0.1):
            self.get_logger().info("Rodeando cono por la IZQUIERDA")
            ang_vel = 0.8 # Giro a la izquierda
        elif dist_der > (dist_izq + 0.1):
            self.get_logger().info("Rodeando cono por la DERECHA")
            ang_vel = -0.8 # Giro a la derecha
        else:
            # Si están muy igualados, elegimos un lado por defecto para no dudar
            ang_vel = 0.8

        return lin_vel, ang_vel

    def constrain_vel(self, input_vel, low_bound, high_bound):
        if input_vel < low_bound:
            input_vel = low_bound
        elif input_vel > high_bound:
            input_vel = high_bound
        else:
            input_vel = input_vel

        return input_vel

    def publish(self, lin_vel, ang_vel):
        """Publish velocity commands to the robot"""
        """move_cmd = Twist()
        move_cmd.linear.x = lin_vel
        move_cmd.angular.z = ang_vel
        self.cmd_vel.publish(move_cmd)"""
        """Publicar comandos de velocidad con sello de tiempo"""
        move_cmd = TwistStamped()
        move_cmd.header.stamp = self.get_clock().now().to_msg()
        move_cmd.header.frame_id = "base_link"  

        move_cmd.twist.linear.x = lin_vel
        move_cmd.twist.angular.z = ang_vel
        self.cmd_vel.publish(move_cmd)

    def laser_callback(self, data):
        """Callback to receive and store laser scan messages"""
        self.laser = data
        self.laser_received = True

    def path_callback(self, path):
        """Callback to receive and store path messages"""

        self.get_logger().info("¡RUTA RECIBIDA! He detectado %d puntos" % len(path.poses))
        self.path = path
        self.path_received = True

    def shutdown_callback(self):
        """Shutdown callback to stop the robot"""
        """self.get_logger().info("Stop TurtleBot")
        # A default Twist has linear.x of 0 and angular.z of 0, so it'll stop TurtleBot
        self.cmd_vel.publish(Twist())"""
        self.get_logger().info("Stop TurtleBot")
        stop_msg = TwistStamped()
        stop_msg.header.stamp = self.get_clock().now().to_msg()
        self.cmd_vel.publish(stop_msg)


def main(args=None):
    rclpy.init(args=args)
    
    # Create and run the controller
    robot_controller = TurtlebotController()
    
    robot_controller.get_logger().info("TurtleBot controller started")
    robot_controller.get_logger().info("To stop TurtleBot press CTRL+C")
    
    try:
        rclpy.spin(robot_controller)
    except KeyboardInterrupt:
        robot_controller.get_logger().info("TurtleBot controller stopped by user")
    finally:
        robot_controller.shutdown_callback()
        robot_controller.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()
