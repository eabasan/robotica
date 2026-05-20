#!/usr/bin/python3
# This Python file uses the following encoding: utf-8

import sys
import math
import rclpy
from rclpy.node import Node
import tf2_ros
import tf2_geometry_msgs  # This registers geometry_msgs types with tf2
from geometry_msgs.msg import TwistStamped
from geometry_msgs.msg import PointStamped

BURGER_MAX_LIN_VEL = 0.50
BURGER_MAX_ANG_VEL = 2.84

LIN_VEL_STEP_SIZE = 0.01
ANG_VEL_STEP_SIZE = 0.1


class Turtlebot(Node):
    def __init__(self):
        super().__init__('robotcontrol')
        
        # Create a publisher which can "talk" to TurtleBot and tell it to move
        self.cmd_vel = self.create_publisher(TwistStamped, 'cmd_vel', 10)

        # Set up TF2 buffer and listener
        self.tf_buffer = tf2_ros.Buffer()
        self.tf_listener = tf2_ros.TransformListener(self.tf_buffer, self)
        
        self.goalx = 0.0
        self.goaly = 0.0

        # Timer for control loop (10 HZ)
        timer_period = 0.1  # seconds
        self.timer = self.create_timer(timer_period, self.timer_callback)
     
    def timer_callback(self):
        self.command(self.goalx, self.goaly)
        
    def command(self, gx, gy):
        
        goal = PointStamped()
        
        goal.header.frame_id = 'odom'
        goal.header.stamp = rclpy.time.Time()  # Use 0 to get latest available transform
        
        goal.point.x = gx
        goal.point.y = gy
        goal.point.z = 0.0
        
        try:
            base_goal = self.tf_buffer.transform(goal, 'base_footprint', timeout=rclpy.duration.Duration(seconds=1.0))
        except tf2_ros.TransformException as e:
            self.get_logger().warn(f"Transform failed: {e}")
            return
        #------------------------------------------------------------------
        # 1. Extraemos las coordenadas locales calculadas por TF
        # x e y son la posición del objetivo respecto al robot [cite: 171]
        x = base_goal.point.x
        y = base_goal.point.y

        # 2. Calculamos la distancia (rho) y el ángulo (alpha)
        # Usamos las fórmulas de la Figura 1 del manual [cite: 169]
        rho = math.sqrt(x**2 + y**2) # Distancia euclídea [cite: 198]
        alpha = math.atan2(y, x)     # Ángulo de giro necesario [cite: 198]

        # 3. Definimos las constantes del controlador proporcional [cite: 197]
        # Estos valores (ganancias) determinan la velocidad de respuesta
        kp_linear = 0.3
        kp_angular = 0.8

        # 4. Ley de control: la velocidad es proporcional al error [cite: 197]
        linear = kp_linear * rho
        angular = kp_angular * alpha

        # 5. Condición de parada: Si estamos a menos de 10cm, nos detenemos
        if rho < 0.1:
            linear = 0.0
            angular = 0.0
        #------------------------------------------------------------------
        # Finalmente, el nodo publica estas velocidades al robot [cite: 171]
        self.publish(linear, angular)

        #------------------------------------------------------------------
        # TODO: Put the control law here.
        # Compute linear and angular vels based on the distance to the goal
        # and the angle between the robot heading and the goal.
    

    def publish(self, lin_vel, ang_vel):
        # Twist is a datatype for velocity
        move_cmd = TwistStamped()
        move_cmd.header.stamp = self.get_clock().now().to_msg()

        # Copy the forward velocity
        move_cmd.twist.linear.x = self.constrain_vel(lin_vel, -BURGER_MAX_LIN_VEL, BURGER_MAX_LIN_VEL)
        # Copy the angular velocity
        move_cmd.twist.angular.z = self.constrain_vel(ang_vel, -BURGER_MAX_ANG_VEL, BURGER_MAX_ANG_VEL)

        self.get_logger().info('Publishing linear: "%.2f angular: "%.2f"' % (move_cmd.twist.linear.x, move_cmd.twist.angular.z))
        self.cmd_vel.publish(move_cmd)


    def constrain_vel(self,input_vel, low_bound, high_bound):
        if input_vel < low_bound:
            input_vel = low_bound
        elif input_vel > high_bound:
            input_vel = high_bound
        else:
            input_vel = input_vel

        return input_vel
        
    def shutdown(self):
        # stop turtlebot
        self.get_logger().info("Stop TurtleBot")
        # a default Twist has linear.x of 0 and angular.z of 0. So it'll stop TurtleBot
        self.cmd_vel.publish(Twist())
 
def main():
    rclpy.init()
    try:
        # tell user how to stop TurtleBot
        robot = Turtlebot()
        robot.get_logger().info("To stop TurtleBot CTRL + C")

        goalx = float(sys.argv[1])
        goaly = float(sys.argv[2])
        
        robot.get_logger().info("Goal set to x: %.2f y: %.2f" % (goalx, goaly))
        
        robot.goalx = goalx
        robot.goaly = goaly

        rclpy.spin(robot)

    except KeyboardInterrupt:
        robot.get_logger().info("robotcontrol node terminated.")
    except IndexError:
        print("Usage: ros2 run epd1 controlGoal.py <goalx> <goaly>")
    finally:
        robot.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()


if __name__ == '__main__':
    main()