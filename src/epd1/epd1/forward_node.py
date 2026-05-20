#!/usr/bin/python3
# This Python file uses the following encoding: utf-8

# A very basic TurtleBot script that moves TurtleBot forward for 5 seconds. Press CTRL + C to stop.  
# To run on TurtleBot:
# ros2 run epd1 forward_node.py


import rclpy
from rclpy.node import Node
from geometry_msgs.msg import Twist, TwistStamped

class GoForward(Node):
    def __init__(self):
        super().__init__('forward_node')
        
        # tell user how to stop TurtleBot
        self.get_logger().info("To stop TurtleBot CTRL + C")
        
        # Create a publisher which can "talk" to TurtleBot and tell it to move
        self.cmd_vel = self.create_publisher(TwistStamped, '/cmd_vel', 10)
        
        # TurtleBot will stop if we don't keep telling it to move. How often should we tell it to move? 10 HZ
        timer_period = 0.1  # seconds
        self.times = 50 
        self.timer = self.create_timer(timer_period, self.timer_callback)
        
        # Twist is a datatype for velocity
        self.move_cmd = TwistStamped()
        # let's go forward at 0.2 m/s
        self.move_cmd.twist.linear.x = 0.2
        # let's turn at 0 radians/s
        self.move_cmd.twist.angular.z = 0.0

    def timer_callback(self):
        self.move_cmd.header.stamp = self.get_clock().now().to_msg()
        self.times -= 1
        # publish the velocity
        if self.times <= 0:
            self.move_cmd.twist.linear.x = 0.0
        self.get_logger().info('Publishing vel: "%s"' % self.move_cmd.twist.linear.x)
        self.cmd_vel.publish(self.move_cmd)

def main(args=None):
    rclpy.init(args=args)
    node = GoForward()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()


if __name__ == '__main__':
    main()