#!/usr/bin/env python3
# This Python file uses the following encoding: utf-8

""" A ROS 2 planning node that subscribes to a costmap
  and generates a path by using the search algorithm """

import sys
import math
import rclpy
from rclpy.node import Node
from tf2_ros import Buffer, TransformListener
from tf2_ros import TransformException

from visualization_msgs.msg import Marker
from geometry_msgs.msg import Point, PoseStamped, PointStamped
from nav_msgs.msg import Path
from nav_msgs.msg import OccupancyGrid
from .myplanner import Planner

class PlannerNode(Node):
    def __init__(self):
        super().__init__('path_planner')

        # Declare parameters for frame ids and whether to update the map on each new costmap message
        self.declare_parameter('base_frame_id', 'base_footprint')
        self.declare_parameter('global_frame_id', 'map')
        self.declare_parameter('update_map', False)
        # self.declare_parameter('use_sim_time', True) # he añadido esto

        self.base_frame_id = self.get_parameter('base_frame_id').value
        self.global_frame_id = self.get_parameter('global_frame_id').value
        self.update_map = self.get_parameter('update_map').value

        self.tf_buffer = Buffer()
        self.tf_listener = TransformListener(self.tf_buffer, self)

        # To store the map
        self.map = None
        # To instantiate our planner
        self.planner = None
        # To store the goal on the map
        self.goal = None

        # QoS profile for map and path can be transient local
        from rclpy.qos import QoSProfile, DurabilityPolicy
        qos = QoSProfile(depth=10, durability=DurabilityPolicy.TRANSIENT_LOCAL)
        
        # Subscribe to the costmap topic and the goal topic
        # self.create_subscription(OccupancyGrid, 'costmap_2d/costmap/costmap', self.map_callback, qos)
        self.create_subscription(OccupancyGrid, 'costmap', self.map_callback, qos)

        self.create_subscription(PoseStamped, 'goal', self.goal_callback, 10) 

        # Publisher for the computed path
        self.path_publisher = self.create_publisher(Path, '/path', qos)

    # Callback for the costmap topic. It initializes the planner with the received map
    def map_callback(self, msg):
        self.map = msg
        if self.planner is None or self.update_map:
            self.get_logger().info("map received!")
            self.planner = Planner(self.map)

    # Callback for the goal topic. It computes the path from the robot position to the goal
    def goal_callback(self, msg):
        self.get_logger().info("Goal received! x: %.2f, y:%.2f" % (msg.pose.position.x, msg.pose.position.y))
        self.goal = [msg.pose.position.x, msg.pose.position.y]

        # First, get the robot position in the map
        base_pos = PointStamped()
        base_pos.header.frame_id = self.base_frame_id
        base_pos.header.stamp = self.get_clock().now().to_msg()
        initial_pos = None

        try:
            # wait for transform
            if self.tf_buffer.can_transform(self.global_frame_id, self.base_frame_id, rclpy.time.Time()):
                import tf2_geometry_msgs
                transform = self.tf_buffer.lookup_transform(self.global_frame_id, self.base_frame_id, rclpy.time.Time())
                initial_pos = tf2_geometry_msgs.do_transform_point(base_pos, transform)
            else:
                self.get_logger().warn("Transform not ready")
        except TransformException as ex:
            self.get_logger().error(f"Error getting robot position in map: {ex}")
            initial_pos = None

        if initial_pos is not None:
            ix = initial_pos.point.x
            iy = initial_pos.point.y
            gx = self.goal[0]
            gy = self.goal[1] 
            self.get_logger().info("Computing path...")
            self.compute_path(ix, iy, gx, gy)

    # This function computes the path from the initial position (ix, iy) to the goal position (gx, gy)
    def compute_path(self, ix, iy, gx, gy):
        if self.planner is not None:
            # Call the planner to compute the path. The planner should return a list of x and y positions of the path points
            path = self.planner.plan(ix, iy, gx, gy)

            if path is not None:
                xpath = path[0]
                ypath = path[1]

                path_msg = Path()
                path_msg.header.stamp = self.get_clock().now().to_msg()
                path_msg.header.frame_id = self.global_frame_id

                for i in range(len(xpath)):
                    pose = PoseStamped()
                    pose.header = path_msg.header
                    pose.pose.position.x = float(xpath[i])
                    pose.pose.position.y = float(ypath[i])
                    pose.pose.position.z = 0.0
                    path_msg.poses.append(pose)

                self.get_logger().info("Publishing computed path!")
                self.path_publisher.publish(path_msg)
            else:
                self.get_logger().error("Error computing path!")

def main(args=None):
    rclpy.init(args=args)
    node = PlannerNode()
    node.get_logger().info("Starting planning node. Waiting for valid map. Press CTRL+C to exit")
    
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        node.get_logger().info("Planning node terminated.")
    finally:
        node.destroy_node()
        rclpy.try_shutdown()

if __name__ == '__main__':
    main()
