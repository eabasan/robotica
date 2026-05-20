#!/usr/bin/python3
# This Python file uses the following encoding: utf-8

# This script is used for generating the statistics that will be used in
# the evaluation of the subject Robotics and Artificial Vision of
# 4th grade of the Informatics Degree at UPO


import sys
import math
import time
import rclpy
from rclpy.node import Node
from tf2_ros import Buffer, TransformListener
from tf2_ros import TransformException
import tf2_geometry_msgs
import numpy as np
from geometry_msgs.msg import TwistStamped, PoseStamped
from sensor_msgs.msg import LaserScan
from visualization_msgs.msg import Marker, MarkerArray
import yaml
import os

class Evaluation(Node):
    def __init__(self):
        super().__init__('evaluation_node')
        self.get_logger().info("Starting Evaluation node...")

        self.tf_buffer = Buffer()
        self.tf_listener = TransformListener(self.tf_buffer, self)
            
        self.declare_parameter('base_frame', 'base_footprint')
        self.declare_parameter('global_frame', 'map')
        self.declare_parameter('output_file', 'metrics.txt')
        self.declare_parameter('max_lin_vel', 0.25)
        self.declare_parameter('max_ang_vel', 1.2)
        self.declare_parameter('goal_tolerance', 0.15)
        self.declare_parameter('min_collision_dist', 0.12)
        self.declare_parameter('goals_config', 'navigation_goals.yaml')

        self.base_frame = self.get_parameter('base_frame').value
        self.global_frame = self.get_parameter('global_frame').value
        self.output_file = self.get_parameter('output_file').value
        self.max_lin_vel = self.get_parameter('max_lin_vel').value
        self.max_ang_vel = self.get_parameter('max_ang_vel').value
        self.goal_gap = self.get_parameter('goal_tolerance').value
        self.min_collision = self.get_parameter('min_collision_dist').value

        # Get the robot pose (base_frame) in map frame (global_frame)
        self.has_transform = False
        while not self.has_transform and rclpy.ok():
            rclpy.spin_once(self, timeout_sec=0.1)
            self.init_x, self.init_y = self.getTransform()
            self.px = self.init_x
            self.py = self.init_y
            if not self.has_transform:
                self.get_logger().info(
                    f"Waiting for TF frames are available: target='{self.global_frame}', source='{self.base_frame}'")
                time.sleep(0.5)
        self.get_logger().info(
            f"Evaluation: got robot position! init_x = {self.init_x} init_y={self.init_y}")
         
        self.metrics = self.Metrics(self.output_file)
        self.hist_dist_to_obs = []

        # read the goals from the YAML config file
        config_path = self.get_parameter('goals_config').value
        
        try:
            with open(config_path, 'r') as f:
                config_data = yaml.safe_load(f)
            
            # Extract goals from the config
            goals = config_data['robotics_challenge_evaluation']['goals']
            
            self.goals_queue = []
            t = self.get_clock().now().to_msg()
            for goal_name, goal_data in goals.items():
                self.get_logger().info(
                    f"Reading goal {goal_name} x: {goal_data['x']:.2f}, y: {goal_data['y']:.2f}")
                pose = PoseStamped()
                pose.header.frame_id = self.global_frame
                pose.header.stamp = t
                pose.pose.position.x = float(goal_data['x'])
                pose.pose.position.y = float(goal_data['y'])
                pose.pose.position.z = 0.0
                pose.pose.orientation.w = 1.0
                self.goals_queue.append(pose)
                
        except Exception as e:
            self.get_logger().error(f"Failed to load goals config from {config_path}: {e}")
            raise

        # generate visualization markers for goals
        markers = self.generate_visualization_goals()
        self.flag_markers = MarkerArray()
        
        # take the first goal
        self.current_goal = self.goals_queue.pop(0)
        self.goal_sent = False
        self.time_limit = 200.0 # 200 seconds = 3 min 20 seg
        self.count = 0

        self.get_logger().info("Creating publishers and subscribers....")
        self.goal_pub = self.create_publisher(PoseStamped, 'goal_pose', 10)
        self.marker_pub = self.create_publisher(MarkerArray, 'goal_markers', 10)
        self.flag_pub = self.create_publisher(MarkerArray, 'flag_markers', 10)
        self.scan_sub = self.create_subscription(LaserScan, 'scan', self.scan_callback, 1)
        self.cmd_vel_sub = self.create_subscription(TwistStamped, 'cmd_vel', self.cmd_vel_callback, 1)


        self.timer = self.create_timer(0.1, self.update) # 10 Hz control loop

        # wait for RViz to be subscribed to the markers before publishing
        start_time = time.time()
        while self.marker_pub.get_subscription_count() < 1 and rclpy.ok():
            self.get_logger().info("Waiting for subscribers to the marker topic...")
            rclpy.spin_once(self, timeout_sec=0.1)
            if time.time() - start_time > 10.0:
                self.get_logger().warn(
                    "Timeout waiting for marker subscribers; continuing anyway.")
                break
        self.get_logger().info("Publishing markers")
        self.marker_pub.publish(markers)
        self.init_time = self.get_clock().now()


    class Metrics:
        def __init__(self, output_file):
        
            self.goals_reached = 0
            self.distance_traveled = 0.0
            self.elapsed_time = 0.0
            self.min_dist_to_obs = 100.0
            self.avg_dist_to_obs = 0.0
            self.collision_penalty = 0
            self.lin_vel_penalty = 0
            self.ang_vel_penalty = 0
            self.out_file = output_file

        def store_metrics(self):
            rospy.loginfo("Stopped Evaluation node. Generating metrics result file")
            try:
                with open(self.out_file,'w') as f:
                    f.write('Goals reached: %d\n'%(self.goals_reached))
                    f.write('Elapsed time: %f\n'%(float(self.elapsed_time)))
                    f.write('Traveled Distance: %f\n'%(self.distance_traveled))
                    f.write('Min distance to obstacles: %f\n'%(self.min_dist_to_obs))
                    f.write('Avg distance to obstacles: %f\n'%(self.avg_dist_to_obs))
                    f.write('Collision penalties: %d\n'%(self.collision_penalty))
                    f.write('Linear vel penalties: %d\n'%(self.lin_vel_penalty))
                    f.write('Angular vel penalties: %d\n'%(self.ang_vel_penalty))
                self.get_logger().info(f'File exported successfully. Filename: {self.out_file}')
            except OSError as e:
                self.get_logger().error('Could not save output file. Error: %s', str(e))

              

    def scan_callback(self, data):
        #rospy.loginfo("scan received")
        m = min(data.ranges)
        self.hist_dist_to_obs.append(m)
        if(m < self.metrics.min_dist_to_obs):
            self.metrics.min_dist_to_obs = m
        if(m <= self.min_collision):
            rospy.logwarn('Collision penalty!!!')
            self.metrics.collision_penalty += 1
        

    def cmd_vel_callback(self, data):
        #rospy.loginfo("cmd_vel received")
        if np.abs(data.twist.linear.x) > self.max_lin_vel:
            self.get_logger().warn('Linear vel penalty!!!')
            self.metrics.lin_vel_penalty += 1
        if np.abs(data.twist.angular.z) > self.max_ang_vel:
            self.get_logger().warn('Angular vel penalty!!!')
            self.metrics.ang_vel_penalty += 1

    # This function gets the robot position in the map frame using the tf2 buffer and listener
    def getTransform(self):
        try:
            if not self.tf_buffer.can_transform(
                    self.global_frame,
                    self.base_frame,
                    rclpy.time.Time(),
                    timeout=rclpy.duration.Duration(seconds=1.0)):
                self.get_logger().debug(
                    f"TF not ready yet for {self.base_frame} -> {self.global_frame}")
                self.has_transform = False
                return (0, 0)

            t = self.tf_buffer.lookup_transform(
                        self.global_frame,
                        self.base_frame,
                        rclpy.time.Time())
        except TransformException as ex:
            self.get_logger().info(
                f'Could not transform {self.base_frame} to {self.global_frame}: {ex}')
            try:
                self.get_logger().info(
                    f"Available TF frames:\n{self.tf_buffer.all_frames_as_yaml()}")
            except Exception:
                self.get_logger().info("Unable to list TF frames")
            self.has_transform = False
            return (0, 0)

        self.has_transform = True
        return (t.transform.translation.x, t.transform.translation.y)

        


    def generate_visualization_goals(self):    
        ma = MarkerArray()
        t = self.get_clock().now().to_msg() #rclpy.time.Time()
        for i, p in enumerate(self.goals_queue):
            m = Marker()
            m.header.frame_id = p.header.frame_id
            m.header.stamp = t
            m.text = str(i)
            m.ns = "goals"
            m.id = i
            m.type = Marker.CYLINDER
            m.action = Marker.ADD
            m.lifetime = rclpy.duration.Duration(seconds=0).to_msg()
            m.pose.position.x = p.pose.position.x
            m.pose.position.y = p.pose.position.y
            m.pose.position.z = 0.01
            m.pose.orientation.w = 1.0
            m.scale.x = 0.16
            m.scale.y = 0.16
            m.scale.z = 0.01
            m.color.a = 1.0
            m.color.r = 0.0
            m.color.g = 1.0
            m.color.b = 0.0
            ma.markers.append(m)
        return ma


    def publish_flag(self, goal):
        t = self.get_clock().now().to_msg()
        m = Marker()
        i = len(self.flag_markers.markers)
        # update stamps
        for f in self.flag_markers.markers:
            f.header.stamp = t

        m.header.frame_id = "map"
        m.header.stamp = t
        #m.text = str(i)
        m.ns = "goal_flags"
        m.id = i
        m.type = Marker.CYLINDER
        m.action = Marker.ADD
        m.lifetime = rclpy.duration.Duration(seconds=0).to_msg()
        m.pose.position.x = goal.pose.position.x
        m.pose.position.y = goal.pose.position.y
        m.pose.position.z = 0.02
        m.pose.orientation.w = 1.0
        m.scale.x = 0.16
        m.scale.y = 0.16
        m.scale.z = 0.01
        m.color.a = 1.0
        m.color.r = 0.0
        m.color.g = 0.0
        m.color.b = 1.0
        self.flag_markers.markers.append(m)
        self.get_logger().info("Publishing goal reached flag!")
        self.flag_pub.publish(self.flag_markers)


    def update(self):
        self.count += 1
        gx = self.current_goal.pose.position.x
        gy = self.current_goal.pose.position.y
        # if no goal has been sent, send it!
        if self.goal_sent == False:
            
            #self.current_goal.header.stamp = rospy.Time.now()
            self.goal_pub.publish(self.current_goal)
            self.get_logger().info(f"\n\t\tNAVIGATION GOAL x:{gx:.2f}, y:{gy:.2f} SENT!\n")
            self.goal_sent = True
    
        # Get the current robot position in the map
        cx,cy = self.getTransform()

        # check the distance to the current goal
        dist_goal = math.sqrt( (cx-gx)**2 + (cy-gy)**2)
        
        # compute the distance that the robot has moved
        self.metrics.distance_traveled += math.sqrt( (cx-self.px)**2 + (cy-self.py)**2)
        # update the current position
        self.px = cx
        self.py = cy

        
        #check that we do not exceed a time deadline
        tdiff = self.get_clock().now() - self.init_time
        ts = float(tdiff.nanoseconds * 1e-9)

        if self.count > 10:
            self.get_logger().info(
                f"Metrics update: Traveled distance {self.metrics.distance_traveled:.2f} Time: {ts:.2f} Dist to goal: {dist_goal:.2f}")
            self.count = 0

        
        if(ts >= self.time_limit):
            self.get_logger().info(f"TIME ELAPSED EXCEEDED THE TIME LIMIT OF {self.time_limit:.1f} SECONDS!!!")
            self.metrics.elapsed_time = ts
            self.metrics.avg_dist_to_obs = round((sum(self.hist_dist_to_obs) / len(self.hist_dist_to_obs)), 2)
            sys.exit()

        # the goal has been reached?
        if dist_goal <= self.goal_gap:
            self.get_logger().info(f"Goal x: {gx:.2f}, y: {gy:.2f} reached!!! Elapsed time: {ts:.2f}\n")
            self.metrics.goals_reached += 1
            self.publish_flag(self.current_goal)
            # update goal if possible
            if len(self.goals_queue) > 0:
                self.current_goal = self.goals_queue.pop(0)
                self.current_goal.header.seq += 1
                self.goal_sent = False
            else:
                #All the goals has been reached, finish the program
                tdiff = self.get_clock().now() - self.init_time
                ts = float(tdiff.nanoseconds * 1e-9)
                self.metrics.elapsed_time = ts
                self.metrics.avg_dist_to_obs = round((sum(self.hist_dist_to_obs) / len(self.hist_dist_to_obs)), 2)
                self.get_logger().info("ALL THE GOALS REACHED!!! Storing metrics...")
                sys.exit()


    def shutdown(self):
        self.metrics.store_metrics()
 


def main(args=None):
    rclpy.init(args=args)
    eval = Evaluation()
    eval.get_logger().info("To stop the Evaluation node press CTRL + C")

    try:
        rclpy.spin(eval)
    except KeyboardInterrupt:
        eval.get_logger().info("Evaluation node terminated by user. Storing metrics...")
    finally:
        eval.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()