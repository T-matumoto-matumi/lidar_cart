#!/usr/bin/env python3
import rclpy
from rclpy.node import Node
from rclpy.duration import Duration
from geometry_msgs.msg import PoseStamped, Point
from nav_msgs.msg import OccupancyGrid, Path
import numpy as np
import heapq
import math

class GlobalPlanner(Node):
    def __init__(self):
        super().__init__('global_planner')
        self.get_logger().info('Initializing Global Planner (A*)')
        
        # Parameters
        self.declare_parameter('planning_timeout', 5.0) # seconds
        
        # Subscribers
        # QoS
        from rclpy.qos import QoSProfile, DurabilityPolicy
        map_qos = QoSProfile(depth=1, durability=DurabilityPolicy.TRANSIENT_LOCAL)
        
        # Subscribers
        self.create_subscription(OccupancyGrid, '/map', self.map_callback, map_qos)
        self.create_subscription(PoseStamped, '/goal_pose', self.goal_callback, 10)
        # Note: In a real nav stack, start pose often comes from /initialpose or TF.
        # For simplicity, we might listen to TF map->base_footprint when goal is received.
        
        # Publishers
        self.plan_pub = self.create_publisher(Path, '/plan', 10)
        
        # State
        self.map_data = None
        self.map_info = None
        
        # TF Buffer checks
        from tf2_ros.buffer import Buffer
        from tf2_ros.transform_listener import TransformListener
        self.tf_buffer = Buffer()
        self.tf_listener = TransformListener(self.tf_buffer, self)

    def map_callback(self, msg):
        self.get_logger().info('Map received')
        self.map_info = msg.info
        self.map_data = np.array(msg.data, dtype=np.int8).reshape(
            (msg.info.height, msg.info.width)
        )

    def goal_callback(self, msg):
        self.get_logger().info('New Goal Received')
        if self.map_data is None:
            self.get_logger().warn('Map not yet received')
            return
            
        # Get Current Robot Pose
        try:
            trans = self.tf_buffer.lookup_transform('map', 'base_footprint', rclpy.time.Time())
            start_pose = PoseStamped()
            start_pose.header = trans.header
            start_pose.pose.position.x = trans.transform.translation.x
            start_pose.pose.position.y = trans.transform.translation.y
            start_pose.pose.orientation = trans.transform.rotation
        except Exception as e:
            self.get_logger().warn(f'Could not get robot pose: {e}')
            return

        # Plan Path
        path = self.plan_path(start_pose, msg)
        if path:
            self.plan_pub.publish(path)
            self.get_logger().info('Path found and published')
        else:
            self.get_logger().warn('Failed to find path')

    def plan_path(self, start, goal):
        # 1. Convert world coords to grid coords
        res = self.map_info.resolution
        origin_x = self.map_info.origin.position.x
        origin_y = self.map_info.origin.position.y
        width = self.map_info.width
        height = self.map_info.height
        
        def world_to_grid(wx, wy):
            gx = int((wx - origin_x) / res)
            gy = int((wy - origin_y) / res)
            return gx, gy

        def grid_to_world(gx, gy):
            wx = gx * res + origin_x + res / 2.0
            wy = gy * res + origin_y + res / 2.0
            return wx, wy
            
        start_gx, start_gy = world_to_grid(start.pose.position.x, start.pose.position.y)
        goal_gx, goal_gy = world_to_grid(goal.pose.position.x, goal.pose.position.y)
        
        # Check bounds
        if not (0 <= start_gx < width and 0 <= start_gy < height):
            self.get_logger().warn('Start position out of bounds')
            return None
        if not (0 <= goal_gx < width and 0 <= goal_gy < height):
            self.get_logger().warn('Goal position out of bounds')
            return None
            
        # Check occupancy (Simple check)
        start_idx = start_gy * width + start_gx
        goal_idx = goal_gy * width + goal_gx
        
        if self.map_data[start_gy, start_gx] > 0:
             self.get_logger().warn('Start position is occupied')
             # return None # Allow starting from occupied (sometimes happens due to noise)
             
        if self.map_data[goal_gy, goal_gx] > 50:
             self.get_logger().warn('Goal position is occupied')
             return None

        # A* Algorithm
        open_set = []
        heapq.heappush(open_set, (0, (start_gx, start_gy)))
        
        came_from = {}
        g_score = { (start_gx, start_gy): 0 }
        f_score = { (start_gx, start_gy): self.heuristic(start_gx, start_gy, goal_gx, goal_gy) }
        
        directions = [(0, 1), (0, -1), (1, 0), (-1, 0), (1, 1), (1, -1), (-1, 1), (-1, -1)]
        
        start_time = self.get_clock().now()
        
        while open_set:
            if (self.get_clock().now() - start_time).nanoseconds / 1e9 > self.get_parameter('planning_timeout').value:
                 self.get_logger().warn('Planning timed out')
                 return None

            current = heapq.heappop(open_set)[1]
            
            if current == (goal_gx, goal_gy):
                return self.reconstruct_path(came_from, current, grid_to_world, start.header)
                
            for dx, dy in directions:
                neighbor = (current[0] + dx, current[1] + dy)
                
                # Check bounds
                if not (0 <= neighbor[0] < width and 0 <= neighbor[1] < height):
                    continue
                
                # Check occupancy (Threshold 50)
                # Diagonal movement cost is sqrt(2), straight is 1
                dist = math.sqrt(dx*dx + dy*dy)
                
                # Simple collision check
                if self.map_data[neighbor[1], neighbor[0]] > 50:
                    continue
                    
                tentative_g_score = g_score[current] + dist
                
                if neighbor not in g_score or tentative_g_score < g_score[neighbor]:
                    came_from[neighbor] = current
                    g_score[neighbor] = tentative_g_score
                    f_score[neighbor] = tentative_g_score + self.heuristic(neighbor[0], neighbor[1], goal_gx, goal_gy)
                    heapq.heappush(open_set, (f_score[neighbor], neighbor))
                    
        return None

    def heuristic(self, x1, y1, x2, y2):
        # Euclidean distance
        return math.sqrt((x1 - x2)**2 + (y1 - y2)**2)

    def reconstruct_path(self, came_from, current, grid_to_world, header):
        path = Path()
        path.header = header
        path.header.stamp = self.get_clock().now().to_msg()
        
        total_path = [current]
        while current in came_from:
            current = came_from[current]
            total_path.append(current)
            
        total_path.reverse()
        
        for gx, gy in total_path:
            wx, wy = grid_to_world(gx, gy)
            pose = PoseStamped()
            pose.header = path.header
            pose.pose.position.x = wx
            pose.pose.position.y = wy
            pose.pose.orientation.w = 1.0
            path.poses.append(pose)
            
        return path

def main(args=None):
    rclpy.init(args=args)
    node = GlobalPlanner()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()
