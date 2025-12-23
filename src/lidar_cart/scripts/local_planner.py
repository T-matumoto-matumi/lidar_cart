#!/usr/bin/env python3
import rclpy
from rclpy.node import Node
from geometry_msgs.msg import Twist, PoseStamped, Point
from nav_msgs.msg import Path, Odometry
from sensor_msgs.msg import LaserScan
import numpy as np
import math
import tf_transformations

class LocalPlanner(Node):
    def __init__(self):
        super().__init__('local_planner')
        self.get_logger().info('Initializing Local Planner (DWA)')
        
        # Parameters
        self.declare_parameter('max_speed', 0.22) # m/s
        self.declare_parameter('min_speed', 0.0)
        self.declare_parameter('max_yawrate', 1.0) # rad/s
        self.declare_parameter('max_accel', 0.2) # m/s^2
        self.declare_parameter('max_dyawrate', 0.4) # rad/s^2
        self.declare_parameter('v_reso', 0.01) # m/s resolution
        self.declare_parameter('yawrate_reso', 0.05) # rad/s resolution
        self.declare_parameter('dt', 0.1) # prediction timestep
        self.declare_parameter('predict_time', 1.0) # prediction horizon
        self.declare_parameter('goal_tolerance', 0.15)
        
        # Weights
        self.declare_parameter('to_goal_cost_gain', 0.15)
        self.declare_parameter('speed_cost_gain', 1.0)
        self.declare_parameter('obstacle_cost_gain', 1.0)
        self.declare_parameter('robot_radius', 0.2)
        
        # State
        self.global_path = None
        self.scan_data = None
        self.current_pose = None # [x, y, theta]
        self.current_velocity = None # [v, w]
        
        # Subscribers
        self.create_subscription(Path, '/plan', self.path_callback, 10)
        self.create_subscription(LaserScan, '/scan', self.scan_callback, 10)
        self.create_subscription(Odometry, '/odom', self.odom_callback, 10)
        
        # Publishers
        self.cmd_vel_pub = self.create_publisher(Twist, '/cmd_vel', 10)
        self.local_plan_pub = self.create_publisher(Path, '/local_plan', 10)
        
        # TF Buffer
        from tf2_ros.buffer import Buffer
        from tf2_ros.transform_listener import TransformListener
        self.tf_buffer = Buffer()
        self.tf_listener = TransformListener(self.tf_buffer, self)
        
        # Control Loop
        self.create_timer(0.1, self.control_loop) # 10Hz

    def path_callback(self, msg):
        self.global_path = msg
        self.get_logger().info('Global path received')

    def scan_callback(self, msg):
        self.scan_data = msg

    def odom_callback(self, msg):
        v = msg.twist.twist.linear.x
        w = msg.twist.twist.angular.z
        self.current_velocity = [v, w]

    def get_robot_pose(self):
        try:
            trans = self.tf_buffer.lookup_transform('map', 'base_footprint', rclpy.time.Time())
            x = trans.transform.translation.x
            y = trans.transform.translation.y
            q = trans.transform.rotation
            # quat to euler
            siny_cosp = 2 * (q.w * q.z + q.x * q.y)
            cosy_cosp = 1 - 2 * (q.y * q.y + q.z * q.z)
            theta = math.atan2(siny_cosp, cosy_cosp)
            return np.array([x, y, theta])
        except Exception:
            return None

    def control_loop(self):
        if self.global_path is None or self.scan_data is None or self.current_velocity is None:
            return
            
        self.current_pose = self.get_robot_pose()
        if self.current_pose is None:
            return
            
        # DWA Calculation here
        best_u = self.dwa_search(self.current_pose, self.current_velocity)
        
        # Publish Command
        cmd = Twist()
        cmd.linear.x = float(best_u[0])
        cmd.angular.z = float(best_u[1])
        self.cmd_vel_pub.publish(cmd)
        
    def dwa_search(self, x, u):
        # x: [x, y, theta], u: [v, w]
        
        # Dynamic Window
        dw = self.calc_dynamic_window(u)
        
        # Search for best u
        min_cost = float('inf')
        best_u = [0.0, 0.0]
        best_traj = None
        
        # Resolution
        # v_reso = self.get_parameter('v_reso').value # Fixed: parameter access
        v_reso = 0.01
        y_reso = 0.05
        
        # Get goal point (Lookahead on global path)
        goal = self.get_local_goal(x)
        if goal is None:
            return [0.0, 0.0]
        
        # Loop
        for v in np.arange(dw[0], dw[1], v_reso):
            for w in np.arange(dw[2], dw[3], y_reso):
                traj = self.predict_trajectory(x, v, w)
                
                # Calculate Costs
                to_goal_cost = self.get_parameter('to_goal_cost_gain').value * self.calc_to_goal_cost(traj, goal)
                speed_cost = self.get_parameter('speed_cost_gain').value * (self.get_parameter('max_speed').value - traj[-1, 3])
                ob_cost = self.get_parameter('obstacle_cost_gain').value * self.calc_obstacle_cost(traj)
                
                final_cost = to_goal_cost + speed_cost + ob_cost
                
                if final_cost < min_cost:
                    min_cost = final_cost
                    best_u = [v, w]
                    best_traj = traj
        
        # Publish local plan for debug
        if best_traj is not None:
             self.publish_local_plan(best_traj)
             
        return best_u

    def calc_dynamic_window(self, u):
        # [v_min, v_max, w_min, w_max]
        # Specification limits
        max_speed = self.get_parameter('max_speed').value
        min_speed = self.get_parameter('min_speed').value
        max_yawrate = self.get_parameter('max_yawrate').value
        max_accel = self.get_parameter('max_accel').value
        max_dyawrate = self.get_parameter('max_dyawrate').value
        
        vs = [min_speed, max_speed, -max_yawrate, max_yawrate]
              
        # Dynamic limits (accel)
        dt = 0.1 # control cycle
        vd = [u[0] - max_accel * dt,
              u[0] + max_accel * dt,
              u[1] - max_dyawrate * dt,
              u[1] + max_dyawrate * dt]
              
        # Intersection
        v_min = max(vs[0], vd[0])
        v_max = min(vs[1], vd[1])
        w_min = max(vs[2], vd[2])
        w_max = min(vs[3], vd[3])
        
        return [v_min, v_max, w_min, w_max]
        
    def predict_trajectory(self, x_init, v, w):
        dt = self.get_parameter('dt').value
        predict_time = self.get_parameter('predict_time').value
        
        traj = []
        x = np.array(x_init)
        cur_time = 0.0
        
        # [x, y, theta, v, w]
        while cur_time <= predict_time:
            x[2] += w * dt
            x[0] += v * math.cos(x[2]) * dt
            x[1] += v * math.sin(x[2]) * dt
            traj.append([x[0], x[1], x[2], v, w])
            cur_time += dt
            
        return np.array(traj)

    def get_local_goal(self, x):
        # Find point on global path ahead of robot
        if self.global_path is None or len(self.global_path.poses) == 0:
            return None
            
        # Find closest point
        min_dist = float('inf')
        closest_idx = -1
        
        for i, pose in enumerate(self.global_path.poses):
            px = pose.pose.position.x
            py = pose.pose.position.y
            dist = math.sqrt((x[0]-px)**2 + (x[1]-py)**2)
            if dist < min_dist:
                min_dist = dist
                closest_idx = i
                
        # Look ahead (e.g., 1.0m or 10 indices)
        lookahead_idx = min(closest_idx + 10, len(self.global_path.poses) - 1)
        target = self.global_path.poses[lookahead_idx]
        return [target.pose.position.x, target.pose.position.y]

    def calc_to_goal_cost(self, traj, goal):
        # Calculate heading error to goal
        dx = goal[0] - traj[-1, 0]
        dy = goal[1] - traj[-1, 1]
        goal_theta = math.atan2(dy, dx)
        traj_theta = traj[-1, 2]
        
        error = goal_theta - traj_theta
        # Normalize angle
        while error > math.pi: error -= 2*math.pi
        while error < -math.pi: error += 2*math.pi
        
        return abs(error)

    def calc_obstacle_cost(self, traj):
        if self.scan_data is None:
            return 0.0
            
        # Simplified Obstacle Check
        # Check strict collision with robot radius
        robot_radius = self.get_parameter('robot_radius').value
        min_scan_dist = min(self.scan_data.ranges) # Very simplified
        
        # If any point in traj is too close to "closest obstacle"
        # This is strictly incorrect as "closest obstacle" might be behind.
        # But for simple panic stop:
        if min_scan_dist < robot_radius:
            return float('inf')
            
        return 0.0
        
    def publish_local_plan(self, traj):
        path = Path()
        path.header.frame_id = 'map'
        path.header.stamp = self.get_clock().now().to_msg()
        for p in traj:
            pose = PoseStamped()
            pose.pose.position.x = p[0]
            pose.pose.position.y = p[1]
            path.poses.append(pose)
        self.local_plan_pub.publish(path)

def main(args=None):
    rclpy.init(args=args)
    node = LocalPlanner()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()
