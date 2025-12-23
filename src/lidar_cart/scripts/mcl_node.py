#!/usr/bin/env python3
import rclpy
from rclpy.node import Node
from geometry_msgs.msg import PoseWithCovarianceStamped, PoseArray, Pose, Point, Quaternion, TransformStamped
from nav_msgs.msg import OccupancyGrid, Odometry
from sensor_msgs.msg import LaserScan
from tf2_ros import TransformBroadcaster
import tf_transformations

import numpy as np
import math
import time

class ParticleFilter:
    def __init__(self, num_particles=1000):
        self.num_particles = num_particles
        self.particles = [] # List of [x, y, theta, weight]
        self.weights = []
        self.initialized = False
        
    def initialize_particles_gaussian(self, x, y, theta, std_x, std_y, std_theta):
        self.particles = np.zeros((self.num_particles, 3))
        self.weights = np.ones(self.num_particles) / self.num_particles
        
        self.particles[:, 0] = np.random.normal(x, std_x, self.num_particles)
        self.particles[:, 1] = np.random.normal(y, std_y, self.num_particles)
        self.particles[:, 2] = np.random.normal(theta, std_theta, self.num_particles)
        
        self.initialized = True

    def initialize_particles_global(self, map_width, map_height, resolution, origin_x, origin_y):
        self.particles = np.zeros((self.num_particles, 3))
        self.weights = np.ones(self.num_particles) / self.num_particles
        
        min_x = origin_x
        max_x = origin_x + map_width * resolution
        min_y = origin_y
        max_y = origin_y + map_height * resolution
        
        self.particles[:, 0] = np.random.uniform(min_x, max_x, self.num_particles)
        self.particles[:, 1] = np.random.uniform(min_y, max_y, self.num_particles)
        self.particles[:, 2] = np.random.uniform(-math.pi, math.pi, self.num_particles)
        
        self.initialized = True

    def motion_update(self, delta_linear, delta_angular, dt):
        # Motion Model (Odometry based)
        # Assuming diff drive
        # Noise parameters
        alpha1 = 0.1 # rotational error from rotation
        alpha2 = 0.1 # rotational error from translation
        alpha3 = 0.1 # translational error from translation
        alpha4 = 0.1 # translational error from rotation
        
        # Sample noisy control
        # This is a simplified velocity motion model
        v = delta_linear
        w = delta_angular
        
        # Add noise to v and w
        # v_hat = v + sample(alpha3*v + alpha4*w)
        # w_hat = w + sample(alpha1*w + alpha2*v)
        
        noise_v = np.random.normal(0, alpha3 * abs(v) + alpha4 * abs(w), self.num_particles)
        noise_w = np.random.normal(0, alpha1 * abs(w) + alpha2 * abs(v), self.num_particles)
        
        v_hat = v + noise_v
        w_hat = w + noise_w
        
        # Update particles
        # x' = x - (v/w)*sin(theta) + (v/w)*sin(theta + w*dt)
        # y' = y + (v/w)*cos(theta) - (v/w)*cos(theta + w*dt)
        # theta' = theta + w*dt
        
        # Avoid division by zero
        epsilon = 1e-6
        mask = np.abs(w_hat) < epsilon
        
        # Straight motion (w ~ 0)
        self.particles[mask, 0] += v_hat[mask] * np.cos(self.particles[mask, 2]) * dt
        self.particles[mask, 1] += v_hat[mask] * np.sin(self.particles[mask, 2]) * dt
        self.particles[mask, 2] += w_hat[mask] * dt
        
        # Circular motion
        not_mask = ~mask
        ratio = v_hat[not_mask] / w_hat[not_mask]
        theta = self.particles[not_mask, 2]
        self.particles[not_mask, 0] += -ratio * np.sin(theta) + ratio * np.sin(theta + w_hat[not_mask] * dt)
        self.particles[not_mask, 1] += ratio * np.cos(theta) - ratio * np.cos(theta + w_hat[not_mask] * dt)
        self.particles[not_mask, 2] += w_hat[not_mask] * dt
        
        # Normalize angles
        self.particles[:, 2] = np.arctan2(np.sin(self.particles[:, 2]), np.cos(self.particles[:, 2]))



    def sensor_update(self, scan_ranges, scan_angle_min, scan_angle_increment, map_data, map_info):
        # Downsample scan for performance
        step = 5
        ranges = np.array(scan_ranges[::step])
        if len(ranges) == 0:
            return

        angles = np.arange(scan_angle_min, scan_angle_min + scan_angle_increment * len(scan_ranges), scan_angle_increment)[::step]
        
        # Valid ranges only
        # max_range check
        valid_mask = (ranges < 10.0) & (ranges > 0.1) # Assuming max range 10m
        ranges = ranges[valid_mask]
        angles = angles[valid_mask]
        
        if len(ranges) == 0:
            return

        # Scan points in robot frame
        xs_robot = ranges * np.cos(angles)
        ys_robot = ranges * np.sin(angles)
        
        # Vectorized transform
        # Particles: (N, 3) -> x, y, theta
        # Create transformation matrices is expensive? 
        # Just use direct formula:
        # x_map = x_p + x_r * cos(theta_p) - y_r * sin(theta_p)
        # y_map = y_p + x_r * sin(theta_p) + y_r * cos(theta_p)
        
        N = self.num_particles
        M = len(ranges)
        
        p_x = self.particles[:, 0].reshape(N, 1)
        p_y = self.particles[:, 1].reshape(N, 1)
        p_theta = self.particles[:, 2].reshape(N, 1)
        
        cos_theta = np.cos(p_theta)
        sin_theta = np.sin(p_theta)
        
        # Broadcast (N, 1) and (M,) -> (N, M)
        x_map = p_x + xs_robot * cos_theta - ys_robot * sin_theta
        y_map = p_y + xs_robot * sin_theta + ys_robot * cos_theta
        
        # Convert to grid coords
        res = map_info.resolution
        origin_x = map_info.origin.position.x
        origin_y = map_info.origin.position.y
        width = map_info.width
        height = map_info.height
        
        gx = ((x_map - origin_x) / res).astype(int)
        gy = ((y_map - origin_y) / res).astype(int)
        
        # Filter valid indices
        in_bounds_mask = (gx >= 0) & (gx < width) & (gy >= 0) & (gy < height)
        
        # Calculate score from map
        # Flatten map (row-major: y * width + x)? ROS OccupancyGrid data is row-major?
        # Yes, usually array[y * width + x]
        # But numpy array from list is heavy, so we keep map_data as list or bytes?
        # msg.data is array.array or list. Convert to numpy once.
        if not isinstance(map_data, np.ndarray):
            map_data = np.array(map_data, dtype=np.int8)
            
        # 0: Free, 100: Occupied, -1: Unknown
        # We want high probability for Occupied (100) or close to it.
        # But standard beam model expects "hit" at occupied cells.
        
        # Simple Model: Count hits in occupied cells
        # Indices for valid points
        # To vectorise:
        # map_values = np.zeros((N, M))
        # map_values[in_bounds_mask] = map_data[gy[in_bounds_mask] * width + gx[in_bounds_mask]]
        
        flat_indices = gy * width + gx
        
        # Initialize scores (probabilities)
        # Assign a small probability for out of bounds or free space
        # High probability for occupied
        
        # We need to handle flat_indices safely.
        # Use a safe lookup
        map_vals = np.zeros((N, M), dtype=np.int8)
        
        # Only look up valid indices
        valid_flat_indices = flat_indices[in_bounds_mask]
        map_vals[in_bounds_mask] = map_data[valid_flat_indices]
        
        # Scoring:
        # Occupied (100) -> High weight (e.g. 1.0)
        # Free (0) -> Low weight (e.g. 0.1)
        # Unknown (-1) -> Low weight (e.g. 0.1)
        
        # Gaussian decay is better but hard without distance transform.
        # Let's try simple binary-ish model first.
        
        weights = np.ones((N, M)) * 0.1 # Default low probability
        
        # If map value > 50 (Occupied), boost weight.
        occupied_mask = (map_vals > 50) & in_bounds_mask
        weights[occupied_mask] = 1.0
        
        # Ideally we also penalize if the ray passes through obstacles (Ray casting), but that is too heavy.
        # We only check endpoints.
        
        # Total weight for each particle = product of weights (or sum of log weights)
        # prod(w) -> sum(log(w))
        
        log_weights = np.sum(np.log(weights), axis=1)
        
        # Shift to avoid underflow/overflow before exp
        max_log_weight = np.max(log_weights)
        self.weights = np.exp(log_weights - max_log_weight)
        
        # Normalize
        sum_weights = np.sum(self.weights)
        if sum_weights > 0:
            self.weights /= sum_weights
        else:
            # Recover if lost?
            self.weights = np.ones(self.num_particles) / self.num_particles

    def resample(self):
        # Low Variance Sampling
        # Only resample if effective number of particles is low
        # N_eff = 1 / sum(w^2)
        
        sum_sq_weights = np.sum(self.weights**2)
        if sum_sq_weights == 0:
            return
            
        neff = 1.0 / sum_sq_weights
        
        if neff > self.num_particles / 2.0:
            return # Particles are good enough
            
        new_particles = np.zeros_like(self.particles)
        r = np.random.uniform(0, 1.0 / self.num_particles)
        c = self.weights[0]
        i = 0
        for m in range(self.num_particles):
            U = r + m * (1.0 / self.num_particles)
            while U > c:
                i = (i + 1) % self.num_particles
                c += self.weights[i]
            new_particles[m] = self.particles[i]
        
        self.particles = new_particles
        self.weights = np.ones(self.num_particles) / self.num_particles

    def get_estimated_pose(self):
        # Mean of particles
        x = np.mean(self.particles[:, 0])
        y = np.mean(self.particles[:, 1])
        # Circular mean for angle
        sin_sum = np.sum(np.sin(self.particles[:, 2]))
        cos_sum = np.sum(np.cos(self.particles[:, 2]))
        theta = np.arctan2(sin_sum, cos_sum)
        return x, y, theta

from rclpy.qos import QoSProfile, DurabilityPolicy

class MclNode(Node):
    def __init__(self):
        super().__init__('mcl_node')
        self.get_logger().info('Initializing MCL Node')
        
        try:
            self.declare_parameter('use_sim_time', False)
        except:
            pass
        
        self.pf = ParticleFilter(num_particles=500)
        
        map_qos = QoSProfile(depth=1, durability=DurabilityPolicy.TRANSIENT_LOCAL)
        self.create_subscription(OccupancyGrid, 'map', self.map_callback, map_qos)
        self.create_subscription(LaserScan, 'scan', self.scan_callback, 10)
        self.create_subscription(Odometry, 'odom', self.odom_callback, 10)
        self.create_subscription(PoseWithCovarianceStamped, 'initialpose', self.initial_pose_callback, 10)
        
        self.particle_pub = self.create_publisher(PoseArray, 'particle_cloud', 10)
        self.tf_broadcaster = TransformBroadcaster(self)
        
        # TF Buffer
        from tf2_ros.buffer import Buffer
        from tf2_ros.transform_listener import TransformListener
        
        self.tf_buffer = Buffer()
        self.tf_listener = TransformListener(self.tf_buffer, self)
        
        self.map_data = None
        self.map_info = None
        self.last_odom_time = None
        self.current_twist = None # Linear and Angular velocity from odom
        
        # Timer for broadcasting TF and publishing particles
        self.create_timer(0.05, self.timer_callback) # 20Hz

    def map_callback(self, msg):
        self.get_logger().info('Map received')
        self.map_data = np.array(msg.data, dtype=np.int8)
        self.map_info = msg.info
        # Initialize particles globally if not initialized
        if not self.pf.initialized:
            self.pf.initialize_particles_global(msg.info.width, msg.info.height, msg.info.resolution, msg.info.origin.position.x, msg.info.origin.position.y)

    def initial_pose_callback(self, msg):
        self.get_logger().info('Initial pose received')
        x = msg.pose.pose.position.x
        y = msg.pose.pose.position.y
        orientation_q = msg.pose.pose.orientation
        
        # Simple quaternion to euler
        siny_cosp = 2 * (orientation_q.w * orientation_q.z + orientation_q.x * orientation_q.y)
        cosy_cosp = 1 - 2 * (orientation_q.y * orientation_q.y + orientation_q.z * orientation_q.z)
        theta = math.atan2(siny_cosp, cosy_cosp)
        
        self.pf.initialize_particles_gaussian(x, y, theta, 0.5, 0.5, 0.1)

    def odom_callback(self, msg):
        current_time = self.get_clock().now()
        
        if self.last_odom_time is None:
            self.last_odom_time = current_time
            return

        dt = (current_time - self.last_odom_time).nanoseconds / 1e9
        self.last_odom_time = current_time
        
        # Use simple velocity motion model based on Twist from Odom
        linear_v = msg.twist.twist.linear.x
        angular_v = msg.twist.twist.angular.z
        
        if self.pf.initialized:
            self.pf.motion_update(linear_v, angular_v, dt)

    def scan_callback(self, msg):
        if not self.pf.initialized or self.map_data is None:
            return
            
        # Sensor update (weight update)
        self.pf.sensor_update(msg.ranges, msg.angle_min, msg.angle_increment, self.map_data, self.map_info)
        self.pf.resample()

    def timer_callback(self):
        if not self.pf.initialized:
            return
            
        # 1. Publish Particles
        pose_array = PoseArray()
        pose_array.header.stamp = self.get_clock().now().to_msg()
        pose_array.header.frame_id = 'map'
        
        for p in self.pf.particles[::5]: # Downsample particles for visu
            pose = Pose()
            pose.position.x = p[0]
            pose.position.y = p[1]
            
            # Euler to Quaternion (Z-rotation only)
            cy = math.cos(p[2] * 0.5)
            sy = math.sin(p[2] * 0.5)
            pose.orientation.w = cy
            pose.orientation.z = sy
            
            pose_array.poses.append(pose)
            
        self.particle_pub.publish(pose_array)
        
        # 2. Broadcast Map -> Odom TF
        x, y, theta = self.pf.get_estimated_pose()
        
        # Get Odom -> Base
        try:
            trans = self.tf_buffer.lookup_transform('odom', 'base_footprint', rclpy.time.Time())
            # T_map_base = T_map_odom * T_odom_base
            # T_map_odom = T_map_base * T_odom_base^-1
            
            # Use matrices
            # T_map_base
            t_map_base = tf_transformations.compose_matrix(
                translate=[x, y, 0], 
                angles=[0, 0, theta]
            )
            
            # T_odom_base
            q = trans.transform.rotation
            t_odom_base = tf_transformations.compose_matrix(
                translate=[trans.transform.translation.x, trans.transform.translation.y, trans.transform.translation.z],
                angles=tf_transformations.euler_from_quaternion([q.x, q.y, q.z, q.w])
            )
            
            # T_map_odom = T_mb * inv(T_ob)
            t_map_odom = np.dot(t_map_base, np.linalg.inv(t_odom_base))
            
            # Extract
            scale, shear, angles, trans_vec, persp = tf_transformations.decompose_matrix(t_map_odom)
            
            t = TransformStamped()
            t.header.stamp = self.get_clock().now().to_msg()
            t.header.frame_id = 'map'
            t.child_frame_id = 'odom'
            t.transform.translation.x = trans_vec[0]
            t.transform.translation.y = trans_vec[1]
            t.transform.translation.z = 0.0 # 2D
            
            q = tf_transformations.quaternion_from_euler(angles[0], angles[1], angles[2])
            t.transform.rotation.x = q[0]
            t.transform.rotation.y = q[1]
            t.transform.rotation.z = q[2]
            t.transform.rotation.w = q[3]
            
            self.tf_broadcaster.sendTransform(t)
            
        except Exception as e:
            # self.get_logger().warn(f'Could not get odom -> base_link transform: {e}')
            pass


def main(args=None):
    rclpy.init(args=args)
    node = MclNode()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()
