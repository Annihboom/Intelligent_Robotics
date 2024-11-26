#!/usr/bin/env python3
import rclpy
from rclpy.node import Node
from nav_msgs.msg import OccupancyGrid
import numpy as np
from math import pi
import math
from rclpy.qos import QoSProfile, QoSReliabilityPolicy, QoSDurabilityPolicy, HistoryPolicy
#prevent package unimported issue
import rclpy
from rclpy.node import Node
from rclpy.time import Time
from rclpy.duration import Duration
from geometry_msgs.msg import Twist, PoseStamped, Point
from nav_msgs.msg import OccupancyGrid, Odometry
from sensor_msgs.msg import LaserScan
from visualization_msgs.msg import Marker, MarkerArray
from std_msgs.msg import Header, ColorRGBA
from threading import Lock
from math import sqrt, cos, sin, pi, atan2,exp
import numpy as np
import random
from tf2_ros import TransformBroadcaster, TransformListener, Buffer
from geometry_msgs.msg import TransformStamped
import transforms3d
import transforms3d.quaternions as tquat
import transforms3d.euler as teuler



class Particle(object):
    def __init__(self, id, x, y, theta):
        self.x = x
        self.y = y
        self.id = id
        self.theta = theta

class MCLNode(Node):
    
    def __init__(self):
        super().__init__('mcl_node')


        self.laser_sub = self.create_subscription(LaserScan, '/bcr_bot/scan', self.laser_scan_callback, 1)
        self.odom_sub = self.create_subscription(Odometry, '/bcr_bot/odom', self.odometry_callback, 1)

        # Publishers
        self.pose_pub = self.create_publisher(PoseStamped, '/bcr_bot/pose_estimation', 1)
        self.particle_cloud_pub = self.create_publisher(MarkerArray, '/bcr_bot/particle_cloud', 1)
        self.laser_points_marker_pub = self.create_publisher(Marker, '/my_mcl_pkg/debug/laser_points', 1)
        self.mutex = Lock()
    
        self.last_scan = None
        self.timer = self.create_timer(1/3.5, self.publish_particle_markers)  # 每秒发布一次
        self.get_logger().info("")


        # Store transforms  for map->odom trnasform
        self.p_map_odom1 = None
        self.q_map_odom1 = None
        self.received_odometry_to_map = False
        self.tf_update_timer = self.create_timer(1/29, self.update_map_to_odom_transform)


        self.get_logger().info("MCLNode initialized")

        #Parameter: 
        self.alpha1 = 0.05
        self.alpha2 = 0.05
        self.alpha3 = 0.05
        self.alpha4 = 0.05
        self.alpha5 = 0.05
        self.num_particles= 150
        self.xmin=-16.0
        self.xmax=7.00
        self.ymin=-10.5
        self.ymax=10.45
        self.get_logger().info("before laser")
        self.laser_min_range=0.50
        self.laser_max_range=16
        self.laser_min_angle=-3.14
        self.laser_max_angle=3.14
        self.get_logger().info("before ogm")
        self.dynamics_translation_noise_std_dev=0.1
        self.dynamics_orientation_noise_std_dev=0.05
        self.beam_range_measurement_noise_std_dev=0.1
        self.ogm = None
        self.grid_map = None
        self.grid_bin = None
        # Number of laser beams 
        self.eval_beams =  360
        # Previous odometry measurement of the robot
        self.last_robot_odom = None
        # Current odometry measurement of the robot
        self.robot_odom = None
        # Relative motion since the last time particles were updated
        self.dx = 0
        self.dy = 0
        self.dyaw = 0
        self.particles = []
        self.weights = []

        self.map_initialized = False  # map initialzed?
        self.map_lock = Lock()  # protect based on multiple thread
        self.resolution = 0.05  # based on resolutions
        self.ogm = None

        # TF2 components
        self.tf_buffer = Buffer()
        self.tf_listener = TransformListener(self.tf_buffer, self)
        self.tf_broadcaster = TransformBroadcaster(self)



        # Subscriptions
        #   Subscribe to the /map topic
         # Subscribe to the /map topic
        self.map_subscription = self.create_subscription(
            OccupancyGrid,
            '/map',
            self.map_callback,
            QoSProfile(
                reliability=QoSReliabilityPolicy.RELIABLE,       # 确保消息传输可靠性
                durability=QoSDurabilityPolicy.TRANSIENT_LOCAL,  # 确保订阅者接收历史消息
                history=HistoryPolicy.KEEP_LAST,                 # 保留最近的若干条消息
                depth=1                                          # 指定深度为1（只保留最新消息）
            )
        )
         # Subscribe to the /map topic


    def get_random_free_state(self):
        while True:
            
            # Note: we initialize particles closer to the robot's initial
            # position in order to make the initialization easier
            self.get_logger().info("inside get random free state")
            xrand = np.random.uniform(self.xmin*0.04, self.xmax*0.04)
            yrand = np.random.uniform(self.ymin*0.04, self.ymax*0.04)
            row, col = self.metric_to_grid_coords(xrand, yrand)

            # debugging
            self.get_logger().info(f"Generated random state: x={xrand:.3f}, y={yrand:.3f}")
            self.get_logger().info(f"Converted to grid: row={row}, col={col}")
            
           
            if self.grid_bin[row, col]:
                theta = np.random.uniform(0, 2*pi)
                return xrand, yrand, theta

    def init_particles(self):
        """Initializes particles uniformly randomly with map frame coordinates,
        within the boundaries set by xmin,xmax, ymin,ymax"""
    
        for i in range(self.num_particles):
            xrand, yrand, theta = self.get_random_free_state()
            # Note: same orientation as the initial orientation of the robot
            # to make initialization easier
            self.particles.append(Particle(i, xrand, yrand, 0))
        self.get_logger().info(f"Initialized particle ")
    
    def handle_observation(self, laser_scan, dt):
        """
        Perform particle prediction, weight update, and resampling based on the latest observation.
        """
        errors = []
        # self.get_logger().info("inside handle_observation")
        # Step 1: Predict relative motion and compute observation error for each particle

        for particle in self.particles:
            # Predict the particle's new position based on odometry
            self.predict_particle_odometry(particle)

            # Compute the squared error between the predicted and actual laser scan
            error = self.get_prediction_error_squared(laser_scan, particle)
            errors.append(error)

        # self.get_logger().info(f"Errors: {errors}")

        # Step 2: Update weights using exponentiated negative error
        self.weights = [exp(-error) for error in errors]
        # self.get_logger().info(f"Weights (before normalization): {self.weights}")
        weight_sum = sum(self.weights)

        if weight_sum == 0:
            # Avoid division by zero in case all weights are zero
            self.weights = [1.0 / len(self.particles)] * len(self.particles)
        else:
            # Normalize weights
            self.weights = [weight / weight_sum for weight in self.weights]

        # Step 3: Calculate the effective sample size (N_eff) to evaluate particle deprivation
        # self.get_logger().info(f"func handle observations:self.weights:",{self.weights},"weight sum:",{weight_sum})
        N_eff = 1.0 / sum(w ** 2 for w in self.weights)

        # Step 4: Resample particles if necessary
        if N_eff < 70:  # Resample only if N_eff falls below a threshold
            self.resample()

    def divide_up(self, id, particle, num, particle_list):
        """
        Divide up a particle into smaller particles by adding random noise.
        """
        for i in range(int(num)):  
            xrand = np.random.uniform(particle.x * -0.5, particle.x * 0.5)
            yrand = np.random.uniform(particle.y * -0.5, particle.y * 0.5)
            theta = np.random.uniform(particle.theta * -0.5, particle.theta * 0.5)
            particle_list.append(Particle(id, xrand, yrand, theta))
            id += 1

    def resample(self):
        """
        resample based on weights
        """
        # noramlize weight
        total_weight = sum(self.weights)
        if total_weight == 0:
            self.get_logger().warn("All particle weights are zero. Reinitializing particles.")
            self.init_particles(self.robot_odom.pose.pose.position.x, self.robot_odom.pose.pose.position.y, radius=1.0)
            return

        self.weights = [w / total_weight for w in self.weights]

        new_particles = []
        index = np.random.randint(0, self.num_particles)  
        beta = 0.0
        max_weight = max(self.weights)

        for _ in range(self.num_particles):
            beta += np.random.uniform(0, 2 * max_weight)
            while beta > self.weights[index]:
                beta -= self.weights[index]
                index = (index + 1) % self.num_particles

            selected_particle = self.particles[index]

            # add random noise
            x_noise = np.random.normal(0, 0.1)  
            y_noise = np.random.normal(0, 0.1)  
            theta_noise = np.random.normal(0, 0.05)  

            new_particles.append(Particle(
                selected_particle.id,
                selected_particle.x + x_noise,
                selected_particle.y + y_noise,
                (selected_particle.theta + theta_noise) % (2 * pi)
            ))

        self.particles = new_particles

    

 
    def simulate_laser_scan_for_particle(self, x, y, yaw_in_map, angles, min_range, max_range):
        """
        Simulate laser scan ranges from a particle's state.
        """
        ranges = []
        range_step = self.resolution  # Step size for ray tracing

        for angle in angles:
            # Precompute angle and boundaries
            phi = yaw_in_map + angle
            cos_phi = cos(phi)
            sin_phi = sin(phi)
            
            # Simulate the ray trace
            r = min_range
            while r <= max_range:
                xm = x + r * cos_phi
                ym = y + r * sin_phi

                # Check if the point is outside the bounds of the workspace
                if xm < self.xmin or xm > self.xmax or ym < self.ymin or ym > self.ymax:
                    break

                # Convert metric coordinates to grid indices
                row, col = self.metric_to_grid_coords(xm, ym)

                
                # Check if the grid cell is occupied
                free = self.grid_bin[row, col].all()
                if not free:
                    break

                # Increment the range
                r += range_step

            # Append the final range to the result
            ranges.append(r)

        return ranges

    def subsample_laser_scan(self, laser_scan_msg):
        """
        Subsamples a set number of beams (self.eval_beams) from the incoming actual laser scan.
        It also converts the Inf range measurements into max_range range measurements,
        in order to compute differences.
        """

        # Number of beams in the laser scan
        N = len(laser_scan_msg.ranges)

        # Extract ranges and angles
        ranges_in_baselaser_frame = list(laser_scan_msg.ranges)
        # angles_in_baselaser_frame = [
        #     laser_scan_msg.angle_min + i * (laser_scan_msg.angle_max - laser_scan_msg.angle_min) / (N - 1)
        #     for i in range(N)
        # ]
        angles_in_baselaser_frame = np.linspace(
            laser_scan_msg.angle_min,
            laser_scan_msg.angle_max,
            N
        )

        # Subsample the beams
        step = max(1, N // self.eval_beams)
        ranges_in_subsampled_frame = ranges_in_baselaser_frame[::step]
        angles_in_subsampled_frame = angles_in_baselaser_frame[::step]

        assert len(ranges_in_subsampled_frame) == len(angles_in_subsampled_frame), \
            "Subsampled ranges and angles must have the same length."

        # Process ranges: clip to [laser_min_range, laser_max_range]
        actual_ranges = []
        for r in ranges_in_subsampled_frame:
            if r < self.laser_min_range:
                actual_ranges.append(self.laser_min_range)
            elif r > self.laser_max_range or not np.isfinite(r):
                actual_ranges.append(self.laser_max_range)
            else:
                actual_ranges.append(r)

        # Return the processed ranges and corresponding angles
        return actual_ranges, angles_in_subsampled_frame

    def get_prediction_error_squared(self, laser_scan_msg, particle):
        """
        Compute the squared norm of the difference between actual and predicted laser scans.
        """
        # Check if the particle is within bounds
        if particle.x < self.xmin or particle.x > self.xmax:
            return 300

        if particle.y < self.ymin or particle.y > self.ymax:
            return 300

        # Check if the particle is inside an obstacle
        row, col = self.metric_to_grid_coords(particle.x, particle.y)
        if row >= 201 or col >= 201:
            return 300

        if not self.grid_bin[row, col]:
            return 300

        assert self.laser_min_range >= 0
        assert self.laser_max_range > 0

        # Subsample the actual laser scan
        actual_ranges, angles = self.subsample_laser_scan(laser_scan_msg)

        # Simulate a laser scan for the given particle
        predict_ranges = self.simulate_laser_scan_for_particle(
            particle.x, particle.y, particle.theta, angles, self.laser_min_range, self.laser_max_range
        )

        # Compute the difference between predicted and actual ranges
        diff = [actual_range - predict_range for actual_range, predict_range in zip(actual_ranges, predict_ranges)]

        # Compute the squared norm of the difference
        # self.get_logger().info(f"Norm error: {norm_error}")
        norm_error = np.linalg.norm(diff)
        #self.get_logger().info(f"Norm error: {norm_error}")
        return norm_error**2

    # def predict_particle_odometry(self, particle):
    #     """
    #     predict particles state considering noise
    #     """
    #     # generate gaussian noise
    #     translation_noise = self.dynamics_translation_noise_std_dev * sqrt(self.dx**2 + self.dy**2)
    #     nx = random.gauss(0, translation_noise)
    #     ny = random.gauss(0, translation_noise)
    #     ntheta = random.gauss(0, self.dynamics_orientation_noise_std_dev * abs(self.dyaw))

    #     # compute vector speed
    #     v = sqrt(self.dx**2 + self.dy**2)

    #     # if v is low  ignore it to prevent noise dominating
    #     if abs(v) < 1e-10 and abs(self.dyaw) < 1e-5:
    #         return 

    #     # particle state with noise
    #     particle.x += (v * cos(particle.theta) + nx)
    #     particle.y += (v * sin(particle.theta) + ny)
    #     particle.theta += self.dyaw + ntheta

    #     # normalize theta into  [-π, π]
    #     particle.theta = (particle.theta + np.pi) % (2 * np.pi) - np.pi


    def predict_particle_odometry(self, particle):
        """
        Predict the particle's state using a motion model that supports forward and backward movement,
        with reduced noise for more stable prediction.
        """
        # Calculate relative motion (robot's translation and rotation)
        delta_rot1 = 0.0
        if abs(self.dx) > 1e-5 or abs(self.dy) > 1e-5:
            delta_rot1 = math.atan2(self.dy, self.dx) - particle.theta

        delta_trans = math.sqrt(self.dx**2 + self.dy**2)
        delta_rot2 = self.dyaw - delta_rot1

        # Add noise to motion model
        delta_rot1_hat = delta_rot1 + random.gauss(0, self.alpha1 *0.05 *abs(delta_rot1) + self.alpha2 *0.05* delta_trans)
        delta_trans_hat = delta_trans + random.gauss(0, self.alpha3 *0.05* delta_trans + self.alpha4 *0.05* (abs(delta_rot1) + abs(delta_rot2)))
        delta_rot2_hat = delta_rot2 + random.gauss(0, self.alpha1 *0.05* abs(delta_rot2) + 0.05*self.alpha2 * delta_trans)

        # Update particle state
        particle.x += delta_trans_hat * math.cos(particle.theta + delta_rot1_hat)
        particle.y += delta_trans_hat * math.sin(particle.theta + delta_rot1_hat)
        particle.theta += delta_rot1_hat + delta_rot2_hat

        # Normalize particle orientation to [-π, π]
        particle.theta = (particle.theta + math.pi) % (2 * math.pi) - math.pi





    def handle_odometry(self, robot_odom):
        """
        Compute the relative motion of the robot from the previous odometry measurement
        to the current odometry measurement using math library.
        """
        self.last_robot_odom = self.robot_odom
        self.robot_odom = robot_odom

        if self.last_robot_odom is not None:
            # Extract current and previous positions
            p_map_currbaselink = np.array([
                self.robot_odom.pose.pose.position.x,
                self.robot_odom.pose.pose.position.y,
                self.robot_odom.pose.pose.position.z
            ])

            p_map_lastbaselink = np.array([
                self.last_robot_odom.pose.pose.position.x,
                self.last_robot_odom.pose.pose.position.y,
                self.last_robot_odom.pose.pose.position.z
            ])

            # Extract quaternions
            q_map_lastbaselink = [
                self.last_robot_odom.pose.pose.orientation.x,
                self.last_robot_odom.pose.pose.orientation.y,
                self.last_robot_odom.pose.pose.orientation.z,
                self.last_robot_odom.pose.pose.orientation.w
            ]

            q_map_currbaselink = [
                self.robot_odom.pose.pose.orientation.x,
                self.robot_odom.pose.pose.orientation.y,
                self.robot_odom.pose.pose.orientation.z,
                self.robot_odom.pose.pose.orientation.w
            ]

            # Compute yaw for current and last orientations
            yaw_last = self.quaternion_to_yaw(q_map_lastbaselink)
            yaw_curr = self.quaternion_to_yaw(q_map_currbaselink)

            # Calculate relative translation
            delta_translation = p_map_currbaselink - p_map_lastbaselink

            # Compute the relative yaw difference, ensuring normalization to [-pi, pi]
            yaw_diff = (yaw_curr - yaw_last + math.pi) % (2 * math.pi) - math.pi
            # Update motion values
            self.dyaw += yaw_diff
            self.dx += delta_translation[0]
            self.dy += delta_translation[1]

    def metric_to_grid_coords(self, x, y):
        """Converts metric coordinates to occupancy grid coordinates"""
        #self.get_logger().info("self.ogm.info.resolution:",{self.ogm.info.resoultion})
        gx = (x - self.ogm.info.origin.position.x) / self.ogm.info.resolution
        gy = (y - self.ogm.info.origin.position.y) / self.ogm.info.resolution
        row = min(max(int(gy), 0), self.ogm.info.height)
        col = min(max(int(gx), 0), self.ogm.info.width)
        # Debugging logs
        # self.get_logger().info(f"Metric to grid conversion: x={x:.3f}, y={y:.3f} -> gx={gx:.3f}, gy={gy:.3f}")
        # self.get_logger().info(f"Grid coordinates: row={row}, col={col}")
        return (row, col)

    def map_callback(self, msg):
        with self.map_lock:
            if not self.map_initialized:
            
            # Initialize the occupancy grid map
                self.ogm = msg
                self.grid_map = np.array(self.ogm.data, dtype='int8').reshape((self.ogm.info.height, self.ogm.info.width))
                self.grid_bin = (self.grid_map == 0).astype('uint8') 
                self.init_particles()
                # Log map information
                # Print entire grid map data (may be large)
                self.get_logger().info("Complete grid map data:")
                self.get_logger().info(f"Number of particles: {len(self.particles)}")
            # Cell is True iff probability of being occupied is zero
                self.map_initialized = True
            else:
        
                self.ogm = msg 
                self.grid_map = np.array(self.ogm.data, dtype='int8').reshape(
                            (self.ogm.info.height, self.ogm.info.width)
                        )
                self.grid_bin = (self.grid_map == 0).astype('uint8') 
                self.get_logger().info("map data updated.")

        self.get_logger().info("Map initialized,outside map lock")
    
    
    def quaternion_to_yaw(self, quaternion):
        """
        Converts a quaternion to a yaw angle (in radians) using math library.
        """
        x, y, z, w = quaternion

        # Compute yaw from quaternion
        siny_cosp = 2 * (w * z + x * y)
        cosy_cosp = 1 - 2 * (y**2 + z**2)
        yaw = math.atan2(siny_cosp, cosy_cosp)
        
        return yaw


    def odometry_callback(self, msg):
      
      if not self.map_initialized:
            self.get_logger().warn("Map not initialized yet. Skipping odometry callback.")
            return

      if not msg:
            self.get_logger().warn('Not Received Odom empty message.')
            self.get_logger().warn('NOT Received Odom empty message.')
     

      with self.mutex:
        self.handle_odometry(msg)

    
    def laser_scan_callback(self, msg):
        """
        Handle laser scan messages, update the particle filter with the observation,
        and publish the corresponding laser points.
        """ 
        if not self.map_initialized:
            self.get_logger().warn("Map not initialized yet. Skipping laser scan callback.")
            return

        if not msg:
            self.get_logger().warn('Received empty message.')
            return
        # Update particle filter with laser parameters

        # self.get_logger().info(f"Laser message received")    
        self.laser_min_angle = msg.angle_min
        self.laser_max_angle = msg.angle_max
        self.laser_min_range = msg.range_min
        self.laser_max_range = msg.range_max

        # Calculate time difference since last scan
        dt_since_last_scan = 0
        if self.last_scan:
            dt_since_last_scan = (msg.header.stamp.sec + msg.header.stamp.nanosec * 1e-9) - \
                                (self.last_scan.header.stamp.sec + self.last_scan.header.stamp.nanosec * 1e-9)

        # Lock mutex to ensure thread-safe operations

        # Publish laser points for visualization
        self.publish_laser_pts(msg)

        # Update particle filter with new observation
        self.handle_observation(msg, dt_since_last_scan)
        with self.mutex:
        # Reset odometry deltas after processing
            self.dx = 0
            self.dy = 0
            self.dyaw = 0

        # Update the last scan message
        self.last_scan = msg
        # self.get_logger().info(f"Laser scan successful! message updated: ")
        
    def update_map_to_odom_transform(self):
        try:
            transform = self.tf_buffer.lookup_transform('map', 'odom', rclpy.time.Time())
            self.p_map_odom1 = [
                transform.transform.translation.x,
                transform.transform.translation.y,
                transform.transform.translation.z
            ]
            self.q_map_odom1 = [
                transform.transform.rotation.x,
                transform.transform.rotation.y,
                transform.transform.rotation.z,
                transform.transform.rotation.w
            ]
            self.received_odometry_to_map = True
            # self.get_logger().info("transform updated")
        except Exception as e:
            self.get_logger().warn(f"Failed to update map -> odom transform: {e}")
            self.received_odometry_to_map = False


    def publish_laser_pts(self, msg):
        """
        Publishes the currently received laser scan points from the robot,
        after subsampling them to compare with the expected laser scan from each particle.
        """
        if self.robot_odom is None or not self.tf_buffer.can_transform('map', 'odom', rclpy.time.Time()):
            return
        
        if not self.received_odometry_to_map:
            self.get_logger().warn("No valid map -> odom transform. Skipping laser point publish.")
            return


        # Subsample the laser scan data
        subsampled_ranges, subsampled_angles = self.subsample_laser_scan(msg)

        # Extract the position and orientation from the robot's odometry
        x = self.robot_odom.pose.pose.position.x
        y = self.robot_odom.pose.pose.position.y

        quaternion = [
            self.robot_odom.pose.pose.orientation.x,
            self.robot_odom.pose.pose.orientation.y,
            self.robot_odom.pose.pose.orientation.z,
            self.robot_odom.pose.pose.orientation.w,
        ]

        # Compute yaw from quaternion (in odom frame)
        siny_cosp = 2 * (quaternion[3] * quaternion[2] + quaternion[0] * quaternion[1])
        cosy_cosp = 1 - 2 * (quaternion[1] * quaternion[1] + quaternion[2] * quaternion[2])
        yaw_in_odom = math.atan2(siny_cosp, cosy_cosp)

        # Compute laser points in the odom frame
        pts_in_odom = [
            (x + r * math.cos(theta + yaw_in_odom), y + r * math.sin(theta + yaw_in_odom), 0.3)
            for r, theta in zip(subsampled_ranges, subsampled_angles)
        ]

        # Transform points from odom to map manually
        try:
            # transform = self.tf_buffer.lookup_transform('map', 'odom', rclpy.time.Time())
            map_tx = self.p_map_odom1[0]
            map_ty = self.p_map_odom1[1]
            map_tz = self.p_map_odom1[2]

            # Extract rotation quaternion
            map_qx = self.q_map_odom1[0]
            map_qy = self.q_map_odom1[1]
            map_qz = self.q_map_odom1[2]
            map_qw = self.q_map_odom1[3]

            # Compute yaw from quaternion (for map -> odom rotation)
            siny_cosp = 2 * (map_qw * map_qz + map_qx * map_qy)
            cosy_cosp = 1 - 2 * (map_qy**2 + map_qz**2)
            yaw_map_to_odom = math.atan2(siny_cosp, cosy_cosp)

            # Build rotation matrix for map -> odom
            cos_yaw = math.cos(yaw_map_to_odom)
            sin_yaw = math.sin(yaw_map_to_odom)
            R_map_odom = [
                [cos_yaw, -sin_yaw, 0],
                [sin_yaw, cos_yaw, 0],
                [0, 0, 1],
            ]
      

            # # Apply transformation to points
            pts_in_map = []
            for pt in pts_in_odom:
                # Rotate
                rotated_x = R_map_odom[0][0] * pt[0] + R_map_odom[0][1] * pt[1]
                rotated_y = R_map_odom[1][0] * pt[0] + R_map_odom[1][1] * pt[1]
                rotated_z = pt[2]  # Z is unchanged since we'r‘e working in 2D

                # Translate
                transformed_x = rotated_x + map_tx
                transformed_y = rotated_y + map_ty
                transformed_z = rotated_z + map_tz
                pts_in_map.append((transformed_x, transformed_y, transformed_z))
        except Exception as e:
            self.get_logger().warn(f"Failed to transform points from odom to map: {e}")
            return

        # Create and publish the laser points marker
        color = ColorRGBA(r=1.0, g=0.0, b=0.0, a=1.0)
        lpmarker = self.get_2d_laser_points_marker(
            self.get_clock().now(),
            "map",
            pts_in_map,
            30000,
            color,
        )
        self.laser_points_marker_pub.publish(lpmarker)


    
    def get_2d_laser_points_marker(self, timestamp, frame_id, pts_in_map, marker_id, rgba):
        """
        Generates an RViz Marker message for visualizing laser scan points in 2D space.
        
        Args:
            timestamp: ROS 2 timestamp of the marker.
            frame_id: Frame ID to associate with the marker (e.g., "map").
            pts_in_map: List of (x, y, z) points representing laser scan points in the map frame.
            marker_id: Unique ID for the marker.
            rgba: Color and transparency of the marker (ColorRGBA).
        
        Returns:
            Marker message for RViz.
        """
        msg = Marker()
        msg.header.stamp = timestamp.to_msg()  
        msg.header.frame_id = frame_id
        msg.ns = 'laser_points'
        msg.id = marker_id
        msg.type = Marker.POINTS  # Marker type for points
        msg.action = Marker.ADD  # Action: add/modify this marker

        # Add points to the marker
        # msg.points = [Point(pt[0], pt[1], pt[2]) for pt in pts_in_map]
        msg.points = [Point(x=pt[0], y=pt[1], z=pt[2]) for pt in pts_in_map]
        # Create and assign colors
        msg.color = rgba


        # Ensure all points are valid
        for pt in pts_in_map:
            assert(not np.isnan(pt).any() and np.isfinite(pt).all()), "Point contains NaN or infinite values."

        # Set marker scale
        msg.scale.x = 0.05  # Width of each point
        msg.scale.y = 0.05  # Height of each point
        msg.scale.z = 0.05  # Depth of each point

        # self.get_logger().info(f"get 2d Laser points marker successful! message updated:")

        return msg

   

    def get_particle_marker(self, timestamp, particle, marker_id):
        """
        Returns an RViz marker that visualizes a single particle.
        """
        msg = Marker()
        msg.header.stamp = timestamp.to_msg()  # Convert ROS 2 timestamp to ROS message format
        msg.header.frame_id = 'map'
        msg.ns = 'particles'
        msg.id = marker_id
        msg.type = Marker.ARROW  # Marker type: ARROW
        msg.action = Marker.ADD  # Marker action: ADD/MODIFY
        msg.lifetime = Duration(seconds=1).to_msg()  # Lifetime of the marker

        # Calculate particle direction
        yaw_in_map = particle.theta
        vx = cos(yaw_in_map)
        vy = sin(yaw_in_map)

        # Set marker color (green)
        msg.color = ColorRGBA()  # RGBA: green, fully opaque
        msg.color.r = 1.0
        msg.color.g = 0.0
        msg.color.b = 0.0
        msg.color.a = 1.0
        # Define start and end points of the arrow
        msg.points.append(Point(x=particle.x, y=particle.y, z=0.2))
        msg.points.append(Point(x=particle.x + 0.2 * vx, y=particle.y + 0.2 * vy, z=0.2))

        # Set scale of the arrow marker
        msg.scale.x = 0.01 # Shaft diameter
        msg.scale.y = 0.02  # Arrowhead diameter
        msg.scale.z = 0.02   # Arrowhead height

       
        # self.get_logger().info(f"GET particle marker successful! message updated: {msg}")

        return msg

    def publish_particle_markers(self):
        """
        Publishes the particles of the particle filter in RViz.
        """
        ma = MarkerArray()
        ts = self.get_clock().now()  # Get current time in ROS 2
        for i, particle in enumerate(self.particles):
            ma.markers.append(self.get_particle_marker(ts, particle, i))

        self.particle_cloud_pub.publish(ma)  # Publish the marker array to RViz
        # self.get_logger().info(f"publish particle markers successful! message updated: ")



def main(args=None):
    rclpy.init(args=args)
    node = MCLNode()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        node.get_logger().info('Node stopped cleanly.')
    except Exception as e:
        node.get_logger().error(f'Exception: {e}')
    finally:
        node.destroy_node()
        rclpy.shutdown()

if __name__ == '__main__':
    main()
