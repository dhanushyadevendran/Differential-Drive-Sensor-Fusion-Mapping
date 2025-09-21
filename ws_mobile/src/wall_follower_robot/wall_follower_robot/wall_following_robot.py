import rclpy
from rclpy.node import Node

from geometry_msgs.msg import Twist
from sensor_msgs.msg import LaserScan
from nav_msgs.msg import Odometry
from gazebo_msgs.msg import ModelStates
import numpy as np
import math
import matplotlib.pyplot as plt
import csv
import os
from datetime import datetime
try:
    import tf_transformations
except ImportError:
    tf_transformations = None


class WheelDistanceTracker(Node):
    def __init__(self):
        super().__init__('wheel_distance_tracker')

        self.laser_topic = '/scan'
        self.wheel_odom_topic = '/wheel_odom'
        self.cmd_topic = '/cmd_vel'
        self.gt_topic = '/gazebo/model_states'

        self.get_logger().info("Wheel Distance Tracker with Kalman Filter started!")

        self.linear_speed = 0.25
        self.angular_speed = 0.6
        self.desired_distance = 0.8
        self.turn_distance = 1.0

        self.kp = 1.5
        self.kd = 0.3
        self.prev_error = 0.0

        self.ranges = None
        
        self.distance_wheel_encoder = 0.0   
        self.distance_kalman_fused = 0.0    
        
        self.trajectory_wheel = []      
        self.trajectory_kalman = []    
        self.trajectory_gt = []       
        
        self.prev_wheel_pose = None
        self.wheel_x = 0.0
        self.wheel_y = 0.0
        self.wheel_theta = 0.0
        self.current_wheel_velocity = 0.0
        
        self.kalman_state = np.zeros(6)  
        self.kalman_covariance = np.eye(6) * 0.1
        self.kalman_initialized = False
        
        self.setup_kalman_filter()
        
        self.start_pose_wheel = None
        self.current_pose_wheel = None
        self.start_pose_gt = None
        self.current_pose_gt = None
        
        self.loop_complete = False
        self.has_moved_away = False
        
        self.move_away_threshold = 1.5
        self.return_threshold = 0.8
        
        self.laser_count = 0
        self.wheel_count = 0
        self.gt_count = 0
        self.control_count = 0

        self.setup_communication()

        self.timer = self.create_timer(0.1, self.control_loop)
        self.debug_timer = self.create_timer(5.0, self.debug_status)
        
        self.output_dir = f"wheel_analysis_{datetime.now().strftime('%Y%m%d_%H%M%S')}"
        os.makedirs(self.output_dir, exist_ok=True)
        self.get_logger().info(f"Data will be saved to: {self.output_dir}")

    def setup_kalman_filter(self):
        """Initialize Kalman filter matrices"""
        
        self.Q = np.diag([0.01, 0.01, 0.1, 0.1, 0.01, 0.1])
        
        self.R_wheel = np.diag([0.05, 0.05, 0.02])

    def setup_communication(self):
        """Setup publishers and subscribers"""
        try:
            self.cmd_vel_pub = self.create_publisher(Twist, self.cmd_topic, 10)
            
            self.laser_sub = self.create_subscription(LaserScan, self.laser_topic, self.laser_callback, 10)
            self.wheel_sub = self.create_subscription(Odometry, self.wheel_odom_topic, self.wheel_callback, 10)
            self.gt_sub = self.create_subscription(ModelStates, self.gt_topic, self.gt_callback, 10)
            
            self.get_logger().info("All subscriptions created successfully")
            
        except Exception as e:
            self.get_logger().error(f"Error setting up communication: {e}")

    def debug_status(self):
        """Print debug information"""
        self.get_logger().info(f"Data Count: Laser={self.laser_count}, Wheel={self.wheel_count}, GT={self.gt_count}")
        self.get_logger().info(f"Distances: Wheel={self.distance_wheel_encoder:.2f}m, Kalman={self.distance_kalman_fused:.2f}m")
        
        if self.current_pose_wheel is not None:
            self.get_logger().info(f"Wheel Position: ({self.current_pose_wheel[0]:.3f}, {self.current_pose_wheel[1]:.3f})")
            
        if self.current_pose_gt is not None:
            self.get_logger().info(f"GT Position: ({self.current_pose_gt[0]:.3f}, {self.current_pose_gt[1]:.3f})")
        
        if self.kalman_initialized:
            kalman_x, kalman_y = self.kalman_state[0], self.kalman_state[1]
            self.get_logger().info(f"Kalman Position: ({kalman_x:.3f}, {kalman_y:.3f})")
        
        self.get_logger().info(f"Kalman initialized: {self.kalman_initialized}, Trajectories recorded: Wheel={len(self.trajectory_wheel)}, Kalman={len(self.trajectory_kalman)}")

    def laser_callback(self, msg):
        self.laser_count += 1
        self.ranges = np.array(msg.ranges)
        self.ranges[np.isinf(self.ranges)] = msg.range_max
        self.ranges[np.isnan(self.ranges)] = msg.range_max
        
        if self.laser_count == 1:
            self.get_logger().info(f"Laser data active: {len(self.ranges)} points")

    def get_regions(self):
        if self.ranges is None:
            return None
        n = len(self.ranges)
        step = n // 8

        regions = {
            'front': min(np.min(self.ranges[:step]), np.min(self.ranges[-step:])),
            'front_left': np.min(self.ranges[step:2*step]),
            'left': np.min(self.ranges[2*step:3*step]),
            'back_left': np.min(self.ranges[3*step:4*step]),
            'back_right': np.min(self.ranges[4*step:5*step]),
            'right': np.min(self.ranges[5*step:6*step]),
            'front_right': np.min(self.ranges[6*step:7*step]),
        }
        return regions

    def follow_wall(self, regions):
        twist = Twist()

        if regions['front'] < self.turn_distance:
            twist.angular.z = self.angular_speed
            return twist

        right_dist = regions['right']
        error = self.desired_distance - right_dist
        derivative = error - self.prev_error
        self.prev_error = error

        angular_correction = self.kp * error + self.kd * derivative

        twist.linear.x = self.linear_speed
        twist.angular.z = np.clip(angular_correction, -self.angular_speed, self.angular_speed)

        return twist

    def quaternion_to_euler(self, x, y, z, w):
        """Convert quaternion to euler angles"""
        if tf_transformations is not None:
            return tf_transformations.euler_from_quaternion([x, y, z, w])
        else:
            siny_cosp = 2 * (w * z + x * y)
            cosy_cosp = 1 - 2 * (y * y + z * z)
            yaw = math.atan2(siny_cosp, cosy_cosp)
            return 0, 0, yaw

    def wheel_callback(self, msg):
        """Process wheel encoder data and update distance"""
        self.wheel_count += 1
        
        x = msg.pose.pose.position.x
        y = msg.pose.pose.position.y
        
        orientation = msg.pose.pose.orientation
        _, _, theta = self.quaternion_to_euler(
            orientation.x, orientation.y, orientation.z, orientation.w
        )
        
        vx = msg.twist.twist.linear.x
        vy = msg.twist.twist.linear.y
        omega = msg.twist.twist.angular.z
        
        timestamp = self.get_clock().now().nanoseconds / 1e9
        current_pose = (x, y)
        
        self.current_wheel_velocity = math.sqrt(vx*vx + vy*vy)
        
        if self.prev_wheel_pose is not None:
            dx = x - self.prev_wheel_pose[0]
            dy = y - self.prev_wheel_pose[1]
            distance_increment = math.sqrt(dx*dx + dy*dy)
            self.distance_wheel_encoder += distance_increment
        
        self.prev_wheel_pose = (x, y)
        self.wheel_x, self.wheel_y, self.wheel_theta = x, y, theta
        
        self.trajectory_wheel.append((x, y, self.distance_wheel_encoder, timestamp))
        self.current_pose_wheel = current_pose

        if not self.kalman_initialized:
            self.kalman_state = np.array([x, y, vx, vy, theta, omega])
            self.prev_kalman_pos = (x, y)
            self.last_kalman_time = timestamp
            self.kalman_initialized = True
            self.get_logger().info(f"Kalman filter initialized at: ({x:.3f}, {y:.3f})")
            
            self.trajectory_kalman.append((x, y, self.distance_kalman_fused, timestamp))
        else:
            self.kalman_update_wheel([x, y, theta], timestamp)

        if self.start_pose_wheel is None:
            self.start_pose_wheel = current_pose
            self.get_logger().info(f"Wheel encoder tracking started: ({x:.3f}, {y:.3f})")

    def kalman_predict(self, dt):
        """Kalman filter prediction step"""
        F = np.eye(6)
        F[0, 2] = dt
        F[1, 3] = dt 
        F[4, 5] = dt 
        
        self.kalman_state = F @ self.kalman_state
        
        self.kalman_covariance = F @ self.kalman_covariance @ F.T + self.Q

    def kalman_update_wheel(self, measurement, timestamp):
        """Update Kalman filter with wheel encoder measurement [x, y, theta]"""
        if not hasattr(self, 'last_kalman_time'):
            return
            
        dt = timestamp - self.last_kalman_time
        
        if dt > 0 and dt < 1.0: 
            try:
                self.kalman_predict(dt)
                
                H = np.zeros((3, 6))
                H[0, 0] = 1
                H[1, 1] = 1
                H[2, 4] = 1 
                
                z = np.array(measurement)
                y = z - H @ self.kalman_state
                
                while y[2] > math.pi:
                    y[2] -= 2 * math.pi
                while y[2] < -math.pi:
                    y[2] += 2 * math.pi
                
                S = H @ self.kalman_covariance @ H.T + self.R_wheel
                
                if np.linalg.det(S) > 1e-10:
                    K = self.kalman_covariance @ H.T @ np.linalg.inv(S)
                
                    self.kalman_state = self.kalman_state + K @ y
                    self.kalman_covariance = (np.eye(6) - K @ H) @ self.kalman_covariance
                    
                    kalman_x, kalman_y = self.kalman_state[0], self.kalman_state[1]
                    
                    if hasattr(self, 'prev_kalman_pos'):
                        dx = kalman_x - self.prev_kalman_pos[0]
                        dy = kalman_y - self.prev_kalman_pos[1]
                        distance_increment = math.sqrt(dx*dx + dy*dy)
                        self.distance_kalman_fused += distance_increment
                    
                    self.prev_kalman_pos = (kalman_x, kalman_y)
                    
                    self.trajectory_kalman.append((kalman_x, kalman_y, self.distance_kalman_fused, timestamp))
                
            except np.linalg.LinAlgError as e:
                self.get_logger().warn(f"Kalman filter numerical error: {e}")
            
        self.last_kalman_time = timestamp

    def gt_callback(self, msg):
        """Process ground truth data automatically"""
        self.gt_count += 1

        try:
            for i, name in enumerate(msg.name):
                if name.lower() in ['ground_plane', 'sun']:
                    continue

                pose = msg.pose[i]
                x = pose.position.x
                y = pose.position.y

                orientation = pose.orientation
                _, _, theta = self.quaternion_to_euler(
                    orientation.x, orientation.y, orientation.z, orientation.w
                )

                timestamp = self.get_clock().now().nanoseconds / 1e9
                current_pose = (x, y)

                gt_distance = 0.0
                if len(self.trajectory_gt) > 0:
                    prev_gt = self.trajectory_gt[-1]
                    dx = x - prev_gt[0]
                    dy = y - prev_gt[1]
                    gt_distance = prev_gt[2] + math.sqrt(dx*dx + dy*dy)

                self.trajectory_gt.append((x, y, gt_distance, timestamp))
                self.current_pose_gt = current_pose

                if self.start_pose_gt is None:
                    self.start_pose_gt = current_pose
                    self.get_logger().info(f"Ground truth reference initialized: ({x:.3f}, {y:.3f})")

                break  

        except Exception as e:
            if self.gt_count < 3:
                self.get_logger().warn(f"Ground truth error: {e}")


    def check_loop_completion(self):
        """Check for loop completion"""
        if self.current_pose_wheel is not None and self.start_pose_wheel is not None:
            current_pose = self.current_pose_wheel
            start_pose = self.start_pose_wheel
            source = "WHEEL"
        elif self.current_pose_gt is not None and self.start_pose_gt is not None:
            current_pose = self.current_pose_gt
            start_pose = self.start_pose_gt
            source = "GT"
        else:
            return False

        dx = current_pose[0] - start_pose[0]
        dy = current_pose[1] - start_pose[1]
        dist_from_start = math.sqrt(dx*dx + dy*dy)

        if not self.has_moved_away:
            if dist_from_start > self.move_away_threshold:
                self.has_moved_away = True
                self.get_logger().info(f"Robot moved away! Distance: {dist_from_start:.3f}m ({source})")
            return False

        if dist_from_start < self.return_threshold:
            self.get_logger().info(f"LOOP COMPLETED! Distance from start: {dist_from_start:.3f}m ({source})")
            return True

        return False

    def save_data(self):
        """Save all trajectory and distance data"""
        try:
            if self.trajectory_wheel:
                wheel_file = os.path.join(self.output_dir, 'wheel_encoder_distance.csv')
                with open(wheel_file, 'w', newline='') as f:
                    writer = csv.writer(f)
                    writer.writerow(['timestamp', 'x', 'y', 'cumulative_distance'])
                    for x, y, dist, t in self.trajectory_wheel:
                        writer.writerow([t, x, y, dist])
                self.get_logger().info(f"Saved wheel encoder: {len(self.trajectory_wheel)} points")
            
            if self.trajectory_kalman:
                kalman_file = os.path.join(self.output_dir, 'kalman_fused_distance.csv')
                with open(kalman_file, 'w', newline='') as f:
                    writer = csv.writer(f)
                    writer.writerow(['timestamp', 'x', 'y', 'cumulative_distance'])
                    for x, y, dist, t in self.trajectory_kalman:
                        writer.writerow([t, x, y, dist])
                self.get_logger().info(f"Saved Kalman fused: {len(self.trajectory_kalman)} points")
                
            if self.trajectory_gt:
                gt_file = os.path.join(self.output_dir, 'ground_truth.csv')
                with open(gt_file, 'w', newline='') as f:
                    writer = csv.writer(f)
                    writer.writerow(['timestamp', 'x', 'y', 'cumulative_distance'])
                    for x, y, dist, t in self.trajectory_gt:
                        writer.writerow([t, x, y, dist])
                self.get_logger().info(f"Saved ground truth: {len(self.trajectory_gt)} points")
                
        except Exception as e:
            self.get_logger().error(f"Error saving data: {e}")
            
    def create_analysis_plots(self):
        """Create comprehensive analysis plots"""
        try:
            plt.style.use('seaborn-v0_8' if 'seaborn-v0_8' in plt.style.available else 'default')
            
            fig = plt.figure(figsize=(20, 15))
            
            ax1 = plt.subplot(2, 3, 1)
            ax1.set_title('Trajectory Comparison - Wheel Encoder vs Kalman', fontsize=14, fontweight='bold')
            
            if self.trajectory_gt:
                gt_x, gt_y = zip(*[(x, y) for x, y, _, _ in self.trajectory_gt])
                ax1.plot(gt_x, gt_y, 'k-', linewidth=3, label='Ground Truth', alpha=0.8, zorder=4)
                ax1.scatter(gt_x[0], gt_y[0], color='black', s=150, marker='o', label='Start', edgecolor='white', linewidth=2, zorder=5)
                ax1.scatter(gt_x[-1], gt_y[-1], color='black', s=150, marker='s', label='End', edgecolor='white', linewidth=2, zorder=5)
                
            if self.trajectory_wheel:
                wheel_x, wheel_y = zip(*[(x, y) for x, y, _, _ in self.trajectory_wheel])
                ax1.plot(wheel_x, wheel_y, 'b-', linewidth=2, label='Wheel Encoder', alpha=0.7, zorder=3)
                
            if self.trajectory_kalman:
                kalman_x, kalman_y = zip(*[(x, y) for x, y, _, _ in self.trajectory_kalman])
                ax1.plot(kalman_x, kalman_y, 'g--', linewidth=2, label='Kalman Fused', alpha=0.7, zorder=1)
            
            ax1.set_xlabel('X Position (m)')
            ax1.set_ylabel('Y Position (m)')
            ax1.legend()
            ax1.grid(True, alpha=0.3)
            ax1.set_aspect('equal')
            
            ax2 = plt.subplot(2, 3, 2)
            ax2.set_title('Distance Traveled Comparison', fontsize=14, fontweight='bold')
            
            methods = []
            distances = []
            colors = []
            
            if self.distance_wheel_encoder > 0:
                methods.append('Wheel\nEncoder')
                distances.append(self.distance_wheel_encoder)
                colors.append('blue')
                
            if self.distance_kalman_fused > 0:
                methods.append('Kalman\nFused')
                distances.append(self.distance_kalman_fused)
                colors.append('green')
            
            if methods:
                bars = ax2.bar(methods, distances, color=colors, alpha=0.7, edgecolor='black')
                
                for bar, distance in zip(bars, distances):
                    height = bar.get_height()
                    ax2.text(bar.get_x() + bar.get_width()/2., height,
                           f'{distance:.2f}m', ha='center', va='bottom', fontweight='bold')
            
            ax2.set_ylabel('Total Distance (m)')
            ax2.grid(True, alpha=0.3, axis='y')
            
            ax3 = plt.subplot(2, 3, 3)
            ax3.set_title('Cumulative Distance vs Time', fontsize=14, fontweight='bold')
            
            if self.trajectory_wheel:
                wheel_times = [(t - self.trajectory_wheel[0][3]) for _, _, _, t in self.trajectory_wheel]
                wheel_distances = [dist for _, _, dist, _ in self.trajectory_wheel]
                ax3.plot(wheel_times, wheel_distances, 'b-', linewidth=2, label='Wheel Encoder')
                
            if self.trajectory_kalman:
                kalman_times = [(t - self.trajectory_kalman[0][3]) for _, _, _, t in self.trajectory_kalman]
                kalman_distances = [dist for _, _, dist, _ in self.trajectory_kalman]
                ax3.plot(kalman_times, kalman_distances, 'g--', linewidth=2, label='Kalman Fused')
            
            ax3.set_xlabel('Time (s)')
            ax3.set_ylabel('Cumulative Distance (m)')
            ax3.legend()
            ax3.grid(True, alpha=0.3)
            
            ax4 = plt.subplot(2, 3, 4)
            ax4.set_title('X Position vs Time', fontsize=14, fontweight='bold')
            
            if self.trajectory_wheel:
                wheel_times = [(t - self.trajectory_wheel[0][3]) for _, _, _, t in self.trajectory_wheel]
                wheel_x = [x for x, _, _, _ in self.trajectory_wheel]
                ax4.plot(wheel_times, wheel_x, 'b-', linewidth=2, label='Wheel Encoder')
                
            if self.trajectory_kalman:
                kalman_times = [(t - self.trajectory_kalman[0][3]) for _, _, _, t in self.trajectory_kalman]
                kalman_x = [x for x, _, _, _ in self.trajectory_kalman]
                ax4.plot(kalman_times, kalman_x, 'g--', linewidth=2, label='Kalman Fused')
            
            ax4.set_xlabel('Time (s)')
            ax4.set_ylabel('X Position (m)')
            ax4.legend()
            ax4.grid(True, alpha=0.3)
            
            ax5 = plt.subplot(2, 3, 5)
            ax5.set_title('Y Position vs Time', fontsize=14, fontweight='bold')
            
            if self.trajectory_wheel:
                wheel_times = [(t - self.trajectory_wheel[0][3]) for _, _, _, t in self.trajectory_wheel]
                wheel_y = [y for _, y, _, _ in self.trajectory_wheel]
                ax5.plot(wheel_times, wheel_y, 'b-', linewidth=2, label='Wheel Encoder')
                
            if self.trajectory_kalman:
                kalman_times = [(t - self.trajectory_kalman[0][3]) for _, _, _, t in self.trajectory_kalman]
                kalman_y = [y for _, y, _, _ in self.trajectory_kalman]
                ax5.plot(kalman_times, kalman_y, 'g--', linewidth=2, label='Kalman Fused')
            
            ax5.set_xlabel('Time (s)')
            ax5.set_ylabel('Y Position (m)')
            ax5.legend()
            ax5.grid(True, alpha=0.3)
            
            ax6 = plt.subplot(2, 3, 6)
            ax6.set_title('Kalman Filter Benefits', fontsize=14, fontweight='bold')
            
            if self.distance_wheel_encoder > 0 and self.distance_kalman_fused > 0:
                improvement = abs(self.distance_kalman_fused - self.distance_wheel_encoder) / self.distance_wheel_encoder * 100
                ax6.text(0.1, 0.8, f'Kalman vs Wheel Encoder:', transform=ax6.transAxes, fontweight='bold')
                ax6.text(0.1, 0.7, f'Difference: {improvement:.1f}%', transform=ax6.transAxes)
            
            ax6.text(0.1, 0.2, 'Benefits of Kalman Filter:', transform=ax6.transAxes, fontweight='bold')
            ax6.text(0.1, 0.1, '• Combines sensor strengths\n• Reduces individual sensor weaknesses\n• Provides uncertainty estimates', transform=ax6.transAxes)
            ax6.set_xlim(0, 1)
            ax6.set_ylim(0, 1)
            ax6.axis('off')
            
            plt.tight_layout()
            
            plot_file = os.path.join(self.output_dir, 'wheel_kalman_analysis.png')
            plt.savefig(plot_file, dpi=300, bbox_inches='tight')
            self.get_logger().info(f"Saved comprehensive analysis: {plot_file}")
            
            self.create_clean_map()
            
        except Exception as e:
            self.get_logger().error(f"Error creating plots: {e}")

    def create_clean_map(self):
        """Create clean trajectory map view"""
        try:
            fig, ax = plt.subplots(1, 1, figsize=(12, 10))
            ax.set_title('Robot Navigation: Wheel Encoder vs Kalman Fusion', fontsize=16, fontweight='bold')
            
            # Plot all trajectories
            if self.trajectory_gt:
                gt_x, gt_y = zip(*[(x, y) for x, y, _, _ in self.trajectory_gt])
                ax.plot(gt_x, gt_y, 'k-', linewidth=4, label='Ground Truth', alpha=0.9, zorder=4)
                ax.scatter(gt_x[0], gt_y[0], color='black', s=200, marker='o', label='Start Position', 
                          edgecolor='white', linewidth=3, zorder=6)
                ax.scatter(gt_x[-1], gt_y[-1], color='black', s=200, marker='s', label='End Position', 
                          edgecolor='white', linewidth=3, zorder=6)
                
            if self.trajectory_wheel:
                wheel_x, wheel_y = zip(*[(x, y) for x, y, _, _ in self.trajectory_wheel])
                ax.plot(wheel_x, wheel_y, 'b-', linewidth=3, label=f'Wheel Encoder ({self.distance_wheel_encoder:.2f}m)', alpha=0.8, zorder=3)
                
                
            if self.trajectory_kalman:
                kalman_x, kalman_y = zip(*[(x, y) for x, y, _, _ in self.trajectory_kalman])
                ax.plot(kalman_x, kalman_y, 'g--', linewidth=2, label=f'Kalman Fused ({self.distance_kalman_fused:.2f}m)', alpha=0.8, zorder=1)
            
            ax.set_xlabel('X Position (m)', fontsize=12)
            ax.set_ylabel('Y Position (m)', fontsize=12)
            ax.legend(loc='upper right', fontsize=11)
            ax.grid(True, alpha=0.3)
            ax.set_aspect('equal')
            
            plt.tight_layout()
            
            map_file = os.path.join(self.output_dir, 'navigation_map_clean.png')
            plt.savefig(map_file, dpi=300, bbox_inches='tight')
            self.get_logger().info(f"Saved clean navigation map: {map_file}")
            
        except Exception as e:
            self.get_logger().error(f"Error creating clean map: {e}")

    def control_loop(self):
        self.control_count += 1
        
        if self.ranges is None:
            return

        if self.loop_complete:
            stop_twist = Twist()
            self.cmd_vel_pub.publish(stop_twist)
            return

        regions = self.get_regions()
        if regions is None:
            return
            
        if self.check_loop_completion():
            self.loop_complete = True
            stop_twist = Twist()
            self.cmd_vel_pub.publish(stop_twist)
            
            self.get_logger().info("Saving sensor fusion data...")
            self.save_data()
            
            self.get_logger().info("Creating analysis plots...")
            self.create_analysis_plots()
            
            self.get_logger().info("SENSOR FUSION ANALYSIS COMPLETE!")
            return

        # Continue wall following
        twist = self.follow_wall(regions)
        self.cmd_vel_pub.publish(twist)


def main(args=None):
    rclpy.init(args=args)
    node = WheelDistanceTracker()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        node.get_logger().info("Interrupted by user")
        if not node.loop_complete:
            node.save_data()
            node.create_analysis_plots()
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()