#!/usr/bin/env python3
import rclpy
from rclpy.action import ActionServer
from rclpy.node import Node
from coordinate_follower.action import Coordinate
from geometry_msgs.msg import Twist, PoseStamped
from nav_msgs.msg import Odometry
from tf_transformations import euler_from_quaternion
import time
import math

class CoordinateActionServer(Node):
    def __init__(self):
        super().__init__('coordinate_action_server')
        self._action_server = ActionServer(
            self,
            Coordinate,
            'coordinate',
            self.execute_callback)
        self.pose = None
        self.subscription = self.create_subscription(
            Odometry, '/odom', self.odom_callback, 10)
        self.cmd_pub = self.create_publisher(Twist, '/cmd_vel', 10)
        self.get_logger().info('Coordinate Action Server started')

    def odom_callback(self, msg: Odometry):
        self.pose = msg.pose.pose

    def normalize_angle(self, angle):
        while angle > math.pi:
            angle -= 2 * math.pi
        while angle < -math.pi:
            angle += 2 * math.pi
        return angle

    def execute_callback(self, goal_handle):
        self.get_logger().info(f'Goal received: x={goal_handle.request.x}, y={goal_handle.request.y}')
        feedback = Coordinate.Feedback()
        result = Coordinate.Result()
        cmd = Twist()
        
        # Wait for initial pose
        timeout = 10.0
        start_time = time.time()
        while self.pose is None and (time.time() - start_time) < timeout:
            self.get_logger().warn('Waiting for odometry data...')
            rclpy.spin_once(self, timeout_sec=0.1)
            
        if self.pose is None:
            self.get_logger().error('No odometry data received within timeout')
            goal_handle.abort()
            result.success = False
            return result

        # Control parameters
        max_iterations = 2000
        iteration = 0
        distance_threshold = 0.1
        angle_threshold = 0.3  # Increased to ~17.2 degrees
        angular_deadband = 0.05  # ~2.9 degrees, no turning if error is smaller
        max_linear_vel = 0.3
        max_angular_vel = 0.2  # Decreased from 0.3
        linear_gain = 1.0
        angular_gain = 1.0  # Decreased from 1.5
        
        while rclpy.ok() and iteration < max_iterations:
            iteration += 1
            
            if goal_handle.is_cancel_requested:
                self.get_logger().info('Goal cancelled')
                cmd.linear.x = 0.0
                cmd.angular.z = 0.0
                self.cmd_pub.publish(cmd)
                goal_handle.canceled()
                result.success = False
                return result

            # Calculate distance and angle to goal
            dx = goal_handle.request.x - self.pose.position.x
            dy = goal_handle.request.y - self.pose.position.y
            dist = math.hypot(dx, dy)
            
            # Get current orientation
            orientation_q = self.pose.orientation
            _, _, current_yaw = euler_from_quaternion([
                orientation_q.x,
                orientation_q.y,
                orientation_q.z,
                orientation_q.w
            ])
            
            # Calculate desired angle to goal
            goal_angle = math.atan2(dy, dx)
            angular_error = self.normalize_angle(goal_angle - current_yaw)

            # Publish feedback
            feedback.current_pose = PoseStamped()
            feedback.current_pose.header.stamp = self.get_clock().now().to_msg()
            feedback.current_pose.header.frame_id = 'odom'
            feedback.current_pose.pose = self.pose
            feedback.distance_to_goal = float(dist)
            goal_handle.publish_feedback(feedback)

            # Check if goal is reached
            if dist < distance_threshold:
                cmd.linear.x = 0.0
                cmd.angular.z = 0.0
                self.cmd_pub.publish(cmd)
                goal_handle.succeed()
                result.success = True
                self.get_logger().info(f'Goal reached! Final distance: {dist:.3f}m')
                return result

            # Two-phase control: Orient first, then move
            if abs(angular_error) > angle_threshold:
                # Phase 1: Orientation correction
                cmd.linear.x = 0.0
                cmd.angular.z = max(-max_angular_vel, min(max_angular_vel, angular_gain * angular_error)) if abs(angular_error) > angular_deadband else 0.0
                self.get_logger().info(f'Orienting: angular_error={angular_error:.3f} rad, cmd.angular.z={cmd.angular.z:.2f}')
            else:
                # Phase 2: Linear movement with minor angular correction
                cmd.linear.x = min(linear_gain * dist, max_linear_vel)
                cmd.angular.z = max(-max_angular_vel * 0.3, min(max_angular_vel * 0.3, angular_gain * 0.5 * angular_error)) if abs(angular_error) > angular_deadband else 0.0
                self.get_logger().info(f'Moving: distance={dist:.3f}m, cmd.linear.x={cmd.linear.x:.2f}, cmd.angular.z={cmd.angular.z:.2f}')
            
            self.cmd_pub.publish(cmd)
            
            if iteration % 5 == 0:  # Log every 0.5 seconds
                self.get_logger().info(f'Iteration {iteration}: Pose=({self.pose.position.x:.2f}, {self.pose.position.y:.2f}), Yaw={current_yaw:.2f}, Angular error={angular_error:.2f}, Cmd=({cmd.linear.x:.2f}, {cmd.angular.z:.2f})')
            
            rclpy.spin_once(self, timeout_sec=0.1)

        # Timeout reached
        self.get_logger().error(f'Failed to reach goal after {max_iterations} iterations')
        cmd.linear.x = 0.0
        cmd.angular.z = 0.0
        self.cmd_pub.publish(cmd)
        goal_handle.abort()
        result.success = False
        return result

def main(args=None):
    rclpy.init(args=args)
    server = CoordinateActionServer()
    rclpy.spin(server)
    server.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()