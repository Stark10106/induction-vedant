#!/usr/bin/env python3
import rclpy
from rclpy.node import Node
from sensor_msgs.msg import LaserScan
from geometry_msgs.msg import Twist

class ObstacleAvoiderNode(Node):
    def __init__(self):
        super().__init__('obstacle_avoider')
        self.subscriber = self.create_subscription(
            LaserScan, '/scan', self.scan_callback, 10)
        self.publisher = self.create_publisher(Twist, '/cmd_vel', 10)
        self.timer = self.create_timer(0.1, self.timer_callback)
        self.is_obstacle_detected = False
        self.threshold = 0.5  # meters
        self.speed = 0.2      # meters/second
        self.turn_speed = 0.5 # radians/second

    def scan_callback(self, msg):
        # Define front range: -10 to 10 degrees (assuming 360 readings, 1 degree each)
        front_indices = list(range(330, 360)) + list(range(0, 31))
        front_ranges = [msg.ranges[i] for i in front_indices]
        # Filter valid ranges
        valid_ranges = [r for r in front_ranges if msg.range_min <= r <= msg.range_max]
        right_indices = list(range(270, 300))  # Right side, e.g., 90 to 120 degrees
        right_ranges = [msg.ranges[i] for i in right_indices if msg.range_min <= msg.ranges[i] <= msg.range_max]
        if valid_ranges and min(valid_ranges) < self.threshold:
            self.is_obstacle_detected = True if not right_ranges or min(right_ranges) < self.threshold else False
        else:
            self.is_obstacle_detected = False

    def timer_callback(self):
        twist = Twist()
        if not self.is_obstacle_detected:
            twist.linear.x = self.speed
            twist.angular.z = 0.0
        else:
            twist.linear.x = 0.0
            twist.angular.z = -self.turn_speed
        self.publisher.publish(twist)

def main(args=None):
    rclpy.init(args=args)
    node = ObstacleAvoiderNode()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()
