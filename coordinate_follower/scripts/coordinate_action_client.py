#!/usr/bin/env python3
import rclpy
from rclpy.node import Node
from rclpy.action import ActionClient
from coordinate_follower.action import Coordinate

class CoordinateActionClient(Node):
    def __init__(self):
        super().__init__('coordinate_action_client')
        self._client = ActionClient(self, Coordinate, 'coordinate')

    def send_next_goal(self, x, y):
        goal_msg = Coordinate.Goal()
        goal_msg.x = x
        goal_msg.y = y
        self._client.wait_for_server()
        send_goal_future = self._client.send_goal_async(
            goal_msg,
            feedback_callback=self.feedback_callback)
        send_goal_future.add_done_callback(lambda future: self.goal_response_callback(future, x, y))

    def goal_response_callback(self, future, x, y):
        goal_handle = future.result()
        if not goal_handle.accepted:
            self.get_logger().info(f'Goal rejected for ({x:.2f}, {y:.2f})')
            return
        self.get_logger().info(f'Goal accepted for ({x:.2f}, {y:.2f})')
        get_result_future = goal_handle.get_result_async()
        get_result_future.add_done_callback(lambda future: self.get_result_callback(future, x, y))

    def feedback_callback(self, feedback_msg):
        fb = feedback_msg.feedback
        self.get_logger().info(
            f'At ({fb.current_pose.pose.position.x:.2f}, '
            f'{fb.current_pose.pose.position.y:.2f}) in frame {fb.current_pose.header.frame_id}; '
            f'distance to goal: {fb.distance_to_goal:.2f} m')

    def get_result_callback(self, future, x, y):
        result = future.result().result
        if result.success:
            self.get_logger().info(f'Reached target ({x:.2f}, {y:.2f})!')
        else:
            self.get_logger().info(f'Failed to reach target ({x:.2f}, {y:.2f}).')

def main(args=None):
    import sys
    rclpy.init(args=args)
    node = CoordinateActionClient()
    if len(sys.argv) < 2:
        node.get_logger().error('Please pass coordinates file path')
        return
    try:
        with open(sys.argv[1], 'r') as f:
            for line in f:
                try:
                    x_str, y_str = line.strip().split()
                    x, y = float(x_str), float(y_str)
                    node.send_next_goal(x, y)
                    # Spin until the result is available
                    while rclpy.ok():
                        rclpy.spin_once(node, timeout_sec=0.1)
                        if hasattr(node, '_current_result_future') and node._current_result_future.done():
                            break
                except ValueError as e:
                    node.get_logger().error(f'Invalid coordinate format in line: {line.strip()}')
                    continue
    except FileNotFoundError:
        node.get_logger().error(f'File not found: {sys.argv[1]}')
    except Exception as e:
        node.get_logger().error(f'Error reading file: {str(e)}')
    finally:
        node.destroy_node()
        rclpy.shutdown()

if __name__ == '__main__':
    main()
