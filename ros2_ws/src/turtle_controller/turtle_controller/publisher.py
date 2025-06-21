import rclpy
from rclpy.node import Node
from geometry_msgs.msg import Twist

class TurtleController(Node):
    def __init__(self):
        super().__init__('turtle_controller')
        self.publisher_ = self.create_publisher(Twist, '/cmd_vel', 10)
        timer_period = 0.5
        self.timer = self.create_timer(timer_period, self.publish_velocity)
        self.get_logger().info('Turtle Controller Node Started')

    def publish_velocity(self):
        msg = Twist()
        msg.linear.x = float(1.0)
        msg.linear.y = float(0.0)
        msg.linear.z = float(0.0)
        msg.angular.x = float(0.0)
        msg.angular.y = float(0.0)
        msg.angular.z = float(2.0)
        self.publisher_.publish(msg)
        self.get_logger().info(f'Publishing: linear={msg.linear.x}, angular={msg.angular.z}')

def main(args=None):
    rclpy.init(args=args)
    node = TurtleController()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()
