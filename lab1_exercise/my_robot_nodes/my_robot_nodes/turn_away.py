import rclpy
from rclpy.node import Node
from geometry_msgs.msg import Twist
from lab_interfaces.msg import ObstacleInfo


class TurnAwayNode(Node):
    def __init__(self):
        super().__init__('turn_away')

        # Publisher to drive commands
        self.cmd_vel_pub = self.create_publisher(Twist, '/cmd_vel', 10)

        # Subscriber to obstacle detector
        self.obstacle_sub = self.create_subscription(
            ObstacleInfo,
            '/obstacle',
            self.obstacle_callback,
            10
        )
        self.get_logger().info('turn_away node started.')

    def obstacle_callback(self, msg: ObstacleInfo):
        twist = Twist()

        if not msg.too_close:
            # Path is clear: drive straight ahead
            twist.linear.x = 0.2
            twist.angular.z = 0.0
        else:
            # Obstacle detected: stop forward motion and turn away
            twist.linear.x = 0.0
            if msg.angle > 0.0:
                # Obstacle on the left -> turn right
                twist.angular.z = -0.5
            else:
                # Obstacle on the right or center -> turn left
                twist.angular.z = 0.5

        self.cmd_vel_pub.publish(twist)


def main(args=None):
    rclpy.init(args=args)
    node = TurnAwayNode()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()