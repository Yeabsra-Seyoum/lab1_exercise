import rclpy
from rclpy.node import Node
from geometry_msgs.msg import Twist
from lab_interfaces.msg import ObstacleInfo


class TurnAway(Node):
    def __init__(self):
        super().__init__('turn_away')
        # Declare parameters with default values
        self.declare_parameter('forward_speed', 0.2)
        self.declare_parameter('turn_speed', 0.5)

        self.cmd_vel = self.create_publisher(Twist, 'cmd_vel', 10)
        self.create_subscription(ObstacleInfo, 'obstacle', self.on_obstacle, 10)

    def on_obstacle(self, msg: ObstacleInfo):
        # Read parameters on every message arrival
        forward_speed = self.get_parameter('forward_speed').value
        turn_speed = self.get_parameter('turn_speed').value

        twist = Twist()
        if not msg.too_close:
            twist.linear.x = float(forward_speed)
            twist.angular.z = 0.0
        else:
            twist.linear.x = 0.0
            if msg.angle > 0.0:
                twist.angular.z = -float(turn_speed)
            else:
                twist.angular.z = float(turn_speed)

        self.cmd_vel.publish(twist)


def main(args=None):
    rclpy.init(args=args)
    node = TurnAway()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()