import rclpy
from rclpy.node import Node
from geometry_msgs.msg import Twist
from lab_interfaces.msg import ObstacleInfo
from my_robot_interfaces.msg import RobotStatus


class TurnAway(Node):
    def __init__(self):
        super().__init__('turn_away')
        # Declare parameters with default values
        self.declare_parameter('forward_speed', 0.2)
        self.declare_parameter('turn_speed', 0.5)

        self.cmd_vel = self.create_publisher(Twist, 'cmd_vel', 10)
        self.robot_status = self.create_publisher(RobotStatus, '/robot_status', 10)
        self.create_subscription(ObstacleInfo, 'obstacle', self.on_obstacle, 10)

    def on_obstacle(self, msg: ObstacleInfo):
        # Read parameters on every message arrival
        forward_speed = self.get_parameter('forward_speed').value
        turn_speed = self.get_parameter('turn_speed').value

        twist = Twist()
        status = RobotStatus()
        status.obstacle_distance = float(msg.distance)
        if not msg.too_close:
            status.state = 'forward'
            twist.linear.x = float(forward_speed)
            twist.angular.z = 0.0
        else:
            status.state = 'turning'
            twist.linear.x = 0.0
            if msg.angle > 0.0:
                twist.angular.z = -float(turn_speed)
            else:
                twist.angular.z = float(turn_speed)

        self.cmd_vel.publish(twist)
        self.robot_status.publish(status)


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
