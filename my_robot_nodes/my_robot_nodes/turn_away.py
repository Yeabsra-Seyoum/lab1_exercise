import rclpy
from rclpy.node import Node
from rclpy.parameter import Parameter
from geometry_msgs.msg import Twist
from lab_interfaces.msg import ObstacleInfo
from my_robot_interfaces.msg import RobotStatus
from my_robot_interfaces.srv import SetTurnSpeed
from std_srvs.srv import SetBool


class TurnAway(Node):
    def __init__(self):
        super().__init__('turn_away')
        # Declare parameters with default values
        self.declare_parameter('forward_speed', 0.2)
        self.declare_parameter('turn_speed', 0.5)
        self.declare_parameter('max_turn_speed', 1.5)

        self.cmd_vel = self.create_publisher(Twist, 'cmd_vel', 10)
        self.status_pub = self.create_publisher(RobotStatus, '/robot_status', 10)
        self.create_subscription(ObstacleInfo, 'obstacle', self.on_obstacle, 10)
        self.enabled = False
        self.create_service(SetBool, '/turn_away_enable', self.on_enable)
        self.create_service(SetTurnSpeed, '/set_turn_speed', self.on_set_turn_speed)

    def on_enable(self, request, response):
        if self.enabled and not request.data:
            twist = Twist()
            twist.linear.x = 0.0
            twist.angular.z = 0.0
            self.cmd_vel.publish(twist)

        self.enabled = request.data
        response.success = True
        response.message = f'turn_away enabled: {self.enabled}'
        return response

    def on_set_turn_speed(self, request, response):
        max_turn_speed = self.get_parameter('max_turn_speed').value
        if request.turn_speed <= 0.0 or request.turn_speed > max_turn_speed:
            response.success = False
            response.message = (
                f'turn_speed must be > 0.0 and <= {max_turn_speed}'
            )
        else:
            self.set_parameters([
                Parameter(
                    'turn_speed',
                    Parameter.Type.DOUBLE,
                    float(request.turn_speed),
                )
            ])
            response.success = True
            response.message = f'turn_speed set to {request.turn_speed}'
        return response

    def on_obstacle(self, msg: ObstacleInfo):
        status = RobotStatus()
        status.obstacle_distance = float(msg.distance)

        if not self.enabled:
            status.state = 'stopped'
            self.status_pub.publish(status)
            return

        # Read parameters on every message arrival
        forward_speed = self.get_parameter('forward_speed').value
        turn_speed = self.get_parameter('turn_speed').value

        twist = Twist()
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
        self.status_pub.publish(status)


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
