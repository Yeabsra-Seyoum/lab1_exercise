
import math
import time
import rclpy
from rclpy.node import Node
from rclpy.action import ActionServer, CancelResponse, GoalResponse
from rclpy.callback_groups import ReentrantCallbackGroup
from rclpy.executors import MultiThreadedExecutor
from geometry_msgs.msg import Twist
from nav_msgs.msg import Odometry

from my_robot_interfaces.action import Rotate


def yaw_from_quaternion(q):
    siny_cosp = 2.0 * (q.w * q.z + q.x * q.y)
    cosy_cosp = 1.0 - 2.0 * (q.y * q.y + q.z * q.z)
    return math.atan2(siny_cosp, cosy_cosp)


def angle_difference(a, b):
    return math.atan2(math.sin(a - b), math.cos(a - b))


class RotateServer(Node):
    def __init__(self):
        super().__init__('rotate_server')

        self.declare_parameter('turn_speed', 0.5)

        self.cb_group = ReentrantCallbackGroup()

        self.cmd_vel_pub = self.create_publisher(Twist, 'cmd_vel', 10)

        self.odom_sub = self.create_subscription(
            Odometry,
            '/odom',
            self.on_odom,
            10,
            callback_group=self.cb_group
        )

        self.current_yaw = None

        self._action_server = ActionServer(
            self,
            Rotate,
            '/rotate',
            execute_callback=self.execute_callback,
            goal_callback=self.goal_callback,
            cancel_callback=self.cancel_callback,
            callback_group=self.cb_group
        )

    def on_odom(self, msg: Odometry):
        self.current_yaw = yaw_from_quaternion(msg.pose.pose.orientation)

    def goal_callback(self, goal_request):
        if abs(goal_request.angle) < 1e-4:
            self.get_logger().info('Rejecting goal: angle is 0')
            return GoalResponse.REJECT
        return GoalResponse.ACCEPT

    def cancel_callback(self, goal_handle):
        self.get_logger().info('Cancel request received')
        return CancelResponse.ACCEPT

    def execute_callback(self, goal_handle):
        # Wait for valid odometry reading
        while self.current_yaw is None and rclpy.ok():
            time.sleep(0.05)

        target_angle = goal_handle.request.angle
        turn_speed = float(self.get_parameter('turn_speed').value)
        direction = 1.0 if target_angle > 0.0 else -1.0

        last_yaw = self.current_yaw
        accumulated_turn = 0.0

        feedback_msg = Rotate.Feedback()
        twist = Twist()

        while rclpy.ok() and abs(accumulated_turn) < abs(target_angle):
            if goal_handle.is_cancel_requested:
                self.cmd_vel_pub.publish(Twist())
                goal_handle.canceled()
                result = Rotate.Result()
                result.turned = float(accumulated_turn)
                return result

            # Integrate angular change
            delta = angle_difference(self.current_yaw, last_yaw)
            accumulated_turn += delta
            last_yaw = self.current_yaw

            # Publish feedback
            feedback_msg.turned = float(accumulated_turn)
            goal_handle.publish_feedback(feedback_msg)

            # Command rotation
            twist.angular.z = direction * turn_speed
            self.cmd_vel_pub.publish(twist)

            time.sleep(0.05)

        # Stop robot on completion
        self.cmd_vel_pub.publish(Twist())
        goal_handle.succeed()

        result = Rotate.Result()
        result.turned = float(accumulated_turn)
        return result


def main(args=None):
    rclpy.init(args=args)
    node = RotateServer()
    executor = MultiThreadedExecutor()
    executor.add_node(node)

    try:
        executor.spin()
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()