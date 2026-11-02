import rclpy
from rclpy.node import Node
from rclpy.action import ActionClient
from action_msgs.msg import GoalStatus
from nav_msgs.msg import Odometry
import time

from lab_interfaces.action import DriveDistance
from my_robot_interfaces.action import Rotate


class SquareDriver(Node):
    def __init__(self):
        super().__init__('square_driver')
        self.declare_parameter('side_length', 0.5)

        self.drive_client = ActionClient(self, DriveDistance, '/drive_distance')
        self.rotate_client = ActionClient(self, Rotate, '/rotate')

        # Track odom so we know the simulation is truly ready before starting
        self.odom_received = False
        self.create_subscription(Odometry, '/odom', self._odom_cb, 10)

    def _odom_cb(self, msg):
        self.odom_received = True

    def run(self):
        side_length = float(self.get_parameter('side_length').value)

        self.get_logger().info('Waiting for /drive_distance and /rotate action servers...')
        self.drive_client.wait_for_server()
        self.rotate_client.wait_for_server()

        # Wait until odometry is actually publishing from Gazebo
        self.get_logger().info('Waiting for simulation odometry...')
        while rclpy.ok() and not self.odom_received:
            rclpy.spin_once(self, timeout_sec=0.1)

        # Give the controllers 1 second to settle
        time.sleep(1.0)

        for side in range(1, 5):
            # 1. Drive side
            self.get_logger().info(f'Driving side {side}/4 (distance: {side_length} m)')
            drive_goal = DriveDistance.Goal()
            drive_goal.distance = side_length

            # Retry sending if rejected due to startup latency
            accepted = False
            for attempt in range(5):
                send_goal_future = self.drive_client.send_goal_async(drive_goal)
                rclpy.spin_until_future_complete(self, send_goal_future)
                goal_handle = send_goal_future.result()

                if goal_handle and goal_handle.accepted:
                    accepted = True
                    break
                self.get_logger().warn(f'DriveDistance goal not accepted (attempt {attempt+1}), retrying in 1s...')
                time.sleep(1.0)

            if not accepted:
                self.get_logger().error(f'DriveDistance goal rejected on side {side}')
                return False

            result_future = goal_handle.get_result_async()
            rclpy.spin_until_future_complete(self, result_future)
            result = result_future.result()

            if result.status != GoalStatus.STATUS_SUCCEEDED:
                self.get_logger().error(f'DriveDistance failed on side {side} with status {result.status}')
                return False

            # 2. Turn left 90 degrees (~1.5708 rad)
            self.get_logger().info(f'Turning left after side {side}/4')
            rotate_goal = Rotate.Goal()
            rotate_goal.angle = 1.5707963

            send_rotate_future = self.rotate_client.send_goal_async(rotate_goal)
            rclpy.spin_until_future_complete(self, send_rotate_future)
            rotate_handle = send_rotate_future.result()

            if not rotate_handle or not rotate_handle.accepted:
                self.get_logger().error(f'Rotate goal rejected on side {side}')
                return False

            rotate_result_future = rotate_handle.get_result_async()
            rclpy.spin_until_future_complete(self, rotate_result_future)
            rotate_result = rotate_result_future.result()

            if rotate_result.status != GoalStatus.STATUS_SUCCEEDED:
                self.get_logger().error(f'Rotate failed on side {side} with status {rotate_result.status}')
                return False

        self.get_logger().info('Square trajectory completed successfully!')
        return True


def main(args=None):
    rclpy.init(args=args)
    node = SquareDriver()
    try:
        node.run()
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()