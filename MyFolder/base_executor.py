import rclpy
from rclpy.node import Node
from std_msgs.msg import String, Float32
from delivery_interfaces.srv import CancelOrder

import time


class BaseExecutor(Node):

    def __init__(self):
        super().__init__('base_executor')

        self.battery_level = 100.0
        self.cancel_client = self.create_client(CancelOrder, '/cancel_order')

        self.goal_subscriber = self.create_subscription(
            String,
            '/next_goal',
            self.handle_new_goal,
            10
        )

        self.battery_subscriber = self.create_subscription(
            Float32,
            '/battery_pct',
            self.update_battery_level,
            10
        )

        self.goal_reached_publisher = self.create_publisher(
            String,
            '/goal_reached',
            10
        )

        self.get_logger().info('Base Executor has been started.')

    def update_battery_level(self, msg):
        self.battery_level = msg.data

    def handle_new_goal(self, msg):
        current_battery = self.battery_level

        if current_battery < 10.0:
            self.get_logger().warn('Battery low (<10%). Heading to charging station...')
            charging_msg = String()
            charging_msg.data = 'CHARGE,anywhere->charging_station'
            self.goal_reached_publisher.publish(charging_msg)
            return

        self.get_logger().info(f'New goal received: {msg.data}')
        order_id = msg.data.split(',')[0]

        self.get_logger().info('Simulating pickup operation...')
        time.sleep(3)

        self.get_logger().info('Simulating drop-off operation...')
        time.sleep(3)

        completed_msg = String()
        completed_msg.data = order_id
        self.goal_reached_publisher.publish(completed_msg)
        self.get_logger().info(f'Goal completed for order {order_id}')


def main(args=None):
    rclpy.init(args=args)
    executor_node = BaseExecutor()
    rclpy.spin(executor_node)
    executor_node.destroy_node()
    rclpy.shutdown()


if __name__ == '__main__':
    main()

