import rclpy
from rclpy.node import Node
from collections import deque

from delivery_interfaces.msg import DeliveryOrder, OrderStatus
from delivery_interfaces.srv import CancelOrder
from std_msgs.msg import String


class OrderManager(Node):
    def __init__(self):
        super().__init__('task_controller')

        self.queue = deque()
        self.current_task = None

        self.order_subscriber = self.create_subscription(
            DeliveryOrder,
            '/delivery_order',
            self.on_order_received,
            10
        )

        self.cancel_service = self.create_service(
            CancelOrder,
            '/cancel_order',
            self.on_order_cancel_request
        )

        self.status_pub = self.create_publisher(
            OrderStatus,
            '/order_status',
            10
        )

        self.goal_pub = self.create_publisher(
            String,
            '/next_goal',
            10
        )

        self.goal_sub = self.create_subscription(
            String,
            '/goal_reached',
            self.on_goal_reached,
            10
        )

        self.dispatch_timer = self.create_timer(2.0, self.dispatch_order)

        self.get_logger().info('TaskController Node is up and running.')

    def on_order_received(self, msg):
        self.queue.append(msg)
        self.get_logger().info(
            f"Order accepted: ID={msg.order_id}, Item={msg.item}, "
            f"From: {msg.pickup_room} → To: {msg.dropoff_room}"
        )
        self.update_order_status(msg.order_id, "NEW")

    def on_order_cancel_request(self, request, response):
        # تلاش برای لغو سفارش از صف
        for entry in list(self.queue):
            if entry.order_id == request.order_id:
                self.queue.remove(entry)
                self.update_order_status(request.order_id, "CANCELED")
                self.get_logger().info(f"Order {request.order_id} was removed from queue.")
                response.success = True
                response.message = "Order was canceled from queue."
                return response

        # اگر سفارش در حال اجراست، لغوش کن
        if self.current_task and self.current_task.order_id == request.order_id:
            self.get_logger().info(f"Order {request.order_id} was canceled during execution.")
            self.update_order_status(request.order_id, "CANCELED")
            self.current_task = None
            response.success = True
            response.message = "Order was canceled while executing."
            return response

        self.get_logger().warn(f"Order ID {request.order_id} not found for cancellation.")
        response.success = False
        response.message = "Cancellation failed. Order not found."
        return response

    def update_order_status(self, order_id, new_status):
        status_msg = OrderStatus()
        status_msg.order_id = order_id
        status_msg.status = new_status
        self.status_pub.publish(status_msg)
        self.get_logger().info(f"Status update: Order {order_id} → {new_status}")

    def dispatch_order(self):
        if self.current_task is None and self.queue:
            next_order = self.queue.popleft()
            self.current_task = next_order

            goal_info = f"{next_order.order_id},{next_order.pickup_room}->{next_order.dropoff_room}"
            msg = String()
            msg.data = goal_info
            self.goal_pub.publish(msg)

            self.update_order_status(next_order.order_id, "DISPATCHED")
            self.get_logger().info(f"Sent goal: {msg.data}")

    def on_goal_reached(self, msg):
        completed_id = msg.data.strip()

        if self.current_task and self.current_task.order_id == completed_id:
            self.get_logger().info(f"Task {completed_id} completed successfully.")
            self.update_order_status(completed_id, "DONE")
            self.current_task = None
        else:
            self.get_logger().warn(f"Unexpected completion notice received for {completed_id}.")


def main(args=None):
    rclpy.init(args=args)
    node = OrderManager()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()

