import rclpy
from rclpy.node import Node
from std_msgs.msg import String


class Receiver(Node):
    def __init__(self):
        super().__init__('receiver')
        self.subscription = self.create_subscription(
            String,
            '/spgc/sender',
            self.subscriber_callback,
            10,
        )

    def subscriber_callback(self, msg: String) -> None:
        self.get_logger().info(msg.data)


def main(args=None):
    rclpy.init(args=args)
    node = Receiver()
    try:
        rclpy.spin(node)
    finally:
        node.destroy_node()
        rclpy.shutdown()

if __name__ == '__main__':
    main()
