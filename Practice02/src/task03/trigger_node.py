import rclpy
from rclpy.node import Node
from std_srvs.srv import Trigger


class TriggerNode(Node):
    def __init__(self):
        super().__init__('trigger_node')
        self.declare_parameter('service_name', '/trigger_service')
        self.declare_parameter('default_string', 'No service available')

        self.service_name = (
            self.get_parameter('service_name').get_parameter_value().string_value
        )
        self.default_string = (
            self.get_parameter('default_string').get_parameter_value().string_value
        )
        self.stored_string = self.default_string

        self.trigger_service = self.create_service(
            Trigger,
            self.service_name,
            self.handle_trigger,
        )

        self.trigger_client = self.create_client(Trigger, '/spgc/trigger')
        self._request_trigger_service()

    def _request_trigger_service(self) -> None:
        if not self.trigger_client.wait_for_service(timeout_sec=2.0):
            return

        future = self.trigger_client.call_async(Trigger.Request())
        rclpy.spin_until_future_complete(self, future, timeout_sec=2.0)

        if not future.done():
            return

        result = future.result()
        if result is None:
            return

        self.stored_string = result.message

    def handle_trigger(self, _request: Trigger.Request, response: Trigger.Response):
        response.success = True
        response.message = self.stored_string
        return response


def main(args=None):
    rclpy.init(args=args)
    node = TriggerNode()
    try:
        rclpy.spin(node)
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()
