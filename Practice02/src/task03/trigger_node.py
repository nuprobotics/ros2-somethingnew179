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

        self.trigger_client = self.create_client(Trigger, '/spgc/trigger')
        self.stored_string = self._load_trigger_message()

        self.service = self.create_service(
            Trigger,
            self.service_name,
            self.trigger_callback,
        )

    def _load_trigger_message(self) -> str:
        if not self.trigger_client.wait_for_service(timeout_sec=1.0):
            return self.default_string

        future = self.trigger_client.call_async(Trigger.Request())
        rclpy.spin_until_future_complete(self, future, timeout_sec=3.0)

        if not future.done():
            return self.default_string

        try:
            response = future.result()
        except Exception:
            return self.default_string

        if response is None:
            return self.default_string

        return response.message

    def trigger_callback(self, request: Trigger.Request, response: Trigger.Response):
        _ = request
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
