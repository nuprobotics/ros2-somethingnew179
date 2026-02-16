import time

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

        self.service = self.create_service(
            Trigger,
            self.service_name,
            self.trigger_callback,
        )

        self.trigger_client = self.create_client(Trigger, '/spgc/trigger')
        self._trigger_call_done = False
        self._trigger_deadline = time.monotonic() + 5.0
        self._trigger_timer = self.create_timer(0.1, self.try_trigger_call)

    def try_trigger_call(self) -> None:
        if self._trigger_call_done:
            return

        if not self.trigger_client.wait_for_service(timeout_sec=0.0):
            if time.monotonic() >= self._trigger_deadline:
                self._trigger_call_done = True
                self._trigger_timer.cancel()
            return

        self._trigger_call_done = True
        self._trigger_timer.cancel()

        try:
            future = self.trigger_client.call_async(Trigger.Request())
        except Exception:
            return

        future.add_done_callback(self.on_trigger_response)

    def on_trigger_response(self, future) -> None:
        try:
            response = future.result()
        except Exception:
            return

        if response is not None:
            self.stored_string = response.message

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
