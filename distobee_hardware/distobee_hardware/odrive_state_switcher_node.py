import rclpy
import rclpy.node
from odrive_can.srv import AxisState

ODRIVES = [
    "odrive_back_left",
    "odrive_back_right",
    "odrive_front_left",
    "odrive_front_right",
    "odrive_pipes",
    "odrive_tilt",
]

class OdriveStateSwitcher(rclpy.node.Node):
    def __init__(self):
        super().__init__("odrive_state_switcher")
        self.srv = self.create_service(AxisState, 'odrive_switch_state', self.request_axis_state_callback)

        self.clis = [self.create_client(AxisState, f'/{name}/request_axis_state') for name in ODRIVES]
        for cli in self.clis:
            while not cli.wait_for_service(timeout_sec=1.0):
                self.get_logger().info(f'service {cli.srv_name} not available, waiting again...')

    def request_axis_state_callback(self, request: AxisState.Request, response: AxisState.Response) -> AxisState.Response:
        self.get_logger().info(f'Incoming request for {request.axis_requested_state}')

        for cli in self.clis:
            req = request
            future = cli.call_async(req)
            # FIXME: add a proper executor for spinning the futures
            continue
            # rclpy.spin_until_future_complete(self, future)
            # if future.result() is not None:
            #     self.get_logger().info(f'Service call to {cli.srv_name} successful: {future.result()}')
            # else:
            #     self.get_logger().error(f'Service call to {cli.srv_name} failed {future.exception()}')

        return response


def main():
    try:
        rclpy.init()
        node = OdriveStateSwitcher()
        rclpy.spin(node)
        node.destroy_node()
        rclpy.shutdown()
    except KeyboardInterrupt:
        pass
