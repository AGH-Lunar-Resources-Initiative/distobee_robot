import rclpy
import rclpy.node
from odrive_can.srv import AxisState
from std_srvs.srv import Empty

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
        self.err_srv = self.create_service(Empty, 'odrive_clear_errors', self.request_clear_errors_callback)

        self.axis_clis = [self.create_client(AxisState, f'/{name}/request_axis_state') for name in ODRIVES]
        self.err_clis = [self.create_client(Empty, f'/{name}/clear_errors') for name in ODRIVES]

        for axis_clis in self.axis_clis:
            while not axis_clis.wait_for_service(timeout_sec=1.0):
                self.get_logger().info(f'service {axis_clis.srv_name} not available, waiting again...')

        for err_cli in self.err_clis:
            while not err_cli.wait_for_service(timeout_sec=1.0):
                self.get_logger().info(f'service {axis_clis.srv_name} not available, waiting again...')

    def request_axis_state_callback(self, request: AxisState.Request, response: AxisState.Response) -> AxisState.Response:
        self.get_logger().info(f'Incoming request for {request.axis_requested_state}')

        for cli in self.axis_clis:
            req = request
            future = cli.call_async(req)
            # FIXME: add a proper executor for spinning the futures
            continue
    
        return response


    def request_clear_errors_callback(self, request: Empty.Request, response: Empty.Response) -> Empty.Response:
        self.get_logger().info(f'Incoming clear error request...')

        for cli in self.err_clis:
            req = request
            future = cli.call_async(req)
            # FIXME: add a proper executor for spinning the futures
            continue

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
