import socket

import state_representation as sr
from modulo_components.component import Component


class TcpServer(Component):
    def __init__(self, node_name: str, *args, **kwargs):
        super().__init__(node_name, *args, **kwargs)
        self.add_parameter(sr.Parameter("host", "0.0.0.0", sr.ParameterType.STRING), "Host address for the TCP server")
        self.add_parameter(sr.Parameter("port", sr.ParameterType.INT), "Port for the TCP server")
        self.add_parameter(sr.Parameter("save_path", sr.ParameterType.STRING), "Save path for the received data")
        self.execute()

    def on_validate_parameter_callback(self, parameter: sr.Parameter) -> bool:
        if parameter.get_name() == "port" and not parameter:
            self.get_logger().error("Port parameter is required")
            return False
        if parameter.get_name() == "save_path" and not parameter:
            self.get_logger().error("Save path parameter is required")
            return False
        return True

    def on_execute_callback(self) -> bool:
        with socket.socket(socket.AF_INET, socket.SOCK_STREAM) as s:
            host = self.get_parameter_value("host")
            port = self.get_parameter_value("port")
            s.bind((host, port))
            s.listen(1)
            self.get_logger().info(f"TCP server listening on {host}:{port}")

            while True:
                conn, addr = s.accept()
                with conn:
                    self.get_logger().info(f"Connection from {addr}")
                    with open(self.get_parameter_value("save_path"), "wb") as f:
                        while True:
                            data = conn.recv(4096)
                            if not data:
                                break
                            f.write(data)
                    self.get_logger().info("Data saved")
        return True
