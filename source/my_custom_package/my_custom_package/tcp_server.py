"""
tcp_server.py

Description:
    Author: Léo Chevalley
    This script implements a TCP server component that listens for incoming connections, receives data, and saves it to a specified path. It is designed to be used as part of the modulo_components framework for modular robotics or automation systems.

License:
    MIT License
    Copyright (c) 2025 Léo Chevalley
    Permission is hereby granted, free of charge, to any person obtaining a copy
    of this software and associated documentation files (the "Software"), to deal
    in the Software without restriction, including without limitation the rights
    to use, copy, modify, merge, publish, distribute, sublicense, and/or sell
    copies of the Software, and to permit persons to whom the Software is
    furnished to do so, subject to the following conditions:

    The above copyright notice and this permission notice shall be included in all
    copies or substantial portions of the Software.

    THE SOFTWARE IS PROVIDED "AS IS", WITHOUT WARRANTY OF ANY KIND, EXPRESS OR
    IMPLIED, INCLUDING BUT NOT LIMITED TO THE WARRANTIES OF MERCHANTABILITY,
    FITNESS FOR A PARTICULAR PURPOSE AND NONINFRINGEMENT. IN NO EVENT SHALL THE
    AUTHORS OR COPYRIGHT HOLDERS BE LIABLE FOR ANY CLAIM, DAMAGES OR OTHER
    LIABILITY, WHETHER IN AN ACTION OF CONTRACT, TORT OR OTHERWISE, ARISING FROM,
    OUT OF OR IN CONNECTION WITH THE SOFTWARE OR THE USE OR OTHER DEALINGS IN THE
    SOFTWARE.
"""

# ====== IMPORTS ======
import socket
import state_representation as sr
from modulo_components.component import Component

# ====== TCP SERVER CLASS ======
class TcpServer(Component):
    # ====== INITIALIZATION ======
    def __init__(self, node_name: str, *args, **kwargs):
        super().__init__(node_name, *args, **kwargs)
        self.add_parameter(sr.Parameter("host", "0.0.0.0", sr.ParameterType.STRING), "Host address for the TCP server")
        self.add_parameter(sr.Parameter("port", sr.ParameterType.INT), "Port for the TCP server")
        self.add_parameter(sr.Parameter("save_path", sr.ParameterType.STRING), "Save path for the received data")
        self.execute()

    # ====== PARAMETER VALIDATION ======
    def on_validate_parameter_callback(self, parameter: sr.Parameter) -> bool:
        name = parameter.get_name()
        if name == "port" and not parameter:
            self.get_logger().error("Port parameter is required")
            return False
        if name == "save_path" and not parameter:
            self.get_logger().error("Save path parameter is required")
            return False
        return True

    # ====== MAIN EXECUTION ======
    def on_execute_callback(self) -> bool:
        host = self.get_parameter_value("host")
        port = self.get_parameter_value("port")
        save_path = self.get_parameter_value("save_path")
        with socket.socket(socket.AF_INET, socket.SOCK_STREAM) as s:
            s.bind((host, port))
            s.listen(1)
            self.get_logger().info(f"TCP server listening on {host}:{port}")
            while True:
                conn, addr = s.accept()
                with conn:
                    self.get_logger().info(f"Connection from {addr}")
                    with open(save_path, "wb") as f:
                        while True:
                            data = conn.recv(4096)
                            if not data:
                                break
                            f.write(data)
                    self.get_logger().info("Data saved")
        return True
