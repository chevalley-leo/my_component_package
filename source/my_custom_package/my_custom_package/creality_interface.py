"""
creality_interface.py

Description:
    Author: Léo Chevalley
    This script reads a DXF file, converts its geometric entities to G-code, and sends the G-code to a GRBL-compatible laser engraving machine via serial communication. It supports lines, arcs, circles, ellipses, polylines, and points. The script is intended for use with laser engravers for precise 2D path following and integrates with the modulo_components framework.

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
from modulo_components.component import Component
import state_representation as sr
import serial
import time
import ezdxf
import math
from threading import Event

# ====== CONFIGURATION ======
SCALE_FACTOR = 1.0       # Scale factor
OFFSET_X = 0             # X axis offset
OFFSET_Y = 0             # Y axis offset
LASER_POWER = 300        # Laser power (0 to 1000)
FEED_RATE = 800          # Movement speed in mm/min

# ====== CREALITY INTERFACE CLASS ======
class CrealityInterface(Component):
    # ====== INITIALIZATION ======
    def __init__(self, node_name: str, *args, **kwargs):
        super().__init__(node_name, *args, **kwargs)
        self.add_parameter(sr.Parameter("usb_path", sr.ParameterType.STRING), "USB path")
        self.add_parameter(sr.Parameter("baud_rate", 115200, sr.ParameterType.INT), "Baud rate")
        self.add_parameter(sr.Parameter("LaserPower", LASER_POWER, sr.ParameterType.INT), "Laser power")
        self.add_parameter(sr.Parameter("FeedRate", FEED_RATE, sr.ParameterType.INT), "Feed rate")
        self._serial = serial.Serial(self.get_parameter_value("usb_path"), self.get_parameter_value("baud_rate"))
        self.send_wake_up()
        self.add_predicate("laser_finished", False)
        self.add_service("run_gcode", self._run_gcode)

    # ====== SERVICE HANDLER ======
    def _run_gcode(self, msg):
        self.set_predicate("laser_finished", False)
        self.get_logger().info(f"Received service call with payload {msg}")
        gcode_lines = self._dxf_to_gcode(msg)
        if not gcode_lines:
            self.get_logger().error(f"Error converting file {msg}.")
            return {"success": False, "message": "Error converting DXF file."}
        self.send_gcode_to_machine(gcode_lines)
        time.sleep(0.5)
        self.set_predicate("laser_finished", False)
        return {"success": True, "message": "G-code sent successfully."}

    # ====== SERIAL COMMUNICATION ======
    def send_gcode_to_machine(self, gcode_lines):
        try:
            with serial.Serial(self.get_parameter_value("usb_path"), self.get_parameter_value("baud_rate"), timeout=1) as ser:
                time.sleep(2)
                ser.write(b"\r\n\r\n")  # Wake up controller
                time.sleep(2)
                ser.flushInput()

                self.initialize_machine(ser, do_homing=True)  # Homing automatically

                for line in gcode_lines:
                    command = line.strip() + '\n'
                    ser.write(command.encode('utf-8'))
                    while True:
                        response = ser.readline()
                        if response:
                            decoded = response.strip().decode()
                            self.get_logger().debug(f"Response: {decoded}")
                            if decoded == "ok" or "error" in decoded:
                                break
                        time.sleep(0.01)  # Small pause for GRBL processing

                # Wait until the machine is really Idle
                self.wait_until_idle(ser)
                self.set_predicate("laser_finished", True)

        except serial.SerialException as e:
            self.get_logger().error(f"Serial communication error: {e}")

    def wait_until_idle(self, ser, timeout=120):
        """Wait until the GRBL machine is Idle (job finished)."""
        self.get_logger().info("Waiting for machine to be Idle...")
        start_time = time.time()
        idle_counter = 0
        while True:
            ser.write(b'?\n')
            grbl_out = ser.readline()
            grbl_response = grbl_out.strip().decode('utf-8')
            self.get_logger().info(f"GRBL Response: {grbl_response}")
            if "ok" in grbl_response or "Idle" in grbl_response:
                idle_counter += 1
            else:
                idle_counter = 0
            if idle_counter > 15:
                break
            if time.time() - start_time > timeout:
                self.get_logger().warning("Timeout waiting for machine to be Idle.")
                break
            time.sleep(0.2)

    # ====== PARAMETER VALIDATION ======
    def on_validate_parameter_callback(self, parameter: sr.Parameter) -> bool:
        if parameter.get_name() == "usb_path" and parameter.is_empty():
            self.get_logger().error("Provide a non empty value for parameter 'usb_path'")
            return False
        return True
    
# ====== UTILITY FUNCTIONS ======

    # ====== WAKE UP THE MACHINE ======
    def send_wake_up(self):
        self._serial.write(str.encode("\r\n\r\n"))
        time.sleep(0.2)   # Wait for controller to initialize
        self._serial.flushInput()  # Flush startup text in serial input

    # ====== GCODE UTILITY FUNCTIONS ======
    def remove_comment(string):
        if (string.find(';') == -1):
            return string
        else:
            return string[:string.index(';')]

    def remove_eol_chars(string):
        return string.strip()
    
    def wait_for_movement_completion(self, cleaned_line):
        Event().wait(1)
        if cleaned_line != '$X' or '$$':
            idle_counter = 0
            while True:
                self._serial.reset_input_buffer()
                command = str.encode('?' + '\n')
                self._serial.write(command)
                grbl_out = self._serial.readline() 
                grbl_response = grbl_out.strip().decode('utf-8')

                if grbl_response != 'ok':
                    if grbl_response.find('Idle') > 0:
                        idle_counter += 1
                if idle_counter > 10:
                    break
    
    def _apply_offset(self, x, y):
        x_offset = x * SCALE_FACTOR + OFFSET_X
        y_offset = y * SCALE_FACTOR + OFFSET_Y
        self.get_logger().info(f"Original: ({x}, {y}), Offset: ({x_offset}, {y_offset})")
        return x_offset, y_offset

    def _laser_on(self, power=None):
        if power is None:
            power = self.get_parameter_value("LaserPower")
        return f"M3 S{power}"

    def _laser_off(self):
        return "M5"

    def _rapid_move(self, x, y):
        return f"G0 X{x:.3f} Y{y:.3f}"

    def _engrave_move(self, x, y, feed_rate=None):
        if feed_rate is None:
            feed_rate = self.get_parameter_value("FeedRate")
        return f"G1 X{x:.3f} Y{y:.3f} F{feed_rate}"
    
    def _approximate_arc(self, x0, y0, x1, y1, bulge, segments=10):
        arc_pts = []
        for i in range(1, segments):
            t = i / segments
            xt = (1 - t) * x0 + t * x1
            yt = (1 - t) * y0 + t * y1
            arc_pts.append((xt, yt))
        arc_pts.append((x1, y1))
        return arc_pts

    def _dxf_to_gcode(self, filename):
        try:
            doc = ezdxf.readfile(filename)
        except IOError:
            self.get_logger().error(f"Error opening file {filename}.")
            return []
        except ezdxf.DXFStructureError:
            self.get_logger().error(f"File {filename} is not a valid DXF file.")
            return []

        msp = doc.modelspace()
        gcode = ["G21", "G90", self._laser_off()]  # Basic initialization

        # Process DXF entities
        for entity in msp:
            self.get_logger().info(f"Entity Type: {entity.dxftype()}, Data: {entity.dxf}")

            if entity.dxftype() == "LINE":
                x1, y1 = self._apply_offset(entity.dxf.start.x, entity.dxf.start.y)
                x2, y2 = self._apply_offset(entity.dxf.end.x, entity.dxf.end.y)
                gcode.append(self._laser_off())
                gcode.append(self._rapid_move(x1, y1))
                gcode.append(self._laser_on())
                gcode.append(self._engrave_move(x2, y2))
                gcode.append(self._laser_off())

            elif entity.dxftype() == "ARC":
                cx, cy = self._apply_offset(entity.dxf.center.x, entity.dxf.center.y)
                radius = entity.dxf.radius * SCALE_FACTOR
                start_angle = math.radians(entity.dxf.start_angle)
                end_angle = math.radians(entity.dxf.end_angle)

                num_points = 200
                points = []
                for i in range(num_points + 1):
                    angle = start_angle + (end_angle - start_angle) * (i / num_points)
                    x = cx + radius * math.cos(angle)
                    y = cy + radius * math.sin(angle)
                    points.append((x, y))

                if points:
                    gcode.append(self._laser_off())
                    gcode.append(self._rapid_move(points[0][0], points[0][1]))
                    gcode.append(self._laser_on())
                    for x, y in points[1:]:
                        gcode.append(self._engrave_move(x, y))
                    gcode.append(self._laser_off())

            elif entity.dxftype() == "CIRCLE":
                cx, cy = self._apply_offset(entity.dxf.center.x, entity.dxf.center.y)
                radius = entity.dxf.radius * SCALE_FACTOR

                num_points = 200
                points = [(cx + radius * math.cos(2 * math.pi * i / num_points),
                           cy + radius * math.sin(2 * math.pi * i / num_points)) for i in range(num_points + 1)]

                if points:
                    gcode.append(self._laser_off())
                    gcode.append(self._rapid_move(points[0][0], points[0][1]))
                    gcode.append(self._laser_on())
                    for x, y in points[1:]:
                        gcode.append(self._engrave_move(x, y))
                    gcode.append(self._laser_off())

            elif entity.dxftype() == "ELLIPSE":

                center = entity.dxf.center
                major_axis = entity.dxf.major_axis
                ratio = entity.dxf.ratio 
                start_param = entity.dxf.start_param
                end_param = entity.dxf.end_param

                num_points = 200
                points = []

                major_length = math.hypot(major_axis.x, major_axis.y)

                for i in range(num_points + 1):
                    t = start_param + (end_param - start_param) * i / num_points
                    cos_t = math.cos(t)
                    sin_t = math.sin(t)

                    x_ell = major_length * cos_t
                    y_ell = major_length * ratio * sin_t

                    angle = math.atan2(major_axis.y, major_axis.x)
                    x_rot = x_ell * math.cos(angle) - y_ell * math.sin(angle)
                    y_rot = x_ell * math.sin(angle) + y_ell * math.cos(angle)

                    x = center.x + x_rot
                    y = center.y + y_rot
                    x_off, y_off = self._apply_offset(x, y)
                    points.append((x_off, y_off))

                if points:
                    gcode.append(self._laser_off())
                    gcode.append(self._rapid_move(points[0][0], points[0][1]))
                    gcode.append(self._laser_on())
                    for x, y in points[1:]:
                        gcode.append(self._engrave_move(x, y))
                    gcode.append(self._laser_off())

            elif entity.dxftype() == "LWPOLYLINE":
                vertices = list(entity.vertices())
                if vertices:
                    x0, y0 = self._apply_offset(vertices[0][0], vertices[0][1])
                    gcode.append(self._laser_off())
                    self.get_logger().info(f"Move to start point: ({x0}, {y0})")
                    gcode.append(self._rapid_move(x0, y0))
                    gcode.append(self._laser_on())
                    for i in range(1, len(vertices)):
                        x1_raw, y1_raw = vertices[i][0], vertices[i][1]
                        x1, y1 = self._apply_offset(x1_raw, y1_raw)
                        bulge = vertices[i - 1][4] if len(vertices[i - 1]) >= 5 else 0.0
                        if bulge != 0.0:
                            arc_points = self._approximate_arc(vertices[i - 1][0], vertices[i - 1][1],
                                                            x1_raw, y1_raw, bulge)
                            for px, py in arc_points:
                                px_off, py_off = self._apply_offset(px, py)
                                gcode.append(self._engrave_move(px_off, py_off))
                        else:
                            gcode.append(self._engrave_move(x1, y1))
                    if entity.closed:
                        gcode.append(self._engrave_move(x0, y0))
                    gcode.append(self._laser_off())

            elif entity.dxftype() == "POINT":
                x, y = self._apply_offset(entity.dxf.location.x, entity.dxf.location.y)
                gcode.append(self._laser_off())
                gcode.append(self._rapid_move(x, y))
                gcode.append(self._laser_on())

        gcode.append(self._laser_off())
        gcode.append("G0 X0 Y0")  # Return to origin
        return gcode

    # ====== MACHINE INITIALIZATION ======
    def initialize_machine(self, ser, do_homing=True):
        print("Initializing machine...")
        if do_homing:
            ser.write(b"$H\n")  # Start homing
            time.sleep(1.5)
        ser.write(b"$X\n")  # Unlock machine if needed
        time.sleep(0.2)
        ser.write(b"G21\n")  # Set units to mm
        time.sleep(0.2)
        ser.write(b"G90\n")  # Absolute mode
        time.sleep(0.2)
        ser.write(b"M5\n")   # Laser OFF
        time.sleep(0.2)
        ser.write(b"G92 X0 Y0\n")  # Set start position to (0,0)
        time.sleep(0.2)
        print("Machine initialized and ready.")
