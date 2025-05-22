from modulo_components.component import Component
import state_representation as sr
import serial
import time
from threading import Event


class CrealityInterface_1(Component):
    def __init__(self, node_name: str, *args, **kwargs):
        super().__init__(node_name, *args, **kwargs)
        
        self.add_parameter(sr.Parameter("usb_path", sr.ParameterType.STRING), "USB path")
        self.add_parameter(sr.Parameter("baud_rate", 115200, sr.ParameterType.INT), "Baud rate")

        self._serial = serial.Serial(self.get_parameter_value("usb_path"), self.get_parameter_value("baud_rate"))
        self.send_wake_up()

        self.add_service("run_gcode", self._run_gcode)

    def _run_gcode(self, msg):
        self.get_logger().info(f"Received service call with payload {msg}")
        # check that filepath exists...
        '''with open(msg, "r") as file:
            for line in file:
                # cleaning up gcode from file
                cleaned_line = self.remove_eol_chars(self.remove_comment(line))
                if cleaned_line:  # checks if string is empty
                    # converts string to byte encoded string and append newline
                    command = str.encode(line + '\n')
                    self._serial.write(command)  # Send g-code

                    self.wait_for_movement_completion(cleaned_line)

                    grbl_out = self._serial.readline()  # Wait for response with carriage return
                    self.get_logger().debug(f"{grbl_out.strip().decode('utf-8')}")'''
        return {"success": True, "message": "Success"}

    def on_validate_parameter_callback(self, parameter: sr.Parameter) -> bool:
        if parameter.get_name() == "usb_path" and parameter.is_empty():
            self.get_logger().error("Provide a non empty value for parameter 'usb_path'")
            return False
        return True

    def send_wake_up(self):
        # Wake up, hit enter a few times to wake the Printrbot
        self._serial.write(str.encode("\r\n\r\n"))
        time.sleep(1)   # Wait for Printrbot to initialize
        self._serial.flushInput()  # Flush startup text in serial input

    def remove_comment(string):
        if (string.find(';') == -1):
            return string
        else:
            return string[:string.index(';')]

    def remove_eol_chars(string):
        # removed \n or traling spaces
        return string.strip()
    
    def wait_for_movement_completion(self, cleaned_line):
        Event().wait(1)
        if cleaned_line != '$X' or '$$':
            idle_counter = 0
            while True:
                # Event().wait(0.01)
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
