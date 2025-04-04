from modulo_components.component import Component
import state_representation as sr
from std_msgs.msg import Int32

from modulo_core.encoded_state import EncodedState
from state_representation import JointPositions

from std_msgs.msg import Float64

from modulo_core.encoded_state import EncodedState

from clproto import MessageType
from state_representation import CartesianPose

class PyComponent(Component):
    def __init__(self, node_name: str, *args, **kwargs):
        super().__init__(node_name, *args, **kwargs)
        # add parameters, inputs and outputs here

                # define input data objects as a class attribute
        self._input_number = 0
        self._input_positions = JointPositions()
        
        # bind the attribute to an output using the attribute name and message type
        self.add_input("number", "_input_number", Int32)
        self.add_input("positions", "_input_positions", EncodedState)

                # define output data objects as a class attribute
        self._output_number = 3.14
        self._output_pose = CartesianPose()
        
        # bind the attribute to an output using the attribute name and message type
        self.add_output("number", "_output_number", Float64)
        
        # for encoded states, further define the expected state type when binding the output
        self.add_output("pose", "_output_pose", EncodedState, MessageType.CARTESIAN_POSE_MESSAGE)

    def on_validate_parameter_callback(self, parameter: sr.Parameter) -> bool:
        # validate an incoming parameter value according to some criteria
        return True

    def on_execute_callback(self) -> bool:
        # If the component needs to do any post-construction behavior, invoke `self.execute()`
        # at the end of the constructor, which will trigger this callback in a separate thread.
        # This is only necessary when the behavior would otherwise block the constructor from completing
        # in a timely manner, such as some time-intensive computation or waiting for an external trigger.

        # return True if the execution was successful, False otherwise
        return True
