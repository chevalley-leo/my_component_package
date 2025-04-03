from modulo_components.component import Component
from state_representation import CartesianPose
from modulo_core.encoded_state import EncodedState
from clproto import MessageType
from modulo_core.parameter import Parameter, ParameterType
import json

class MoveToPositionComponent(Component):
    def __init__(self, node_name: str, *args, **kwargs):
        super().__init__(node_name, *args, **kwargs)
        
        # Définir les paramètres modifiables pour la position (x, y, z) et l'orientation (quaternions)
        self.param_x = Parameter("target_x", ParameterType.FLOAT, default_value=100.0)
        self.param_y = Parameter("target_y", ParameterType.FLOAT, default_value=100.0)
        self.param_z = Parameter("target_z", ParameterType.FLOAT, default_value=100.0)
        self.param_orientation = Parameter("target_orientation", ParameterType.STRING, default_value="[0.0, 0.0, 0.0, 1.0]")  # Quaternions en format string
        
        # Ajout des paramètres à la liste du composant
        self.add_parameter("target_x", "X-coordinate of target position")
        self.add_parameter("target_y", "Y-coordinate of target position")
        self.add_parameter("target_z", "Z-coordinate of target position")
        self.add_parameter("target_orientation", "Orientation of target position (quaternion)")

        # Définir la position cible (initialement basée sur les paramètres)
        self.target_position = CartesianPose()
        self.update_target_position()

        # Sortie : La position cible à atteindre
        self.add_output("target_pose", "target_position", EncodedState, MessageType.CARTESIAN_POSE_MESSAGE)
    
    def on_execute_callback(self) -> bool:
        # Lorsque le composant est activé, on génère la sortie avec la position cible mise à jour
        self.update_target_position()
        self._output_position = self.target_position
        return True
    
    def update_target_position(self):
        # Met à jour la position cible en fonction des paramètres
        self.target_position.position = [self.param_x.value, self.param_y.value, self.param_z.value]
        
        # Conversion de la chaîne de quaternions (format string) en liste de float en utilisant json.loads pour plus de sécurité
        try:
            orientation = json.loads(self.param_orientation.value)  # Utilisation de json.loads() pour convertir la chaîne en liste
            self.target_position.orientation = orientation
        except ValueError:
            # En cas d'échec de la conversion de la chaîne en liste de flottants
            self.get_logger().error("Invalid quaternion format. Using default orientation.")
            self.target_position.orientation = [0.0, 0.0, 0.0, 1.0]  # Orientation par défaut
