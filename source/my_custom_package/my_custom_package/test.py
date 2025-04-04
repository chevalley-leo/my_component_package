import json
import state_representation as sr
from modulo_components.component import Component
from modulo_core.encoded_state import EncodedState
from clproto import MessageType
from state_representation import CartesianState  # Utilisation de CartesianState

class MoveToPositionComponent(Component):
    def __init__(self, node_name: str, *args, **kwargs):
        super().__init__(node_name, *args, **kwargs)
        
        # Définition des paramètres en tant qu'attributs privés
        self._param_x = sr.Parameter("target_x", 100.0, sr.ParameterType.DOUBLE)
        self._param_y = sr.Parameter("target_y", 100.0, sr.ParameterType.DOUBLE)
        self._param_z = sr.Parameter("target_z", 100.0, sr.ParameterType.DOUBLE)
        self._param_orientation = sr.Parameter("target_orientation", "[0.0, 0.0, 0.0, 1.0]", sr.ParameterType.STRING)

        # Ajout des paramètres au composant
        self.add_parameter(self._param_x, "X-coordinate of target position")
        self.add_parameter(self._param_y, "Y-coordinate of target position")
        self.add_parameter(self._param_z, "Z-coordinate of target position")
        self.add_parameter(self._param_orientation, "Orientation of target position (quaternion)")

        # Initialisation de la position cible
        self.target_position = CartesianState()  # Création d'un nouvel objet CartesianState
        self.target_pose = None  # Définition de target_pose

        self.update_target_position()

        # Définition de la sortie
        self.add_output("target_pose", "Target Cartesian pose", EncodedState, MessageType.CARTESIAN_POSE_MESSAGE)

    def on_execute_callback(self) -> bool:
        """Mise à jour de la position cible à chaque exécution du composant."""
        self.update_target_position()

        # Assigner target_pose avant de publier
        self.target_pose = self.target_position  # Assigner la position cible à target_pose
        
        self.publish_output("target_pose", self.target_pose)
        return True

    def update_target_position(self):
        """Mise à jour de la position cible en fonction des paramètres actuels."""
        # Mise à jour de la position dans CartesianState
        self.target_position.set_position(self._param_x.get_value(), self._param_y.get_value(), self._param_z.get_value())

        # Sécurisation de la conversion du quaternion
        try:
            orientation = json.loads(self._param_orientation.get_value())
            if isinstance(orientation, list) and len(orientation) == 4:
                self.target_position.set_orientation(orientation)
            else:
                raise ValueError("Quaternion must be a list of four float values.")
        except (json.JSONDecodeError, ValueError) as e:
            self.get_logger().error(f"Invalid quaternion format: {e}. Using default orientation.")
            self.target_position.set_orientation([1.0, 0.0, 0.0, 0.0])  # Identity quaternion (par défaut)

    def on_validate_parameter_callback(self, parameter: sr.Parameter) -> bool:
        """Valide et corrige les paramètres si nécessaire."""
        if parameter.get_name() in ["target_x", "target_y", "target_z"]:
            if parameter.get_value() < 0.0:
                self.get_logger_
