from modulo_components.component import Component
from sensor_msgs.msg import Image, PointCloud2
from cv_bridge import CvBridge
import cv2
from modulo_core.encoded_state import EncodedState
from state_representation import CartesianPose
from std_msgs.msg import Float64
from clproto import MessageType

class RealSenseComponent(Component):
    def __init__(self, node_name: str, *args, **kwargs):
        super().__init__(node_name, *args, **kwargs)

        # Initialisation des attributs
        self.bridge = CvBridge()
        self._input_depth_image = None
        self._input_rgb_image = None
        self._input_pointcloud = None
        
        self._output_processed_depth_image = None
        self._output_processed_rgb_image = None
        self._output_processed_pointcloud = None

        # Abonnement aux topics
        self.add_input("depth_image", "_input_depth_image", Image)
        self.add_input("rgb_image", "_input_rgb_image", Image)
        self.add_input("pointcloud", "_input_pointcloud", PointCloud2)

        # Publication des résultats traités
        self.add_output("processed_depth_image", "_output_processed_depth_image", Image)
        self.add_output("processed_rgb_image", "_output_processed_rgb_image", Image)
        self.add_output("processed_pointcloud", "_output_processed_pointcloud", PointCloud2)

    def on_execute_callback(self) -> bool:
        if self._input_depth_image is not None:
            processed_depth_image = self.process_depth_image(self._input_depth_image)
            self._output_processed_depth_image = processed_depth_image
            self.publish_output("processed_depth_image", self._output_processed_depth_image)

        if self._input_rgb_image is not None:
            processed_rgb_image = self.process_rgb_image(self._input_rgb_image)
            self._output_processed_rgb_image = processed_rgb_image
            self.publish_output("processed_rgb_image", self._output_processed_rgb_image)

        if self._input_pointcloud is not None:
            processed_pointcloud = self.process_pointcloud(self._input_pointcloud)
            self._output_processed_pointcloud = processed_pointcloud
            self.publish_output("processed_pointcloud", self._output_processed_pointcloud)

        return True

    def process_depth_image(self, data):
        depth_image = self.bridge.imgmsg_to_cv2(data, "32FC1")
        processed_depth_image = cv2.normalize(depth_image, None, 0, 255, cv2.NORM_MINMAX)
        return self.bridge.cv2_to_imgmsg(processed_depth_image, "mono8")

    def process_rgb_image(self, data):
        rgb_image = self.bridge.imgmsg_to_cv2(data, "bgr8")
        gray_image = cv2.cvtColor(rgb_image, cv2.COLOR_BGR2GRAY)
        return self.bridge.cv2_to_imgmsg(gray_image, "mono8")

    def process_pointcloud(self, data):
        # Implémentez votre traitement du nuage de points ici
        return data  # Si aucun traitement n'est nécessaire
