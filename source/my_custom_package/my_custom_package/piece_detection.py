import numpy as np
import open3d as o3d
import state_representation as sr
from cv_bridge import CvBridge
from modulo_components.lifecycle_component import LifecycleComponent
from modulo_core import EncodedState
from scipy.spatial import cKDTree
from scipy.spatial.transform import Rotation as R
from sensor_msgs.msg import CameraInfo, Image
from sklearn.cluster import DBSCAN
from copy import copy


class PieceDetectionComponent(LifecycleComponent):
    def __init__(self, node_name: str, *args, **kwargs):
        super().__init__(node_name, *args, **kwargs)

        # Définir les paramètres
        self._max_cluster_size = 10000
        self._voxel_size = 0.001
        self._min_depth = 150
        self._max_depth = 700
        self._target_color = None
        self._positive_tolerance = None
        self._negative_tolerance = None

        self.add_parameter(sr.Parameter("piece_path", "piece.stl", sr.ParameterType.STRING), "Path to the STL file")

        self.add_parameter(
            sr.Parameter("number_of_points", 5000, sr.ParameterType.INT), "Number of points to sample from the mesh"
        )

        self.add_parameter(sr.Parameter("voxel_size", self._voxel_size, sr.ParameterType.DOUBLE), "Voxel size for downsampling")

        self.add_parameter(
            sr.Parameter("min_depth", self._min_depth, sr.ParameterType.INT), "Minimum depth for filtering"
        )

        self.add_parameter(
            sr.Parameter("max_depth", self._max_depth, sr.ParameterType.INT), "Maximum depth for filtering"
        )
        self.add_parameter(
            sr.Parameter("color_to_filter", [128, 96, 49], sr.ParameterType.INT_ARRAY),
            "Color to filter in RGB format",
        )

        self.add_parameter(
            sr.Parameter("positive_tolerance", [20, 20, 20], sr.ParameterType.INT_ARRAY),
            "Tolerance for positive filtering",
        )

        self.add_parameter(
            sr.Parameter("negative_tolerance", [30, 30, 30], sr.ParameterType.INT_ARRAY),
            "Tolerance for negative filtering",
        )

        self.add_parameter(
            sr.Parameter(
                "camera_pose_world", [0.420451, 0.0175, 0.766251, 0.707107, 0, -0.707107, 0], sr.ParameterType.DOUBLE_ARRAY
            ),
            "Camera pose in world coordinates",
        )

        self.add_parameter(sr.Parameter("max_cluster_size", self._max_cluster_size, sr.ParameterType.INT), "Maximum cluster size")

        self.add_parameter(
            sr.Parameter("fit_threshold", 97.0, sr.ParameterType.DOUBLE),
            "Fit threshold (%) for confirming the piece"
        )

        self.cv_bridge = CvBridge()
        self.depth_camera_info = None
        self.add_input("depth_camera_info", self._on_depth_info, CameraInfo)
        self.color_image = None
        self.add_input("color_image", self._on_color, Image)
        self.depth_image = None
        self.add_input("depth_image", self._on_depth, Image)

        # Définir les sorties
        self.piece_pose = sr.CartesianPose()
        self.add_output("piece_pose", "piece_pose", EncodedState)

        self.pcd_model = None
        self.initial_model_points = None
        self.add_predicate("is_piece_confirmed", False)
        self.add_predicate("is_piece_not_confirmed", False)
    
    def _on_depth_info(self, msg: CameraInfo):
        self.depth_camera_info = msg
    
    def _on_depth(self, msg: Image):
        self.depth_image = self.cv_bridge.imgmsg_to_cv2(msg, desired_encoding="passthrough")
    
    def _on_color(self, msg: Image):
        self.color_image = self.cv_bridge.imgmsg_to_cv2(msg, desired_encoding="passthrough")

    def on_configure_callback(self) -> bool:
        """Initialisation de la caméra et du modèle 3D."""
        try:
            self._voxel_size = self.get_parameter_value("voxel_size")
            self._min_depth = self.get_parameter_value("min_depth")
            self._max_depth = self.get_parameter_value("max_depth")
            self._target_color = np.array(self.get_parameter_value("color_to_filter")) / 255.0
            self._positive_tolerance = np.array(self.get_parameter_value("positive_tolerance")) / 255.0
            self._negative_tolerance = np.array(self.get_parameter_value("negative_tolerance")) / 255.0

            # Charger le modèle STL
            mesh_model = o3d.io.read_triangle_mesh(self.get_parameter_value("piece_path"))
            mesh_model.scale(0.001, center=mesh_model.get_center())
            raw_pcd_model = mesh_model.sample_points_uniformly(self.get_parameter_value("number_of_points"))
            raw_pcd_model.estimate_normals()

            model_points = np.asarray(raw_pcd_model.points)
            z_min = np.min(model_points[:, 2])
            tolerance_z = 0.0001
            filtered_model_points = model_points[np.abs(model_points[:, 2] - z_min) <= tolerance_z]

            self.pcd_model = o3d.geometry.PointCloud()
            self.pcd_model.points = o3d.utility.Vector3dVector(filtered_model_points)
            self.pcd_model.estimate_normals()

            rotation_matrix = o3d.geometry.get_rotation_matrix_from_axis_angle([0, 0, np.pi])
            self.pcd_model.rotate(rotation_matrix, center=self.pcd_model.get_center())
            self.initial_model_points = np.asarray(self.pcd_model.points).copy()
            return True
        except Exception as e:
            self.get_logger().error(f"Erreur lors de la configuration : {e}")
            return False

    def on_step_callback(self) -> None:
        """Traitement principal pour détecter la pièce."""
        try:
            if self.depth_camera_info is None or self.depth_image is None or self.color_image is None:
                self.get_logger().info("Didn't receive all images yet", throttle_duration_sec=1.0)
                return

            color_image = copy(self.color_image)
            depth_image = copy(self.depth_image)
            self.color_image = None
            self.depth_image = None

            depth_image = np.where(
                (depth_image > self._min_depth) & (depth_image < self._max_depth),
                depth_image,
                0,
            )
            k = self.depth_camera_info.k
            intrinsic_o3d = o3d.camera.PinholeCameraIntrinsic(
                self.depth_camera_info.width, self.depth_camera_info.height, k[0], k[4], k[2], k[5]
            )
            depth_o3d = o3d.geometry.Image(depth_image)
            color_o3d = o3d.geometry.Image(color_image)
            rgbd_image = o3d.geometry.RGBDImage.create_from_color_and_depth(
                color_o3d, depth_o3d, depth_scale=1000.0, depth_trunc=3.0, convert_rgb_to_intensity=False
            )
            pcd = o3d.geometry.PointCloud.create_from_rgbd_image(rgbd_image, intrinsic_o3d)

            # Filtrer les couleurs
            colors = np.asarray(pcd.colors)
            mask = np.all(
                (colors >= (self._target_color - self._negative_tolerance))
                & (colors <= (self._target_color + self._positive_tolerance)),
                axis=1,
            )
            pcd_filtered = pcd.select_by_index(np.where(mask)[0])

            # Clustering pour détecter la pièce
            points = np.asarray(pcd_filtered.points)
            clustering = DBSCAN(eps=0.01, min_samples=10).fit(points)
            labels = clustering.labels_
            unique_labels, counts = np.unique(labels, return_counts=True)
            target_label = unique_labels[np.argmax(counts)]
            mask = labels == target_label
            piece_points = points[mask]
            if len(piece_points) > self._max_cluster_size:
                self.get_logger().warning(f"Cluster trop gros : {len(piece_points)} points. Détection ignorée.")
                self.set_predicate("is_piece_confirmed", False)
                self.set_predicate("is_piece_not_confirmed", True)
                return
            pcd_piece = o3d.geometry.PointCloud()
            pcd_piece.points = o3d.utility.Vector3dVector(piece_points)

            # Calculer la transformation
            self.pcd_model.points = o3d.utility.Vector3dVector(self.initial_model_points)
            average_position_piece = np.mean(piece_points, axis=0)
            model_points = np.asarray(self.pcd_model.points)
            average_position_model = np.mean(model_points, axis=0)
            translation = average_position_piece - average_position_model

            initial_transformation = np.eye(4)
            initial_transformation[:3, 3] = translation
            self.pcd_model.transform(initial_transformation)
            self.pcd_model = self.pcd_model.voxel_down_sample(self._voxel_size)
            pcd_piece = pcd_piece.voxel_down_sample(self._voxel_size)

            # Recherche des meilleures rotations et calcul du fit
            rot_Y, fit_Y = self.find_best_rotation(self.pcd_model, pcd_piece, axis="y", angle_range=(-30, 35))
            rot_X, fit_X = self.find_best_rotation(self.pcd_model, pcd_piece, axis="x", angle_range=(-30, 35))
            rot_Z, fit_Z = self.find_best_rotation(self.pcd_model, pcd_piece, axis="z", angle_range=(-45, 50))
            best_rotation_matrix = rot_X @ rot_Y @ rot_Z

            fit_mean = (fit_X + fit_Y + fit_Z) / 3
            fit_threshold = self.get_parameter_value("fit_threshold")
            self.set_predicate("is_piece_confirmed", bool(fit_mean > fit_threshold))
            self.set_predicate("is_piece_not_confirmed", bool(fit_mean <= fit_threshold))
            self.get_logger().info(f"Fit mean: {fit_mean:.1f}% (threshold: {fit_threshold}%)")

            # Transformation globale
            global_transformation = np.eye(4)
            translation_matrix = np.eye(4)
            translation_matrix[:3, 3] = average_position_piece
            global_transformation = global_transformation @ translation_matrix
            rotation_matrix_4x4 = np.eye(4)
            rotation_matrix_4x4[:3, :3] = best_rotation_matrix
            global_transformation = global_transformation @ rotation_matrix_4x4

            pose = self.get_parameter_value("camera_pose_world")
            camera_orientation_world = R.from_quat([pose[4], pose[5], pose[6], pose[3]])
            camera_orientation_matrix = camera_orientation_world.as_matrix()
            camera_transformation_world = np.eye(4)
            camera_transformation_world[:3, :3] = camera_orientation_matrix
            camera_transformation_world[:3, 3] = pose[:3]

            piece_transformation_world = camera_transformation_world @ global_transformation

            piece_position_world = piece_transformation_world[:3, 3]
            piece_orientation_world = piece_transformation_world[:3, :3]
            piece_quaternion = R.from_matrix(piece_orientation_world).as_quat()
            piece_quaternion = [piece_quaternion[3], piece_quaternion[0], piece_quaternion[1], piece_quaternion[2]]

            self.piece_pose = sr.CartesianPose("piece", "world")
            self.piece_pose.set_position(piece_position_world)
            self.piece_pose.set_orientation(piece_quaternion)
            self.get_logger().info(f"pose: {self.piece_pose}")

        except Exception as e:
             self.get_logger().error(f"Erreur dans on_step_callback : {e}")

    # Method to find the best rotation angle with fit percentage
    def find_best_rotation(self, pcd_model, pcd_piece, axis, angle_range, angle_step=5, fit_tolerance=0.01):
        def calculate_average_distance(source_points, target_points):
            tree = cKDTree(target_points)
            distances, _ = tree.query(source_points, k=1)
            return np.mean(distances), distances

        initial_model_points = np.asarray(pcd_model.points).copy()
        center_of_model = np.mean(initial_model_points, axis=0)
        best_angle = None
        min_distance = float("inf")
        best_distances = None
        angles = np.arange(*angle_range, angle_step)

        for angle in angles:
            pcd_model.points = o3d.utility.Vector3dVector(initial_model_points)
            angle_rad = np.radians(angle)
            rotation_vector = [
                angle_rad if axis == "x" else 0,
                angle_rad if axis == "y" else 0,
                angle_rad if axis == "z" else 0,
            ]
            rotation_matrix = o3d.geometry.get_rotation_matrix_from_axis_angle(rotation_vector)
            pcd_model.rotate(rotation_matrix, center=center_of_model)
            avg_distance, distances = calculate_average_distance(
                np.asarray(pcd_model.points), np.asarray(pcd_piece.points)
            )
            if avg_distance < min_distance:
                min_distance = avg_distance
                best_angle = angle
                best_distances = distances

        pcd_model.points = o3d.utility.Vector3dVector(initial_model_points)
        angle_rad = np.radians(best_angle)
        best_rotation_vector = [
            angle_rad if axis == "x" else 0,
            angle_rad if axis == "y" else 0,
            angle_rad if axis == "z" else 0,
        ]
        best_rotation_matrix = o3d.geometry.get_rotation_matrix_from_axis_angle(best_rotation_vector)
        pcd_model.rotate(best_rotation_matrix, center=center_of_model)

        # Calcul du pourcentage de fit
        fit_percent = 0.0
        if best_distances is not None and len(best_distances) > 0:
            fit_percent = np.sum(best_distances < fit_tolerance) / len(best_distances) * 100
        return best_rotation_matrix, fit_percent