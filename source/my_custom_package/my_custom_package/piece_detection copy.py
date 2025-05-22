from modulo_components.component import Component
import open3d as o3d
import pyrealsense2 as rs
import numpy as np
import cv2
from scipy.spatial.transform import Rotation as R
from sklearn.cluster import DBSCAN
from scipy.spatial import cKDTree

class PieceDetectionComponent(Component):
    def __init__(self, node_name: str, *args, **kwargs):
        super().__init__(node_name, *args, **kwargs)

        # Définir les paramètres
        self.add_parameter("piece_path", "Path to the STL file", "piece.stl")
        self.add_parameter("number_of_points", "Number of points to sample from the mesh", 5000)
        self.add_parameter("voxel_size", "Voxel size for downsampling", 0.001)
        self.add_parameter("min_depth", "Minimum depth for filtering", 150)
        self.add_parameter("max_depth", "Maximum depth for filtering", 700)
        self.add_parameter("color_to_filter", "Color to filter in RGB format", [128, 96, 49])
        self.add_parameter("positive_tolerance", "Tolerance for positive filtering", [20, 20, 20])
        self.add_parameter("negative_tolerance", "Tolerance for negative filtering", [30, 30, 30])
        self.add_parameter("camera_position_world", "Camera position in world coordinates", [0.42104, 0, 0.78244])
        self.add_parameter("camera_quaternion", "Camera orientation in quaternion format", [0.0, 0.707107, -0.707107, 0])

        # Définir les sorties
        self.add_output("piece_position", "Position of the detected piece", np.ndarray)
        self.add_output("piece_orientation", "Orientation of the detected piece", np.ndarray)

        # Initialiser les variables
        self.pipeline = None
        self.align = None
        self.spatial = None
        self.temporal = None
        self.pcd_model = None
        self.initial_model_points = None

    def on_configure_callback(self) -> bool:
        """Initialisation de la caméra et du modèle 3D."""
        try:
            # Charger le modèle STL
            piece_path = self.get_parameter_value("piece_path")
            mesh = o3d.io.read_triangle_mesh(piece_path)
            mesh.scale(0.001, center=mesh.get_center())
            self.pcd_model = mesh.sample_points_uniformly(5000)
            self.pcd_model.estimate_normals()

            self.model_points = np.asarray(self.pcd_model.points)
            z_min = np.min(self.model_points[:, 2])
            tolerance_z = 0.0001
            filtered_model_points = self.model_points[np.abs(self.model_points[:, 2] - z_min) <= tolerance_z]

            self.pcd_model = o3d.geometry.PointCloud()
            self.pcd_model.points = o3d.utility.Vector3dVector(filtered_model_points)
            self.pcd_model.estimate_normals()

            rotation_matrix = o3d.geometry.get_rotation_matrix_from_axis_angle([0, 0, np.pi])
            self.pcd_model.rotate(rotation_matrix, center=self.pcd_model.get_center())

            # Initialiser la caméra RealSense
            self.pipeline = rs.pipeline()
            config = rs.config()
            config.enable_stream(rs.stream.depth, 1280, 720, rs.format.z16, 30)
            config.enable_stream(rs.stream.color, 1280, 720, rs.format.bgr8, 30)
            self.pipeline.start(config)
            self.align = rs.align(rs.stream.color)
            self.spatial = rs.spatial_filter()
            self.temporal = rs.temporal_filter()

            self.initial_model_points = np.asarray(self.pcd_model.points).copy()
            return True
        except Exception as e:
            self.get_logger().error(f"Erreur lors de la configuration : {e}")
            return False

    def on_step_callback(self) -> bool:
        """Traitement principal pour détecter la pièce."""
        try:
            # Obtenir les frames de la caméra
            frames = self.pipeline.wait_for_frames()
            aligned_frames = self.align.process(frames)
            depth_frame = aligned_frames.get_depth_frame()
            color_frame = aligned_frames.get_color_frame()

            if not depth_frame or not color_frame:
                return False

            # Appliquer les filtres spatiaux et temporels
            depth_frame = self.spatial.process(depth_frame)
            depth_frame = self.temporal.process(depth_frame)

            # Convertir les frames en images
            color_image = np.asanyarray(color_frame.get_data())
            depth_image = np.asanyarray(depth_frame.get_data())
            depth_image = np.where(
                (depth_image > self.get_parameter_value("min_depth")) &
                (depth_image < self.get_parameter_value("max_depth")),
                depth_image, 0
            )
            color_image = cv2.cvtColor(color_image, cv2.COLOR_BGR2RGB)

            # Créer une image RGBD pour Open3D
            intr = depth_frame.profile.as_video_stream_profile().intrinsics
            intrinsic_o3d = o3d.camera.PinholeCameraIntrinsic(
                intr.width, intr.height, intr.fx, intr.fy, intr.ppx, intr.ppy
            )
            depth_o3d = o3d.geometry.Image(depth_image)
            color_o3d = o3d.geometry.Image(color_image)
            rgbd_image = o3d.geometry.RGBDImage.create_from_color_and_depth(
                color_o3d, depth_o3d, depth_scale=1000.0, depth_trunc=3.0, convert_rgb_to_intensity=False
            )
            pcd = o3d.geometry.PointCloud.create_from_rgbd_image(rgbd_image, intrinsic_o3d)

            # Filtrer les couleurs
            target_color = np.array(self.get_parameter_value("color_to_filter")) / 255.0
            tolerance_positive = np.array(self.get_parameter_value("positive_tolerance")) / 255.0
            tolerance_negative = np.array(self.get_parameter_value("negative_tolerance")) / 255.0
            colors = np.asarray(pcd.colors)
            mask = np.all(
                (colors >= (target_color - tolerance_negative)) &
                (colors <= (target_color + tolerance_positive)),
                axis=1
            )
            pcd_filtered = pcd.select_by_index(np.where(mask)[0])

            # Clustering pour détecter la pièce
            points = np.asarray(pcd_filtered.points)
            eps = 0.01
            min_points = 10
            clustering = DBSCAN(eps=eps, min_samples=min_points).fit(points)
            labels = clustering.labels_
            unique_labels, counts = np.unique(labels, return_counts=True)
            target_label = unique_labels[np.argmax(counts)]
            mask = labels == target_label
            piece_points = points[mask]
            pcd_piece = o3d.geometry.PointCloud()
            pcd_piece.points = o3d.utility.Vector3dVector(piece_points)

            # Calculer la transformation
            average_position_piece = np.mean(piece_points, axis=0)
            model_points = np.asarray(self.pcd_model.points)
            average_position_model = np.mean(model_points, axis=0)
            translation = average_position_piece - average_position_model

            initial_transformation = np.eye(4)
            initial_transformation[:3, 3] = translation
            self.pcd_model.transform(initial_transformation)

            # Réduction de la densité des nuages de points
            voxel_size = self.get_parameter_value("voxel_size")
            self.pcd_model = self.pcd_model.voxel_down_sample(voxel_size)
            pcd_piece = pcd_piece.voxel_down_sample(voxel_size)

            # Trouver la meilleure rotation
            rot_Y = find_best_rotation(self.pcd_model, pcd_piece, axis='y', angle_range=(-30, 35))
            rot_X = find_best_rotation(self.pcd_model, pcd_piece, axis='x', angle_range=(-30, 35))
            rot_Z = find_best_rotation(self.pcd_model, pcd_piece, axis='z', angle_range=(0, 360))
            best_rotation_matrix = rot_X @ rot_Y @ rot_Z

            # Calculer la transformation globale
            global_transformation = np.eye(4)
            translation_matrix = np.eye(4)
            translation_matrix[:3, 3] = average_position_piece
            global_transformation = global_transformation @ translation_matrix
            rotation_matrix_4x4 = np.eye(4)
            rotation_matrix_4x4[:3, :3] = best_rotation_matrix
            global_transformation = global_transformation @ rotation_matrix_4x4

            # Transformation de la caméra
            camera_position_world = np.array(self.get_parameter_value("camera_position_world"))
            camera_quaternion = np.array(self.get_parameter_value("camera_quaternion"))
            camera_orientation_world = R.from_quat([
                camera_quaternion[1], camera_quaternion[2], camera_quaternion[3], camera_quaternion[0]
            ])
            camera_orientation_matrix = camera_orientation_world.as_matrix()
            camera_transformation_world = np.eye(4)
            camera_transformation_world[:3, :3] = camera_orientation_matrix
            camera_transformation_world[:3, 3] = camera_position_world

            # Transformation finale
            piece_transformation_world = camera_transformation_world @ global_transformation
            rotation_90_x = np.eye(4)
            rotation_90_x[:3, :3] = R.from_euler('x', 90, degrees=True).as_matrix()
            piece_transformation_world = piece_transformation_world @ rotation_90_x

            # Extraire la position et l'orientation
            piece_position_world = piece_transformation_world[:3, 3]
            piece_orientation_world = piece_transformation_world[:3, :3]
            piece_quaternion = R.from_matrix(piece_orientation_world).as_quat()
            piece_quaternion = [piece_quaternion[3], piece_quaternion[0], piece_quaternion[1], piece_quaternion[2]]

            # Publier les résultats
            self.publish_output("piece_position", piece_position_world)
            self.publish_output("piece_orientation", piece_quaternion)

            return True
        except Exception as e:
            self.get_logger().error(f"Erreur dans on_step_callback : {e}")
            return False
        

    def on_cleanup_callback(self) -> bool:
        """Nettoyage des ressources."""
        if self.pipeline:
            self.pipeline.stop()
        return True   


# Method to find the best rotation angle
def find_best_rotation(pcd_model, pcd_piece, axis, angle_range, angle_step=3):
    def calculate_average_distance(source_points, target_points):
        tree = cKDTree(target_points)
        distances, _ = tree.query(source_points, k=1)
        return np.mean(distances)

    initial_model_points = np.asarray(pcd_model.points).copy()
    center_of_model = np.mean(initial_model_points, axis=0)
    best_angle = None
    min_distance = float('inf')
    angles = np.arange(*angle_range, angle_step)

    for angle in angles:
        pcd_model.points = o3d.utility.Vector3dVector(initial_model_points)
        angle_rad = np.radians(angle)
        rotation_vector = [angle_rad if axis == 'x' else 0, angle_rad if axis == 'y' else 0, angle_rad if axis == 'z' else 0]
        rotation_matrix = o3d.geometry.get_rotation_matrix_from_axis_angle(rotation_vector)
        pcd_model.rotate(rotation_matrix, center=center_of_model)
        avg_distance = calculate_average_distance(np.asarray(pcd_model.points), np.asarray(pcd_piece.points))
        if avg_distance < min_distance:
            min_distance = avg_distance
            best_angle = angle

    pcd_model.points = o3d.utility.Vector3dVector(initial_model_points)
    angle_rad = np.radians(best_angle)
    best_rotation_vector = [angle_rad if axis == 'x' else 0, angle_rad if axis == 'y' else 0, angle_rad if axis == 'z' else 0]
    best_rotation_matrix = o3d.geometry.get_rotation_matrix_from_axis_angle(best_rotation_vector)
    pcd_model.rotate(best_rotation_matrix, center=center_of_model)
    return best_rotation_matrix


