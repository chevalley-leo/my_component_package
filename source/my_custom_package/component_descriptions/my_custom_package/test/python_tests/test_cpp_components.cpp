{
    "model_file": "piece.stl",
    "camera": {
        "depth_stream": {
            "width": 1280,
            "height": 720,
            "format": "z16",
            "fps": 30
        },
        "color_stream": {
            "width": 1280,
            "height": 720,
            "format": "bgr8",
            "fps": 30
        }
    },
    "filter": {
        "depth_range": {
            "min": 150,
            "max": 700
        },
        "target_color": [128, 96, 49],
        "tolerance_positive": [20, 20, 20],
        "tolerance_negative": [30, 30, 30]
    },
    "rotation": {
        "angle_step": 5,
        "y_angle_range": [-30, 35],
        "x_angle_range": [-30, 35],
        "z_angle_range": [0, 360]
    },
    "voxel_size": 0.001
}