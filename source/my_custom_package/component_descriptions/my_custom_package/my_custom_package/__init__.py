{
    "model_file": "piece.stl",
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
    },
    "depth_filter": {
        "min_distance": 150,
        "max_distance": 700
    },
    "color_filter": {
        "target_color": [128, 96, 49],
        "tolerance_positive": [20, 20, 20],
        "tolerance_negative": [30, 30, 30]
    },
    "rotation_angles": {
        "x": [-30, 35],
        "y": [-30, 35],
        "z": [0, 360]
    },
    "voxel_size": 0.001
}