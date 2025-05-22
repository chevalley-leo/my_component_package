{
    "model_file": "piece.stl",
    "scale": 0.001,
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
    "dbscan": {
        "eps": 0.01,
        "min_samples": 10
    },
    "rotation": {
        "angle_step": 5,
        "y_angle_range": [-30, 35],
        "x_angle_range": [-30, 35],
        "z_angle_range": [0, 360]
    }
}