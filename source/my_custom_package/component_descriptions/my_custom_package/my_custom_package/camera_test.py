{
    "model_file": "piece.stl",
    "model_scale": 0.001,
    "sample_points": 5000,
    "z_min_tolerance": 0.0001,
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
        },
        "depth_trunc": 3.0,
        "depth_scale": 1000.0
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