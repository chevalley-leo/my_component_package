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
        "min_depth": 150,
        "max_depth": 700
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
        "y_range": [-30, 35],
        "x_range": [-30, 35],
        "z_range": [0, 360]
    }
}