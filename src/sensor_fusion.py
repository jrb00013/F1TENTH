import numpy as np

def senso(lidar_data, camera_data):
    lidar_points = np.array([data['distance'] for data in lidar_data])
    camera_features = np.array([data['feature'] for data in camera_data])
    
    # Combine both sensor types' data
    fused_data = np.concatenate((lidar_points, camera_features), axis=0)
    return fused_data
