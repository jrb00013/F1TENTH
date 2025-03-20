import numpy as np

def process_lidar_data(lidar_data):
    # Process and filter lidar data
    return np.array([data for data in lidar_data if data['distance'] < 5.0])
