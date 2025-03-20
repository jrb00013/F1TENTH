import numpy as np

def update_map(vehicle_position, map_data):
    map_data.append(vehicle_position)
    return map_data
