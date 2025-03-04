import math

def check_collision(vehicle_position, obstacle_position, threshold=1.5):
    distance = math.sqrt((vehicle_position[0] - obstacle_position[0])**2 + (vehicle_position[1] - obstacle_position[1])**2)
    
    if distance < threshold:
        return True
    return False
