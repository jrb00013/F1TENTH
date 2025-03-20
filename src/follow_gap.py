def avoid_obstacles(vehicle_position, obstacle_positions):
    for obstacle in obstacle_positions:
        if abs(vehicle_position[0] - obstacle[0]) < 1.0 and abs(vehicle_position[1] - obstacle[1]) < 1.0:
            # Move around the obstacle
            return "Turn Left"
    return "Continue Straight"
