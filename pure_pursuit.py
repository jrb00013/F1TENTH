def pure_pursuit(vehicle_position, target_position):
    angle_to_target = math.atan2(target_position[1] - vehicle_position[1], target_position[0] - vehicle_position[0])
    return angle_to_target
