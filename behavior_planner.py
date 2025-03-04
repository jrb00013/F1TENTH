def behavior_planner(state):
    if state == "ObstacleDetected":
        return "AvoidObstacle"
    elif state == "ClearPath":
        return "Proceed"
    return "Idle"
