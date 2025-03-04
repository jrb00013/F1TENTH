import random

def rrt(start, goal, obstacles):
    # Simple RRT algorithm
    path = [start]
    current_position = start
    
    while distance(current_position, goal) > 1.0:
        random_point = random.choice(obstacles)  # Simulate random sampling
        path.append(random_point)
        current_position = random_point
        
    return path
