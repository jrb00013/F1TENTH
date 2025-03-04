class PIDController:
    def __init__(self, kp, ki, kd):
        self.kp = kp
        self.ki = ki
        self.kd = kd
        self.prev_error = 0
        self.integral = 0

    def compute(self, error):
        self.integral += error
        derivative = error - self.prev_error
        self.prev_error = error
        return self.kp * error + self.ki * self.integral + self.kd * derivative

def follow_wall(sensor_distance):
    pid = PIDController(1.0, 0.1, 0.05)
    error = sensor_distance - 1.0  # Assuming 1 meter distance to the wall
    steering_correction = pid.compute(error)
    return steering_correction
