import math

deadband = 0.15
max_speed = 3 # m/s
max_rot = 2 # rps

def get_translation_velocities(x, y):
    x, y = apply_deadband(x, y)
    vx = x * max_speed
    vy = y * max_speed
    return vx, vy

def get_rotation_velocities(x):
    x, _ = apply_deadband(x, 0)
    return max_rot * x

def apply_deadband(x, y):
    h = math.hypot(x, y)
    if h < deadband:
        return 0, 0
    if h > 1:
        fac = 1 / h
        x *= fac
        y *= fac
    return x, y