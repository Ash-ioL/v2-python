import math
from constants import DEADBAND, MAX_SPEED, MAX_ROT

def get_translation_velocities(x, y):
    x, y = apply_deadband(x, y)
    vx = x * MAX_SPEED
    vy = y * MAX_SPEED
    return vx, vy

def get_rotation_velocities(x):
    x, _ = apply_deadband(x, 0)
    return MAX_ROT * x

def apply_deadband(x, y):
    h = math.hypot(x, y)
    if h < DEADBAND:
        return 0, 0
    if h > 1:
        fac = 1 / h
        x *= fac
        y *= fac
    return x, y