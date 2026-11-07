import rev
import phoenix6
import wpimath.controller
import math

class SwerveModule():

    # Type = Mk4 L1
    # Drive ratio: 8.14:1
    # Steer ratio: 12.8:1

    def __init__(self, drive_motor, steer_motor, encoder, front, left, encoder_offset):
        self.drive_motor = rev.SparkMax(drive_motor)
        self.steer_motor = rev.SparkMax(steer_motor)
        self.encoder = phoenix6.hardware.CANcoder(encoder)

        self.front = front
        self.left = left
        self.encoder_offset = encoder_offset

        kp = 0.1
        kd = 0.001

        self.pid = wpimath.controller.PIDController(kp, 0.0, kd)
        self.pid.enableContinuousInput(-math.pi, math.pi)

        config = rev.SparkMaxConfig()
        config.setIdleMode(rev.SparkBaseConfig.IdleMode.kBrake)
        
        self.steer_motor.configure(config, rev.ResetMode.kResetSafeParameters, rev.PersistMode.kPersistParameters)
        self.drive_motor.configure(config, rev.ResetMode.kResetSafeParameters, rev.PersistMode.kPersistParameters)
    
    def set_drive(self, percent_speed):
        self.drive_motor.set(percent_speed)
    
    def set_steer(self, angle):
        current_angle = self.encoder.get_absolute_position().value - self.encoder_offset

        motor_speed = self.pid.calculate(current_angle, angle)

        self.steer_motor.set(motor_speed)