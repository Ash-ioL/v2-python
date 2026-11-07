#import rev
#import phoenix6

#class SwerveModule():

    # Type = Mk4 L1
    # Drive ratio: 8.14:1
    # Steer ratio: 12.8:1

    #def __init__(self, drive_motor, steer_motor, encoder, front, left, encoder_offset):
    #    self.drive_motor = rev.SparkMax(drive_motor)
    #    self.steer_motor = rev.SparkMax(steer_motor)
    #    self.encoder = phoenix6.hardware.CANcoder(encoder)

    #    self.front = front
    #    self.left = left
    #    self.encoder_offset = encoder_offset

import math
import rev
import phoenix6
from wpimath.geometry import Rotation2d
from wpimath.kinematics import SwerveModuleState

class SwerveModule:
    # SDS Mk4 L1 physical properties
    WHEEL_DIAMETER_METERS = 0.1016  # 4 inches
    DRIVE_GEAR_RATIO = 8.14
    STEER_GEAR_RATIO = 12.8

    def __init__(self, drive_id: int, steer_id: int, cancoder_id: int, encoder_offset: float):
        # 1. Initialize Hardware Components
        self.drive_motor = rev.SparkMax(drive_id, rev.SparkLowLevel.MotorType.kBrushless)
        self.steer_motor = rev.SparkMax(steer_id, rev.SparkLowLevel.MotorType.kBrushless)
        self.cancoder = phoenix6.hardware.CANcoder(cancoder_id)
        
        self.encoder_offset = encoder_offset

        # 2. Extract Built-in Closed Loop Controllers
        self.drive_pid = self.drive_motor.getClosedLoopController()
        self.steer_pid = self.steer_motor.getClosedLoopController()

        self.drive_encoder = self.drive_motor.getEncoder()
        self.steer_encoder = self.steer_motor.getEncoder()

        # 3. Apply Local Hardware Configuration
        self._configure_hardware()

    def _configure_hardware(self):
        """Configures conversion metrics and brake modes locally for this module."""
        # Drive Motor Settings
        drive_config = rev.SparkMaxConfig()
        drive_config.setIdleMode(rev.SparkBaseConfig.IdleMode.kBrake)
        
        # Calculate drive translation factor (Motor Rotations -> Wheel Meters / sec)
        drive_factor = (self.WHEEL_DIAMETER_METERS * math.pi) / self.DRIVE_GEAR_RATIO
        drive_config.encoder.positionConversionFactor(drive_factor)
        drive_config.encoder.velocityConversionFactor(drive_factor / 60.0)
        
        # Apply drive loop PID settings (Placeholders—tune these to your weight!)
        drive_config.closedLoop.P(0.1).I(0.0).D(0.0)
        self.drive_motor.configure(drive_config, rev.ResetMode.kResetSafeParameters, rev.PersistMode.kPersistParameters)

        # Steering Motor Settings
        steer_config = rev.SparkMaxConfig()
        steer_config.setIdleMode(rev.SparkBaseConfig.IdleMode.kBrake)
        
        # Calculate steer factor (Motor Rotations -> Radians)
        steer_factor = (2 * math.pi) / self.STEER_GEAR_RATIO
        steer_config.encoder.positionConversionFactor(steer_factor)
        
        # Apply steer loop PID settings (Steer loops require a higher P value than drive loops)
        steer_config.closedLoop.P(1.5).I(0.0).D(0.0)
        self.steer_motor.configure(steer_config, rev.ResetMode.kResetSafeParameters, rev.PersistMode.kPersistParameters)

    def get_absolute_heading(self) -> Rotation2d:
        """Returns the module angle calculated cleanly using a Rotation2d wrapper."""
        cancoder_pos = self.cancoder.get_absolute_position().value
        adjusted_rotations = cancoder_pos - self.encoder_offset
        return Rotation2d(adjusted_rotations * 2 * math.pi)

    def set_desired_state(self, desired_state: SwerveModuleState):
        """Accepts a target command, runs self-optimization calculations, and sets inputs."""
        current_heading = self.get_absolute_heading()
        optimized_state = SwerveModuleState.optimize(desired_state, current_heading)

        # Because we configured conversion factors, we can pass native units (m/s and Radians)
        self.drive_pid.setReference(optimized_state.speed, rev.SparkMax.ControlType.kVelocity)
        self.steer_pid.setReference(optimized_state.angle.radians(), rev.SparkMax.ControlType.kPosition)

