import rev
import phoenix6
from wpilib import SmartDashboard
import wpimath.controller
from wpimath.geometry import Rotation2d
import math
import math
import phoenix6
import wpilib
from wpimath.geometry import Translation2d;
from wpimath.kinematics import SwerveDrive4Kinematics, ChassisSpeeds, SwerveModuleState
import commands2
from commands2.button import CommandXboxController
import util.drive_deadband as db
from phoenix5.sensors import WPI_PigeonIMU
import constants
import phoenix6
from wpilib import SmartDashboard
import rev


class SwerveModule():

    # Type = Mk4 L1
    # Drive ratio: 8.14:1
    # Steer ratio: 12.8:1

    def __init__(self, drive_motor, steer_motor, encoder, offset, encoder_offset):
        self.drive_motor = rev.SparkMax(drive_motor, rev.SparkMax.MotorType.kBrushless)
        self.steer_motor = rev.SparkMax(steer_motor, rev.SparkMax.MotorType.kBrushless)
        self.encoder = phoenix6.hardware.CANcoder(encoder)

        self.front = offset.X()
        self.left = offset.Y()
        self.offset = offset
        self.encoder_offset = encoder_offset

        kp = 0.1
        kd = 0.001

        self.steer_pid = wpimath.controller.PIDController(kp, 0.0, kd)
        self.steer_pid.enableContinuousInput(-math.pi, math.pi)

        self.drive_pid = self.drive_motor.getClosedLoopController()
        # self.steer_pid = self.steer_motor.getClosedLoopController()

        steer_config = rev.SparkMaxConfig()
        drive_config = rev.SparkMaxConfig()

        # steer_config.closedLoop.positionWrappingEnabled(True)
        # steer_config.closedLoop.positionWrappingMinInput(-math.pi)
        # steer_config.closedLoop.positionWrappingMaxInput(math.pi)
        # steer_config.closedLoop.P(2.5)
        steer_config.setIdleMode(rev.SparkBaseConfig.IdleMode.kBrake)

        drive_config.closedLoop.P(0.1)
        drive_config.closedLoop.I(0.0)
        drive_config.closedLoop.D(0.0)
        drive_config.closedLoop.velocityFF(0.2)
        drive_config.setIdleMode(rev.SparkBaseConfig.IdleMode.kBrake)

        self.steer_motor.configure(steer_config, rev.ResetMode.kResetSafeParameters, rev.PersistMode.kPersistParameters)
        self.drive_motor.configure(drive_config, rev.ResetMode.kResetSafeParameters, rev.PersistMode.kPersistParameters)

    
    def set_drive(self, vel):
        vel = min(vel, constants.MAX_SPEED)
        rpm = (vel * 60 * constants.SwerveModules.DRIVE_RATIO) / (constants.SwerveModules.WHEEL_RADIUS * 2 * math.pi)

        self.drive_pid.setReference(rpm, rev.SparkMaxControlType.kVelocity)
    
    def set_steer(self, angle):
        angle = Rotation2d(angle).radians()
        current_angle = self.encoder.get_absolute_position().value * 2 * math.pi - self.encoder_offset
        SmartDashboard.putNumber("expected swerve angle", angle)
        SmartDashboard.putNumber("current swerve angle", current_angle)

        output = self.steer_pid.calculate(current_angle, angle)
        output = min(1, output)
        self.steer_motor.set(output)
    
    def set_state(self, state):
        if state is None:
            return
        self.set_drive(state.speed)
        self.set_steer(state.angle.radians())

    def get_offset(self):
        return self.offset
    
    def get_steer_angle(self):
        return self.encoder.get_absolute_position().value * 2 * math.pi - self.encoder_offset

    def get_speed(self):
        return (self.drive_motor.getEncoder().getVelocity() * 2 * math.pi * constants.SwerveModules.WHEEL_RADIUS) / (60 * constants.SwerveModules.DRIVE_RATIO)

    def get_state(self):
        return SwerveModuleState(self.get_speed(), wpimath.geometry.Rotation2d(self.get_steer_angle()))