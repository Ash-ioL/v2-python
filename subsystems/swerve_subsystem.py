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
from subsystems.swerve_module import SwerveModule
import rev
import commands2


class SwerveSubsystem(commands2.SubsystemBase):
    def __init__(self, front_left, front_right, back_left, back_right, gyro: WPI_PigeonIMU):
        self.inc = 0
        super().__init__()
        
        self.fl_module: SwerveModule = front_left
        self.fr_module: SwerveModule = front_right
        self.bl_module: SwerveModule = back_left
        self.br_module: SwerveModule = back_right

        self.modules = [
            self.fl_module,
            self.fr_module,
            self.bl_module,
            self.br_module
        ]

        self.kinematics = SwerveDrive4Kinematics(
            self.fl_module.get_offset(),
            self.fr_module.get_offset(),
            self.bl_module.get_offset(),
            self.br_module.get_offset(),
        )

        self.gyro = gyro

    def from_field_relative_speeds(self, x_speed, y_speed, rot_speed):
        chassis_speeds = ChassisSpeeds.fromFieldRelativeSpeeds(
            x_speed, y_speed, rot_speed, self.gyro.getRotation2d()
        )
        
        module_states = self.kinematics.toSwerveModuleStates(chassis_speeds)

        return module_states

    def set_states(self, vx, vy, omega):
        speeds = ChassisSpeeds.fromFieldRelativeSpeeds(
            vx, vy, omega,
            self.gyro.getRotation2d()
        )

        module_states = self.kinematics.toSwerveModuleStates(speeds)
        for i in range(len(module_states)):
            state = module_states[i]

            module = self.modules[i]
            module.set_state(state)
    
    def run_all(self):
        for module in self.modules:
            module.run_light()

    def periodic(self):
        self.fl_module.log_module()
        self.fr_module.log_module()
        self.bl_module.log_module()
        self.br_module.log_module()
        SmartDashboard.putNumber("swerve/yaw", self.gyro.getYaw())
        SmartDashboard.putNumber("random", self.inc)
        self.inc += 0.001