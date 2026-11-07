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
    def __init__(self, front_left, front_right, back_left, back_right):
        super().__init__()
        
        self.fl_module = front_left
        self.fr_module = front_right
        self.bl_module = back_left
        self.br_module = back_right

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

        self.gyro = WPI_PigeonIMU(constants.PIGEON_ID)

    def from_field_relative_speeds(self, x_speed, y_speed, rot_speed):
        chassis_speeds = ChassisSpeeds.fromFieldRelativeSpeeds(
            x_speed, y_speed, rot_speed, self.gyro.getRotation2d()
        )
        
        module_states = self.kinematics.toSwerveModuleStates(chassis_speeds)

        return module_states

    def set_states(self, module_states):
        for i in range(len(module_states)):
            state = module_states[i]
            state = SwerveModuleState.optimize(state, Rotation2d(self.modules[i].get_steer_angle()))

            module = self.modules[i]
            module.set_state(state)

        