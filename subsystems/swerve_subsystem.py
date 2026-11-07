# import commands2


# class SwerveSubsystem(commands2.SubsystemBase):
#     def __init__(self, front_left, front_right, back_left, back_right, gyro):
#         super().__init__()
#         self.front_left_location = Translation2d()
#         self.front_right_location = Translation2d()
#         self.back_left_location = Translation2d()
#         self.back_right_location = Translation2d()

#         self.controller = CommandXboxController(0)

#         self.kinematics = SwerveDrive4Kinematics(
#             self.front_left_location,
#             self.front_right_location,
#             self.back_left_location,
#             self.back_right_location,
#         )

#         self.gyro = WPI_PigeonIMU(constants.pigeon_id)
import math
import wpilib
from wpilib import SmartDashboard
import commands2
from wpimath.geometry import Translation2d, Rotation2d
from wpimath.kinematics import SwerveDrive4Kinematics, ChassisSpeeds
from phoenix5.sensors import WPI_PigeonIMU

import constants
import util.drive_deadband as db
from .swerve_module import SwerveModule  # Assumes your module class is in the same directory

class SwerveSubsystem(commands2.Subsystem):
    def __init__(self):
        super().__init__()

        # 1. Define physical module offsets from center of robot (e.g., 0.3 meters or ~12 inches)
        # Adjust these values based on your physical robot wheelbase/trackwidth dimensions
        self.front_left_location  = Translation2d(0.3, 0.3)
        self.front_right_location = Translation2d(0.3, -0.3)
        self.back_left_location   = Translation2d(-0.3, 0.3)
        self.back_right_location  = Translation2d(-0.3, -0.3)

        # 2. Kinematics Pipeline Container
        self.kinematics = SwerveDrive4Kinematics(
            self.front_left_location,
            self.front_right_location,
            self.back_left_location,
            self.back_right_location,
        )

        # 3. Gyroscope Sensor Container
        self.gyro = WPI_PigeonIMU(constants.PIGEON_ID)

        # 4. Instantiate 4 separate, self-managed Swerve Module objects
        # Arguments: (drive_id, steer_id, cancoder_id, angle_offset_rotations)
        self.front_left  = SwerveModule(11, 12, 13, encoder_offset=0.0)
        self.front_right = SwerveModule(21, 22, 23, encoder_offset=0.0)
        self.back_left   = SwerveModule(31, 32, 33, encoder_offset=0.0)
        self.back_right  = SwerveModule(41, 42, 43, encoder_offset=0.0)
        
        self.modules = [self.front_left, self.front_right, self.back_left, self.back_right]

    def drive(self, x: float, y: float, r_x: float, field_relative: bool = True):
        """High-level abstraction called by the controller to drive the robot."""
        # Cleanly offload translation/rotation calculation to your utility module
        vx, vy = db.get_translation_velocities(x, y)
        omega = db.get_rotation_velocities(r_x)

        # Calculate speeds relative to either the field or the chassis frame
        if field_relative:
            speeds = ChassisSpeeds.fromFieldRelativeSpeeds(vx, vy, omega, self.gyro.getRotation2d())
        else:
            speeds = ChassisSpeeds(vx, vy, omega)

        # Convert to target states and safeguard motor over-saturation limits
        module_states = self.kinematics.toSwerveModuleStates(speeds)
        self.kinematics.desaturateWheelSpeeds(module_states, db.max_speed)

        # Direct, index-matched assignment delegating goals out to individual objects
        self.front_left.set_desired_state(module_states[0])
        self.front_right.set_desired_state(module_states[1])
        self.back_left.set_desired_state(module_states[2])
        self.back_right.set_desired_state(module_states[3])

    def periodic(self):
        """Runs every 20ms automatically via the command scheduler."""
        SmartDashboard.putString("Gyro Yaw", str(self.gyro.getYaw()))
        SmartDashboard.putNumber("FL Cancoder", self.front_left.get_absolute_heading().degrees())
