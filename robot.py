import phoenix6
import wpilib
from wpimath.geometry import Translation2d;
from wpimath.kinematics import SwerveDrive4Kinematics, ChassisSpeeds, SwerveModuleState
import commands2
from commands2.button import CommandXboxController
import util.drive_deadband as db
from phoenix5.sensors import WPI_PigeonIMU
import constants
from wpilib import SmartDashboard

class Robot(wpilib.TimedRobot):
    def robotInit(self):
        self.front_left_location = Translation2d()
        self.front_right_location = Translation2d()
        self.back_left_location = Translation2d()
        self.back_right_location = Translation2d()

        self.controller = CommandXboxController(0)

        self.kinematics = SwerveDrive4Kinematics(
            self.front_left_location,
            self.front_right_location,
            self.back_left_location,
            self.back_right_location,
        )

        self.gyro = WPI_PigeonIMU(constants.PIGEON_ID)

    def teleopPeriodic(self):
        x = self.controller.getLeftX()
        y = self.controller.getLeftY()

        r_x = self.controller.getRightX()

        vx, vy = db.get_translation_velocities(x, y)

        omega = db.get_rotation_velocities(r_x)

        speeds = ChassisSpeeds.fromFieldRelativeSpeeds(
            vx, vy, omega,
            self.gyro.getRotation2d()
        )

        module_states = self.kinematics.toSwerveModuleStates(speeds)

        self.kinematics.desaturateWheelSpeeds(module_states, db.max_speed)

        fl_states = module_states[0]
        fr_states = module_states[1]
        bl_states = module_states[2]
        br_states = module_states[3]

        # fl_states = SwerveModuleState.optimize(fl_states, self.front_left.get_state().angle)
        # fr_states = SwerveModuleState.optimize(fr_states, self.front_right.get_state().angle)
        # bl_states = SwerveModuleState.optimize(bl_states, self.back_left.get_state().angle)
        # br_states = SwerveModuleState.optimize(br_states, self.back_right.get_state().angle)

        
        # set states ot the swerve mdouels
        SmartDashboard.putString("test", str(self.gyro.getYaw()))

        encoders = [
            phoenix6.hardware.CANcoder(constants.SwerveModules.FrontLeft.ENCODER_ID),
            phoenix6.hardware.CANcoder(constants.SwerveModules.FrontRight.ENCODER_ID),
            phoenix6.hardware.CANcoder(constants.SwerveModules.BackLeft.ENCODER_ID),
            phoenix6.hardware.CANcoder(constants.SwerveModules.BackRight.ENCODER_ID),
        ]

        for encoder in encoders:
            config = phoenix6.configs.CANcoderConfiguration()
            config.magnet_sensor.magnet_offset = 0.0
            encoder.configurator.apply(config)

    def robotPeriodic(self):
        commands2.CommandScheduler.getInstance().run()