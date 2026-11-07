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
from subsystems.swerve_subsystem import SwerveSubsystem
import rev

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

        self.encoders = [
            phoenix6.hardware.CANcoder(13),
            phoenix6.hardware.CANcoder(23),
            phoenix6.hardware.CANcoder(33),
            phoenix6.hardware.CANcoder(43),
        ]


        # self.motor = rev.SparkMax(21, rev.SparkMax.MotorType.kBrushless)

        # config = rev.SparkMaxConfig()

        # config.setIdleMode(rev.SparkBaseConfig.IdleMode.kBrake)

        # self.motor.configure(
        #     config,
        #     rev.ResetMode.kResetSafeParameters,
        #     rev.PersistMode.kPersistParameters
        # )

        # for encoder in self.encoders:
        #     config = phoenix6.configs.CANcoderConfiguration()
        #     config.magnet_sensor.magnet_offset = -1*encoder.get_absolute_position().value
        #     encoder.configurator.apply(config)

        #self.br_module = SwerveModule(21, 22, 23, 0, 0, 0)

        self.swerve_subsystem = SwerveSubsystem(
            SwerveModule(
                constants.SwerveModules.FrontLeft.DRIVE_ID,
                constants.SwerveModules.FrontLeft.STEER_ID,
                constants.SwerveModules.FrontLeft.ENCODER_ID,
                Translation2d(
                    constants.SwerveModules.FrontLeft.FRONT, 
                    constants.SwerveModules.FrontLeft.LEFT
                ),
                constants.SwerveModules.FrontLeft.ENCODER_OFFSET
            ),
            SwerveModule(
                constants.SwerveModules.FrontRight.DRIVE_ID,
                constants.SwerveModules.FrontRight.STEER_ID,
                constants.SwerveModules.FrontRight.ENCODER_ID,
                Translation2d(
                    constants.SwerveModules.FrontRight.FRONT, 
                    constants.SwerveModules.FrontRight.LEFT
                ),
                constants.SwerveModules.FrontRight.ENCODER_OFFSET
            ),
            SwerveModule(
                constants.SwerveModules.BackLeft.DRIVE_ID,
                constants.SwerveModules.BackLeft.STEER_ID,
                constants.SwerveModules.BackLeft.ENCODER_ID,
                Translation2d(
                    constants.SwerveModules.BackLeft.FRONT, 
                    constants.SwerveModules.BackLeft.LEFT
                ),
                constants.SwerveModules.BackLeft.ENCODER_OFFSET
            ),
            SwerveModule(
                constants.SwerveModules.BackRight.DRIVE_ID,
                constants.SwerveModules.BackRight.STEER_ID,
                constants.SwerveModules.BackRight.ENCODER_ID,
                Translation2d(
                    constants.SwerveModules.BackRight.FRONT, 
                    constants.SwerveModules.BackRight.LEFT
                ),
                constants.SwerveModules.BackRight.ENCODER_OFFSET
            )
        )
        

    def teleopPeriodic(self):
        x = self.controller.getLeftX()
        y = self.controller.getLeftY()

        r_x = self.controller.getRightX()

        vx, vy = db.get_translation_velocities(x, y)

        SmartDashboard.putNumber("vx", vx)
        SmartDashboard.putNumber("vy", vy)

        omega = db.get_rotation_velocities(r_x)

        speeds = ChassisSpeeds.fromFieldRelativeSpeeds(
            vx, vy, omega,
            self.gyro.getRotation2d()
        )

        module_states = self.kinematics.toSwerveModuleStates(speeds)

        self.kinematics.desaturateWheelSpeeds(module_states, constants.MAX_SPEED)

        self.swerve_subsystem.set_states(module_states)

        # fl_states = SwerveModuleState.optimize(fl_states, self.front_left.get_state().angle)
        # fr_states = SwerveModuleState.optimize(fr_states, self.front_right.get_state().angle)
        # bl_states = SwerveModuleState.optimize(bl_states, self.back_left.get_state().angle)
        # br_states = SwerveModuleState.optimize(br_states, self.back_right.get_state().angle)

        
        # set states ot the swerve mdouels
        SmartDashboard.putString("test", str(self.gyro.getYaw()))

        # encoders = [
        #     phoenix6.hardware.CANcoder(constants.SwerveModules.FrontLeft.ENCODER_ID),
        #     phoenix6.hardware.CANcoder(constants.SwerveModules.FrontRight.ENCODER_ID),
        #     phoenix6.hardware.CANcoder(constants.SwerveModules.BackLeft.ENCODER_ID),
        #     phoenix6.hardware.CANcoder(constants.SwerveModules.BackRight.ENCODER_ID),
        # ]

        # for encoder in encoders:
        #     config = phoenix6.configs.CANcoderConfiguration()
        #     config.magnet_sensor.magnet_offset = 0.0
        #     encoder.configurator.apply(config)

        SmartDashboard.putNumber("13 Encoder", self.encoders[0].get_absolute_position().value)

        drive_x, drive_y = db.apply_deadband(self.controller.getLeftX(), self.controller.getLeftY())

        mag = math.hypot(drive_x, drive_y)
        angle = math.atan2(drive_y, drive_x)

        #self.br_module.set_drive(mag)
        #self.br_module.set_steer(angle)

    def robotPeriodic(self):
        commands2.CommandScheduler.getInstance().run()
        
