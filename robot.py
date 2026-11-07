import wpilib
from wpimath.geometry import Translation2d;
from wpimath.kinematics import SwerveDrive4Kinematics, ChassisSpeeds
import commands2
from commands2.button import CommandXboxController
import util.drive_deadband as db
from phoenix5.sensors import WPI_PigeonIMU
import constants
import phoenix6
from wpilib import SmartDashboard
from rev import SparkMax, SparkLowLevel, SparkMaxConfig

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

        self.gyro = WPI_PigeonIMU(constants.pigeon_id)

        self.encoders = [
            phoenix6.hardware.CANcoder(13),
            phoenix6.hardware.CANcoder(23),
            phoenix6.hardware.CANcoder(33),
            phoenix6.hardware.CANcoder(43),
        ]


        self.config = SparkMaxConfig()
        self.config.setIdleMode(SparkMaxConfig.IdleMode.kBrake)
        self.motor = SparkMax(21, SparkLowLevel.MotorType.kBrushless)
        self.motor.configure(
            self.config,
            SparkMax.PersistantMode.kPersistParameters
        )

        # for encoder in self.encoders:
        #     config = phoenix6.configs.CANcoderConfiguration()
        #     config.magnet_sensor.magnet_offset = -1*encoder.get_absolute_position().value
        #     encoder.configurator.apply(config)

        

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

        SmartDashboard.putNumber("13 Encoder", self.encoders[0].get_absolute_position().value)

        
