import wpilib
from wpimath.geometry import Translation2d;
from wpimath.kinematics import SwerveDrive4Kinematics, ChassisSpeeds
import commands2
from commands2.button import CommandXboxController
import util.drive_deadband as db
import phoenix5
import constants

class Robot(wpilib.TimedRobot):
    def robotInit(self):
        self.front_left_location = Translation2d();
        self.front_right_location = Translation2d();
        self.back_left_location = Translation2d();
        self.back_right_location = Translation2d();

        self.controller = CommandXboxController(0);

        self.kinematics = SwerveDrive4Kinematics(
            self.front_left_location,
            self.front_right_location,
            self.back_left_location,
            self.back_right_location,
        )

        self.gryo = phoenix5.WPI_PigeonIMU(constants.pigeon_id)

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

        # set states ot the swerve mdouels