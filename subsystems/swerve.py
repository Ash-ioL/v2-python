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