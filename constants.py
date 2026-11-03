import wpimath.units as units

PIGEON_ID = 30


class SwerveModules:
    class FrontLeft:
        DRIVE_ID = 41
        SWERVE_ID = 42
        ENCODER_ID = 43

        FRONT = units.inchesToMeters(11.5)
        LEFT = units.inchesToMeters(11.5)

        ENCODER_OFFSET = 5.177187613
    
    class FrontRight:
        DRIVE_ID = 31
        SWERVE_ID = 32
        ENCODER_ID = 33

        FRONT = units.inchesToMeters(11.5)
        LEFT = units.inchesToMeters(-11.5)

        ENCODER_OFFSET = 1.6689711
    
    class BackLeft:
        DRIVE_ID = 11
        SWERVE_ID = 12
        ENCODER_ID = 13

        FRONT = units.inchesToMeters(-11.5)
        LEFT = units.inchesToMeters(11.5)

        ENCODER_OFFSET = 3.33334290279
    
    class BackRight:
        DRIVE_ID = 21
        SWERVE_ID = 22
        ENCODER_ID = 23

        FRONT = units.inchesToMeters(-11.5)
        LEFT = units.inchesToMeters(-11.5)

        ENCODER_OFFSET = 1.6812421581