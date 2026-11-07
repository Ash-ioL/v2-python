import wpimath.units as units

PIGEON_ID = 30

DEADBAND = 0.15
MAX_SPEED = 3 # m/s
MAX_ROT = 2 # rps

class SwerveModules:
    DRIVE_RATIO = 8.14
    STEER_RATIO = 12.8
    WHEEL_RADIUS = units.inchesToMeters(2.0)

    class FrontLeft:
        DRIVE_ID = 41
        STEER_ID = 42
        ENCODER_ID = 43

        FRONT = units.inchesToMeters(11.5)
        LEFT = units.inchesToMeters(11.5)

        ENCODER_OFFSET = 0#5.177187613
    
    class FrontRight:
        DRIVE_ID = 31
        STEER_ID = 32
        ENCODER_ID = 33

        FRONT = units.inchesToMeters(11.5)
        LEFT = units.inchesToMeters(-11.5)

        ENCODER_OFFSET = 0#1.6689711
    
    class BackLeft:
        DRIVE_ID = 11
        STEER_ID = 12
        ENCODER_ID = 13

        FRONT = units.inchesToMeters(-11.5)
        LEFT = units.inchesToMeters(11.5)

        ENCODER_OFFSET = 0#3.33334290279
    
    class BackRight:
        DRIVE_ID = 21
        STEER_ID = 22
        ENCODER_ID = 23

        FRONT = units.inchesToMeters(-11.5)
        LEFT = units.inchesToMeters(-11.5)

        ENCODER_OFFSET = 0#1.6812421581