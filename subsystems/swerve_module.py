import rev
import phoenix6

class SwerveModule():

    # Type = Mk4 L1
    # Drive ratio: 8.14:1
    # Steer ratio: 12.8:1

    def __init__(self, drive_motor, steer_motor, encoder, front, left, encoder_offset):
        self.drive_motor = rev.SparkMax(drive_motor)
        self.steer_motor = rev.SparkMax(steer_motor)
        self.encoder = phoenix6.hardware.CANcoder(encoder)

        self.front = front
        self.left = left
        self.encoder_offset = encoder_offset
    
