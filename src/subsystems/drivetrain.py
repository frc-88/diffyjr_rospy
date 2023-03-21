from navx import AHRS
from commands2 import SubsystemBase
from wpimath.kinematics import ChassisSpeeds


class Drivetrain(SubsystemBase):
    def __init__(self):
        self.imu = AHRS()
        super().__init__()

    def periodic(self) -> None:
        pass

    def set_coast(self, coast: bool) -> None:
        pass

    def set_enabled(self, enabled: bool) -> None:
        pass

    def drive(self, chassis_speeds: ChassisSpeeds) -> None:
        pass

    def stop(self) -> None:
        pass
