from navx import AHRS
from commands2 import SubsystemBase
from wpimath.kinematics import ChassisSpeeds
from diffswerve.diff_swerve_chassis import DiffSwerveChassis


class Drivetrain(SubsystemBase):
    def __init__(self):
        super().__init__()
        self.imu = AHRS()
        self.chassis = DiffSwerveChassis(self.imu)

    def periodic(self) -> None:
        self.chassis.periodic()

    def fast_periodic(self) -> None:
        self.chassis.controller_periodic()

    def set_coast(self, coast: bool) -> None:
        self.chassis.set_coast(coast)

    def set_enabled(self, enabled: bool) -> None:
        self.chassis.set_enabled(enabled)

    def drive(self, chassis_speeds: ChassisSpeeds) -> None:
        self.chassis.drive(chassis_speeds)

    def stop(self) -> None:
        self.chassis.stop()
