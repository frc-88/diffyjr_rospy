from commands2 import CommandBase

from subsystems.drivetrain import Drivetrain


class CoastDriveMotors(CommandBase):
    def __init__(self, drivetrain: Drivetrain) -> None:
        super().__init__()
        self.drivetrain = drivetrain
        self.addRequirements(self.drive)

    def execute(self) -> None:
        self.drivetrain.stop()
        self.drivetrain.set_coast(True)

    def end(self, interrupted: bool) -> None:
        self.drivetrain.set_coast(False)

    def isFinished(self) -> bool:
        return False

    def runsWhenDisabled(self) -> bool:
        return True
