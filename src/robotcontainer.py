import commands2
from subsystems.drivetrain import Drivetrain
from commands.passthrough_cmdvel_command import PassthroughCmdVelCommand
from commands.coast_drive_motors import CoastDriveMotors
from wpilib import RobotController


class RobotContainer:
    def __init__(self) -> None:
        self.drivetrain = Drivetrain()
        self.user_button = commands2.Trigger(lambda: RobotController.getUserButton())

        self.passthrough_ros_command = PassthroughCmdVelCommand(self.drivetrain)
        self.coast_drive_motors = CoastDriveMotors(self.drivetrain)

        self.configure_default_commands()

    def configure_default_commands(self) -> None:
        self.drivetrain.setDefaultCommand(self.passthrough_ros_command)
        self.user_button.whileTrue(self.coast_drive_motors)

    def get_autonomous_command(self) -> commands2.Command:
        return commands2.WaitCommand(15.0)

    def set_enable_drive(self, enabled: bool) -> None:
        self.drivetrain.set_enabled(enabled)
        self.drivetrain.set_coast(not enabled)

    def fast_periodic(self):
        self.drivetrain.fast_periodic()
