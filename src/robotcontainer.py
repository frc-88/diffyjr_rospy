import commands2
from subsystems.drivetrain import Drivetrain
from subsystems.ros_interface import RosInterface
from commands.passthrough_cmdvel_command import PassthroughCmdVelCommand
from commands.coast_drive_motors import CoastDriveMotors
from wpilib import RobotController
from commands.follow_holonomic_trajectory import (
    FollowHolonomicTrajectory,
    FollowTrajectoryConfig,
)
from util.trajectory_helper import TrajectoryHelper
from wpimath.geometry import Rotation2d


class RobotContainer:
    def __init__(self) -> None:
        self.drivetrain = Drivetrain()
        self.ros_interface = RosInterface(self.drivetrain)
        self.user_button = commands2.Trigger(lambda: RobotController.getUserButton())

        self.passthrough_ros_command = PassthroughCmdVelCommand(self.drivetrain)
        self.coast_drive_motors = CoastDriveMotors(self.drivetrain)
        self.follow_trajectory = FollowHolonomicTrajectory(
            self.drivetrain,
            FollowTrajectoryConfig(),
            TrajectoryHelper.from_pathweaver_json("test.wpilib.json"),
            Rotation2d(),
            Rotation2d(),
            True,
        )

        self.configure_default_commands()

    def configure_default_commands(self) -> None:
        self.drivetrain.setDefaultCommand(self.passthrough_ros_command)
        self.user_button.whileTrue(self.coast_drive_motors)

    def get_autonomous_command(self) -> commands2.Command:
        return self.follow_trajectory

    def set_enable_drive(self, enabled: bool) -> None:
        self.drivetrain.set_enabled(enabled)
        self.drivetrain.set_coast(not enabled)

    def slow_periodic(self):
        self.ros_interface.slow_periodic()

    def fast_periodic(self):
        self.drivetrain.fast_periodic()
