#!/usr/bin/env python3
import typing
import rospy
import wpilib
import commands2
from configure_rospy import configure_rospy
from robotcontainer import RobotContainer
from diffswerve import constants


class MyRobot(commands2.TimedCommandRobot):
    container: RobotContainer
    autonomous_command: typing.Optional[commands2.Command] = None

    def robotInit(self):
        print("Initializing node")
        rospy.init_node(
            "diffyjr_roborio"
            # disable_signals=True
            # log_level=rospy.DEBUG
        )
        print("Node initialized")
        self.container = RobotContainer()
        self.addPeriodic(
            self.container.fast_periodic, constants.DiffSwerveModule.kDt, 0.0025
        )
        self.addPeriodic(self.container.slow_periodic, 1.0 / 5.0, 0.02)
        print("Robot initialized!")

    def disabledInit(self) -> None:
        """This function is called once each time the robot enters Disabled mode."""
        self.container.set_enable_drive(False)
        if self.autonomous_command:
            self.autonomous_command.cancel()

    def disabledPeriodic(self) -> None:
        """This function is called periodically when disabled"""

    def autonomousInit(self):
        """This function is run once each time the robot enters autonomous mode."""
        self.autonomous_command = self.container.get_autonomous_command()

        if self.autonomous_command:
            self.autonomous_command.schedule()

    def autonomousPeriodic(self):
        """This function is called periodically during autonomous."""

    def teleopInit(self) -> None:
        # This makes sure that the autonomous stops running when
        # teleop starts running. If you want the autonomous to
        # continue until interrupted by another command, remove
        # this line or comment it out.
        self.container.set_enable_drive(True)
        if self.autonomous_command:
            self.autonomous_command.cancel()

    def teleopPeriodic(self):
        """This function is called periodically during operator control."""

    def testInit(self) -> None:
        # Cancels all running commands at the start of test mode
        self.container.set_enable_drive(True)
        commands2.CommandScheduler.getInstance().cancelAll()


if __name__ == "__main__":
    configure_rospy()
    wpilib.run(MyRobot)
