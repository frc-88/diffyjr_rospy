#!/usr/bin/env python3

import rospy
import wpilib
import wpilib.drive


class MyRobot(wpilib.TimedRobot):

    def robotInit(self):
        """
        This function is called upon program startup and
        should be used for any initialization code.
        """
        self.timer = wpilib.Timer()

    def autonomousInit(self):
        """This function is run once each time the robot enters autonomous mode."""

    def autonomousPeriodic(self):
        """This function is called periodically during autonomous."""

    def teleopPeriodic(self):
        """This function is called periodically during operator control."""


if __name__ == "__main__":
    wpilib.run(MyRobot)
