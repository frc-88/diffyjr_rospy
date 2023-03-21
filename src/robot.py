#!/usr/bin/env python3

import os
import importlib

HOST_MACHINE = "10.0.88.44"
os.environ["ROS_IP"] = "10.0.88.2"
os.environ["ROS_MASTER_URI"] = f"http://{HOST_MACHINE}:11311"
os.environ["ROS_PYTHON_LOG_CONFIG_FILE"] = "/home/lvuser/py/python_logging.yaml"

import logging
import rospy
import wpilib
from std_msgs.msg import Int64
importlib.reload(logging)


class MyRobot(wpilib.TimedRobot):
    def robotInit(self):
        """
        This function is called upon program startup and
        should be used for any initialization code.
        """
        print("Initializing node")
        self.timer = wpilib.Timer()
        rospy.init_node(
            "diffyjr_roborio"
            # disable_signals=True
            # log_level=rospy.DEBUG
        )
        self.count = 0
        self.talker = rospy.Publisher("talker", Int64, queue_size=10)
        print("Robot initialized!")

    def autonomousInit(self):
        """This function is run once each time the robot enters autonomous mode."""

    def autonomousPeriodic(self):
        """This function is called periodically during autonomous."""

    def teleopPeriodic(self):
        """This function is called periodically during operator control."""
    
    def disabledPeriodic(self):
        self.count += 1
        self.talker.publish(Int64(self.count))
        rospy.loginfo(f"count: {self.count}")


if __name__ == "__main__":
    wpilib.run(MyRobot)
