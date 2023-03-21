#!/usr/bin/env python3
import rospy
import wpilib
from std_msgs.msg import Int64
from configure_rospy import configure_rospy


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
    configure_rospy()
    wpilib.run(MyRobot)
