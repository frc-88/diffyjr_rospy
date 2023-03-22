from typing import Optional
import rospy
from commands2 import CommandBase
from geometry_msgs.msg import Twist
from subsystems.drivetrain import Drivetrain
from wpimath.kinematics import ChassisSpeeds


class PassthroughCmdVelCommand(CommandBase):
    def __init__(self, drivetrain: Drivetrain) -> None:
        super().__init__()
        self.drivetrain = drivetrain
        self.cmd_vel_sub: Optional[rospy.Subscriber] = None
        self.chassis_speeds = ChassisSpeeds()
        self.addRequirements(self.drive)

    def init_subscriber(self) -> None:
        self.cmd_vel_sub = rospy.Subscriber(
            "/tj2/cmd_vel", Twist, self.twist_callback, queue_size=10
        )

    def close_subscriber(self) -> None:
        if self.cmd_vel_sub:
            self.cmd_vel_sub.unregister()

    def twist_callback(self, msg: Twist) -> None:
        self.chassis_speeds.vx = msg.linear.vx
        self.chassis_speeds.vy = msg.linear.vy
        self.chassis_speeds.omega = msg.angular.z

    def initialize(self) -> None:
        self.init_subscriber()

    def execute(self) -> None:
        if not self.cmd_vel_sub:
            print("cmd_vel topic is not connected! Can't command from ROS.")
            return
        self.drivetrain.drive(self.chassis_speeds)

    def end(self, interrupted: bool) -> None:
        if interrupted:
            self.drivetrain.stop()

    def isFinished(self) -> bool:
        return False
