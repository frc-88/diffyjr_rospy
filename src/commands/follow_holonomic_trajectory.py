import math
from dataclasses import dataclass
from commands2 import CommandBase
from wpimath.controller import (
    HolonomicDriveController,
    ProfiledPIDController,
    PIDController,
)
from wpimath.trajectory import TrapezoidProfile, Trajectory
from wpimath.geometry import Pose2d, Translation2d, Rotation2d
from wpilib import Timer

from subsystems.drivetrain import Drivetrain


@dataclass
class PidConfig:
    kP: float = 0.0
    kI: float = 0.0
    kD: float = 0.0
    kF: float = 0.0
    iZone: float = 0.0
    iMax: float = 0.0
    tolerance: float = 0.0


@dataclass
class FollowTrajectoryConfig:
    vx_pid: PidConfig = PidConfig()
    vy_pid: PidConfig = PidConfig()
    vtheta_pid: PidConfig = PidConfig()
    max_angular_vel: float = math.pi  # radians/second
    max_angular_accel: float = math.pi  # radians/second^2
    x_tolerance: float = 0.1  # meters
    y_tolerance: float = 0.1  # meters
    theta_tolerance: float = 0.05  # radians
    initial_distance_limit: float = 3.0
    initial_rotation_limit: float = math.radians(45.0)  # radians


class FollowHolonomicTrajectory(CommandBase):
    controller: HolonomicDriveController

    def __init__(
        self,
        drive: Drivetrain,
        config: FollowTrajectoryConfig,
        trajectory: Trajectory,
        start_rotation: Rotation2d,
        end_rotation: Rotation2d,
        reset_odometry: bool,
    ) -> None:
        super().__init__()
        self.config = config
        self.drive = drive
        self.trajectory = trajectory
        self.rotation = end_rotation - start_rotation
        self.reset_odometry = reset_odometry

        self.initial_pose = Pose2d()
        self.timer = Timer()

        self.addRequirements(self.drive)

    def initialize(self) -> None:
        self.controller = HolonomicDriveController(
            PIDController(
                self.config.vx_pid.kP, self.config.vx_pid.kI, self.config.vx_pid.kD
            ),
            PIDController(
                self.config.vy_pid.kP, self.config.vy_pid.kI, self.config.vy_pid.kD
            ),
            ProfiledPIDController(
                self.config.vtheta_pid.kP,
                self.config.vtheta_pid.kI,
                self.config.vtheta_pid.kD,
                TrapezoidProfile.Constraints(
                    self.config.max_angular_vel, self.config.max_angular_accel
                ),
            ),
        )
        self.controller.setTolerance(
            Pose2d(
                Translation2d(self.config.x_tolerance, self.config.y_tolerance),
                Rotation2d(self.config.theta_tolerance),
            )
        )

        if self.reset_odometry:
            self.initial_pose = self.drive.get_pose()
        else:
            offset = self.drive.get_pose() - self.trajectory.initialPose()
            if (
                offset.translation().distance(Translation2d())
                > self.config.initial_distance_limit
                or offset.rotation() > self.config.initial_rotation_limit
            ):
                print("Canceling holomic trajectory! Initial trajectory limit reached.")
                self.cancel()
        self.controller.setEnabled(True)
        self.timer.reset()
        self.timer.start()

    def execute(self) -> None:
        current_pose = self.drive.get_pose().relativeTo(self.initial_pose)
        desired_state = self.trajectory.sample(self.timer.get())
        target_rotation = Rotation2d(
            self.timer.get() * self.rotation.radians() / self.trajectory.totalTime()
        )
        target_speeds = self.controller.calculate(
            current_pose, desired_state, target_rotation
        )
        self.drive.drive(target_speeds)

    def end(self, interrupted: bool) -> None:
        self.drive.stop()

    def isFinished(self) -> bool:
        return self.timer.get() > self.trajectory.totalTime()
