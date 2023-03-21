import math
from typing import Tuple
from navx import AHRS
from wpilib import RobotController
from wpimath.geometry import Translation2d, Rotation2d, Pose2d
from wpimath.kinematics import (
    SwerveDrive4Kinematics,
    SwerveDrive4Odometry,
    SwerveModulePosition,
    SwerveModuleState,
    ChassisSpeeds,
)
from wpimath.trajectory import TrapezoidProfile
from wpimath.controller import ProfiledPIDController
from wpimath.filter import SlewRateLimiter
from diffswerve.diff_swerve_module import DiffSwerveModule
from diffswerve import constants


class DiffSwerveChassis:
    def __init__(self, imu: AHRS) -> None:
        self.is_field_relative = False
        self.angle_controller_enabled = False
        self.setpoint = ChassisSpeeds()
        self.field_relative_offset = Rotation2d()
        self.angle_setpoint = 0.0

        self.imu = imu
        width = constants.DriveTrain.WIDTH
        length = constants.DriveTrain.LENGTH
        self.front_left_position = Translation2d(x=width / 2.0, y=length / 2.0)
        self.back_left_position = Translation2d(x=-width / 2.0, y=length / 2.0)
        self.back_right_position = Translation2d(x=-width / 2.0, y=-length / 2.0)
        self.front_right_position = Translation2d(x=width / 2.0, y=-length / 2.0)

        self.front_left = DiffSwerveModule(
            self.front_left_position,
            constants.TalonIds.FL_LO_FALCON,
            constants.TalonIds.FL_HI_FALCON,
            constants.CanIds.CANIFIER,
            constants.DioIDs.ENCODER_FL,
            constants.DriveTrain.FRONT_LEFT_ENCODER_OFFSET,
        )
        self.back_left = DiffSwerveModule(
            self.back_left_position,
            constants.TalonIds.BL_HI_FALCON,
            constants.TalonIds.BL_LO_FALCON,
            constants.CanIds.CANIFIER,
            constants.DioIDs.ENCODER_BL,
            constants.DriveTrain.BACK_LEFT_ENCODER_OFFSET,
        )
        self.back_right = DiffSwerveModule(
            self.back_right_position,
            constants.TalonIds.BR_LO_FALCON,
            constants.TalonIds.BR_HI_FALCON,
            constants.CanIds.CANIFIER,
            constants.DioIDs.ENCODER_BR,
            constants.DriveTrain.BACK_RIGHT_ENCODER_OFFSET,
        )
        self.front_right = DiffSwerveModule(
            self.front_right_position,
            constants.TalonIds.FR_LO_FALCON,
            constants.TalonIds.FR_HI_FALCON,
            constants.CanIds.CANIFIER,
            constants.DioIDs.ENCODER_FR,
            constants.DriveTrain.FRONT_RIGHT_ENCODER_OFFSET,
        )
        self.modules: Tuple[DiffSwerveModule] = (
            self.front_left,
            self.back_left,
            self.back_right,
            self.front_right,
        )
        self.module_locations: Tuple[Translation2d] = tuple(
            [module.get_module_location() for module in self.modules]
        )
        self.module_positions = tuple(
            [SwerveModulePosition() for _ in range(len(self.modules))]
        )
        self.kinematics = SwerveDrive4Kinematics(*self.module_locations)
        self.odometry = SwerveDrive4Odometry(
            self.kinematics, self.get_imu_yaw(), self.module_positions
        )
        self.angle_controller = ProfiledPIDController(
            constants.DriveTrain.ANGLE_kP,
            constants.DriveTrain.ANGLE_kI,
            constants.DriveTrain.ANGLE_kD,
            TrapezoidProfile.Constraints(
                constants.DriveTrain.CONSTRAINT_LINEAR_VEL,
                constants.DriveTrain.CONSTRAINT_LINEAR_ACCEL,
            ),
        )
        self.angle_controller.enableContinuousInput(-math.pi / 2.0, math.pi / 2.0)

        self.vx_limiter = SlewRateLimiter(constants.DriveTrain.CONSTRAINT_LINEAR_ACCEL)
        self.vy_limiter = SlewRateLimiter(constants.DriveTrain.CONSTRAINT_LINEAR_ACCEL)
        self.vt_limiter = SlewRateLimiter(constants.DriveTrain.CONSTRAINT_ANG_ACCEL)
        self.battery_limiter = SlewRateLimiter(
            constants.DriveTrain.MAX_BATTERY_SLEW_RATE
        )
        self.battery_limiter.reset(self.get_battery_voltage())

    # ---
    # Interfaces
    # ---

    def drive(self, setpoint: ChassisSpeeds) -> None:
        if self.is_within_deadband(setpoint):
            # if setpoints are almost zero, set chassis to hold position
            self.setpoint = ChassisSpeeds()
        elif not self.angle_controller_enabled or abs(setpoint.omega) > 0.0:
            # if translation and rotation are significant, push setpoints as-is
            self.setpoint = setpoint
            self.reset_angle_setpoint()
        else:
            # if only translation is significant, set angular velocity according to
            # previous angle setpoint
            controller_omega = self.angle_controller.calculate(
                self.get_imu_yaw_with_offset().radians(), self.angle_setpoint
            )
            self.setpoint = ChassisSpeeds(setpoint.vx, setpoint.vy, controller_omega)

    def stop(self) -> None:
        self.drive(ChassisSpeeds())

    def get_chassis_speeds(self) -> ChassisSpeeds:
        states = [module.get_state() for module in self.modules]
        return self.kinematics.toChassisSpeeds(*states)

    def get_odometry_pose(self) -> Pose2d:
        return self.odometry.getPose()

    def reset_odometry(self, pose: Pose2d) -> None:
        for module in self.modules:
            module.reset_position(SwerveModulePosition())
        self.update_module_positions()
        self.odometry.resetPosition(self.get_imu_yaw(), self.module_positions, pose)

    def zero_field_offset(self) -> None:
        self.field_relative_offset = self.get_imu_yaw()

    # ---
    # Get sensor values
    # ---

    def get_imu_yaw(self) -> Rotation2d:
        return Rotation2d.fromDegrees(-self.imu.getYaw())

    def get_imu_yaw_with_offset(self) -> Rotation2d:
        return self.get_imu_yaw() - self.field_relative_offset

    def get_battery_voltage(self) -> float:
        return RobotController.getBatteryVoltage()

    # ---
    # Toggle flags
    # ---

    def set_coast(self, coast: bool) -> None:
        for module in self.modules:
            module.set_coast(coast)

    def set_is_field_relative(self, is_field_relative: bool) -> None:
        self.is_field_relative = is_field_relative

    def set_enabled(self, enabled: bool) -> None:
        for module in self.modules:
            module.set_enabled(enabled)

    def set_angle_controller_enabled(self, enabled: bool) -> None:
        self.angle_controller_enabled = enabled

    # ---
    # Periodics
    # ---

    def periodic(self) -> None:
        # should run at 50 Hz
        self.update_module_positions()
        self.odometry.update(self.get_imu_yaw(), self.module_positions)
        self.update_ideal_state(self.setpoint)

    def controller_periodic(self) -> None:
        # this should run at 200 Hz
        for module in self.modules:
            module.update()

    # ---
    # Internal updates
    # ---

    def update_module_positions(self) -> None:
        self.module_positions = tuple(
            [module.get_position() for module in self.modules]
        )

    def update_ideal_state(self, setpoint: ChassisSpeeds) -> None:
        self.set_ideal_state(self.module_states_with_constraints(setpoint))

    def set_ideal_state(
        self, *swerve_module_states: Tuple[SwerveModuleState, ...]
    ) -> None:
        for index in range(len(self.modules)):
            self.modules[index].set_ideal_state(swerve_module_states[index])

    # ---
    # Angle controller
    # ---

    def reset_angle_setpoint(self) -> None:
        if self.angle_controller_enabled:
            self.angle_setpoint = self.get_imu_yaw_with_offset().radians()
            self.angle_controller.reset(self.angle_setpoint)

    # ---
    # Constraints
    # ---

    def is_within_deadband(self, speeds: ChassisSpeeds) -> bool:
        return (
            abs(speeds.vx) < constants.DriveTrain.LINEAR_DEADBAND
            and abs(speeds.vy) < constants.DriveTrain.LINEAR_DEADBAND
            and abs(speeds.omega) < constants.DriveTrain.ANG_DEADBAND
        )

    def module_states_with_constraints(
        self, speeds: ChassisSpeeds
    ) -> Tuple[SwerveModuleState, ...]:
        limit_chassis_speeds = self.get_accel_limited_chassis_speeds(speeds)
        if self.is_field_relative:
            # Apply field relative adjustment after slew limiter to avoid lagging
            limit_chassis_speeds = ChassisSpeeds.fromFieldRelativeSpeeds(
                limit_chassis_speeds, self.get_imu_yaw_with_offset()
            )

        if self.is_within_deadband(limit_chassis_speeds):
            # if within the command deadband, hold the module directions
            states = tuple(
                [
                    SwerveModuleState(0.0, module.get_module_angle())
                    for module in self.modules
                ]
            )
            self.reset_angle_setpoint()
        else:
            states = self.kinematics.toSwerveModuleStates(limit_chassis_speeds)
            states = self.kinematics.desaturateWheelSpeeds(
                states, self.get_battery_limited_max_speed()
            )
        return states

    def get_accel_limited_chassis_speeds(self, speeds: ChassisSpeeds) -> ChassisSpeeds:
        return ChassisSpeeds(
            self.vx_limiter.calculate(speeds.vx)
            if constants.DriveTrain.ENABLE_LINEAR_ACCEL_CONSTRAINT
            else speeds.vx,
            self.vy_limiter.calculate(speeds.vy)
            if constants.DriveTrain.ENABLE_LINEAR_ACCEL_CONSTRAINT
            else speeds.vy,
            self.vt_limiter.calculate(speeds.omega)
            if constants.DriveTrain.ENABLE_ANG_ACCEL_CONSTRAINT
            else speeds.omega,
        )

    def get_battery_limited_max_speed(self) -> float:
        if not constants.DriveTrain.ENABLE_BATTERY_CONSTRAINT:
            return constants.DriveTrain.MAX_CHASSIS_SPEED

        battery_voltage = self.get_battery_voltage()
        limited_battery_voltage = self.battery_limiter.calculate(battery_voltage)
        if battery_voltage < limited_battery_voltage:
            self.battery_limiter.reset(battery_voltage)
            limited_battery_voltage = battery_voltage
        if limited_battery_voltage < constants.DriveTrain.BROWNOUT_ZONE:
            limited_battery_voltage = constants.DriveTrain.BROWNOUT_ZONE_MAX_VOLTAGE
        adjusted_max_speed = (
            constants.DriveTrain.MAX_CHASSIS_SPEED
            * limited_battery_voltage
            / constants.DiffSwerveModule.CONTROL_EFFORT
        )
        if adjusted_max_speed > constants.DriveTrain.MAX_CHASSIS_SPEED:
            adjusted_max_speed = constants.DriveTrain.MAX_CHASSIS_SPEED
        elif adjusted_max_speed < 0.0:
            adjusted_max_speed = 0.0
        return adjusted_max_speed
