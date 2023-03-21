import math
from typing import Annotated, Tuple, Literal
import numpy as np
from numpy import typing as npt
from wpilib import RobotController

from wpimath.geometry import Translation2d, Rotation2d
from wpimath.kinematics import SwerveModulePosition, SwerveModuleState
from wpimath.system.plant import DCMotor
from diffswerve import constants
from diffswerve.diff_swerve_motor import DiffSwerveMotor
from diffswerve.canified_pwm_encoder import CANifiedPWMEncoder

from wpimath.system import LinearSystem_2_2_2 as LinearSystem
from wpimath.system import LinearSystemLoop_2_2_2 as LinearSystemLoop
from wpimath.estimator import KalmanFilter_2_2_2 as KalmanFilter
from wpimath.controller import LinearQuadraticRegulator_2_2 as LinearQuadraticRegulator

SystemStateArray = Annotated[npt.NDArray[np.float64], Literal[3, 1]]
SystemVelocityStateArray = Annotated[npt.NDArray[np.float64], Literal[2, 1]]
SystemInputArray = Annotated[npt.NDArray[np.float64], Literal[2, 1]]
System3x3Array = Annotated[npt.NDArray[np.float64], Literal[3, 3]]
System3x2Array = Annotated[npt.NDArray[np.float64], Literal[3, 3]]


def input_modulus(input: float, min_input: float, max_input: float):
    modulus = max_input - min_input

    # Wrap input if it's above the maximum input
    num_max = int((input - min_input) / modulus)
    input -= num_max * modulus

    # Wrap input if it's below the minimum input
    num_min = int((input - max_input) / modulus)
    input -= num_min * modulus

    return input


def bound_half_angle(angle: float) -> float:
    angle %= 2.0 * math.pi
    if angle >= math.pi:
        angle -= 2.0 * math.pi
    if angle < -math.pi:
        angle += 2.0 * math.pi
    return angle


class DiffSwerveModule:
    def __init__(
        self,
        module_location: Translation2d,
        lo_can_id: int,
        hi_can_id: int,
        azimuth_canifier_id: int,
        azimuth_pwm_channel: int,
        azimuth_offset_radians: float,
    ) -> None:
        self.module_location = module_location
        self.lo_can_id = lo_can_id
        self.hi_can_id = hi_can_id
        self.azimuth_canifier_id = azimuth_canifier_id
        self.azimuth_pwm_channel = azimuth_pwm_channel
        self.azimuth_offset_radians = azimuth_offset_radians

        self.is_enabled = False
        self.prev_position_update_time = 0.0
        self.wheel_position = 0.0

        self.lo_motor = DiffSwerveMotor(self.lo_can_id)
        self.hi_motor = DiffSwerveMotor(self.hi_can_id)
        self.azimuth_sensor = CANifiedPWMEncoder(
            self.azimuth_canifier_id,
            self.azimuth_pwm_channel,
            self.azimuth_offset_radians,
            constants.DiffSwerveModule.AZIMUTH_ROTATIONS_TO_RADIANS,
            False,
        )

        # fmt: off
        self.diff_matrix = np.array([
            [constants.DiffSwerveModule.GEAR_M11, constants.DiffSwerveModule.GEAR_M12],
            [constants.DiffSwerveModule.GEAR_M21, constants.DiffSwerveModule.GEAR_M22],
        ])
        # fmt: on
        self.inv_diff_matrix = np.linalg.inv(self.diff_matrix)

        (
            self.swerve_control_loop,
            self.swerve_controller,
            self.swerve_observer,
        ) = self.init_control_loop()
        self.input: SystemInputArray = np.zeros((2, 1))
        self.reference: SystemStateArray = np.zeros((3, 1))

    # ---
    # DiffSwerveChassis interfaces
    # ---

    def get_module_location(self) -> Translation2d:
        return self.module_location

    def set_coast(self, coast: bool) -> None:
        self.hi_motor.set_coast(coast)
        self.lo_motor.set_coast(coast)

    def set_enabled(self, enabled: bool) -> None:
        self.is_enabled = enabled

    def get_position(self) -> float:
        return self.wheel_position

    def update(self) -> None:
        self.swerve_control_loop.setNextR(self.reference)
        state = self.get_angular_velocities()
        azimuth_velocity = state[0, 0]
        wheel_velocity = state[1, 0]
        self.swerve_control_loop.correct(
            self.get_module_angle(), azimuth_velocity, wheel_velocity
        )
        if not self.predict():
            return
        self.update_wheel_position(self.get_wheel_ground_velocity())
        if self.is_enabled:
            self.hi_motor.set_voltage(self.input[1, 0])
            self.lo_motor.set_voltage(self.input[0, 0])

    def reset_position(self, position: SwerveModulePosition) -> None:
        self.wheel_position = position.distance

    def get_state(self) -> SwerveModuleState:
        return SwerveModuleState(
            self.get_wheel_ground_velocity(), self.get_module_angle()
        )

    def set_ideal_state(self, state: SwerveModuleState) -> None:
        self.set_reference(SwerveModuleState.optimize(state, self.get_module_angle()))

    def get_module_angle(self) -> Rotation2d:
        return Rotation2d(bound_half_angle(self.azimuth_sensor.get_position()))

    # ---
    # System model
    # ---

    def init_control_loop(
        self,
    ) -> Tuple[LinearSystemLoop, LinearQuadraticRegulator, KalmanFilter]:
        (A_mat, B_mat, C_mat, D_mat) = self.init_diff_swerve_plant(DCMotor.falcon500(2))

        plant = LinearSystem(A_mat, B_mat, C_mat, D_mat)
        swerve_controller = LinearQuadraticRegulator(
            A_mat,
            B_mat,
            (
                constants.DiffSwerveModule.Q_AZIMUTH,
                constants.DiffSwerveModule.Q_AZIMUTH_ANG_VELOCITY,
                constants.DiffSwerveModule.Q_WHEEL_ANG_VELOCITY,
            ),
            (
                constants.DiffSwerveModule.CONTROL_EFFORT,
                constants.DiffSwerveModule.CONTROL_EFFORT,
            ),
            constants.DiffSwerveModule.kDt,
        )

        swerve_observer = KalmanFilter(
            plant,
            (
                constants.DiffSwerveModule.MODEL_AZIMUTH_ANGLE_NOISE,
                constants.DiffSwerveModule.MODEL_AZIMUTH_ANG_VELOCITY_NOISE,
                constants.DiffSwerveModule.MODEL_WHEEL_ANG_VELOCITY_NOISE,
            ),
            (
                constants.DiffSwerveModule.SENSOR_AZIMUTH_ANGLE_NOISE,
                constants.DiffSwerveModule.SENSOR_AZIMUTH_ANG_VELOCITY_NOISE,
                constants.DiffSwerveModule.SENSOR_WHEEL_ANG_VELOCITY_NOISE,
            ),
        )

        control_loop = LinearSystemLoop(
            plant,
            swerve_controller,
            swerve_observer,
            constants.DiffSwerveModule.CONTROL_EFFORT,
            constants.DiffSwerveModule.kDt,
        )

        control_loop.reset(np.zeros((3, 1)))
        return control_loop, swerve_controller, swerve_observer

    def init_diff_swerve_plant(
        self, motor_model: DCMotor
    ) -> Tuple[System3x3Array, System3x2Array, System3x3Array, System3x2Array]:
        J_w = constants.DiffSwerveModule.INERTIA_WHEEL
        K_t = motor_model.Kt  # Nm / Amp
        K_v = motor_model.Kv  # rad / s / Volt
        R_ohm = motor_model.R
        A_subset = (
            self.inv_diff_matrix @ self.inv_diff_matrix @ (-K_t / (K_v * R_ohm * J_w))
        )
        B_subset = self.inv_diff_matrix @ (K_t / (R_ohm * J_w))

        A_mat = np.zeros((3, 3))
        A_mat[0, 1] = 1.0
        A_mat[1:3, 1:3] = A_subset

        B_mat = np.zeros((3, 2))
        B_mat[1:3, 1:3] = B_subset

        C_mat = np.eye(3)
        D_mat = np.zeros((3, 2))

        return A_mat, B_mat, C_mat, D_mat

    # ---
    # System updates
    # ---

    def predict(self) -> bool:
        input_vels = self.get_differential_inputs(
            self.reference[1, 0], self.reference[2, 0]
        )
        feed_forward_volts = input_vels * constants.DiffSwerveModule.FEED_FORWARD

        error = self.compute_error_wrapped(
            self.swerve_control_loop.nextR(),
            self.swerve_control_loop.xhat(),
            -math.pi,
            math.pi,
        )
        clamp_input = self.swerve_control_loop.clampInput(
            (self.swerve_controller.K @ error) + feed_forward_volts
        )
        if math.isnan(clamp_input[0, 0]) or math.isnan(clamp_input[1, 0]):
            print("Input is NaN! Resetting controller.")
            self.swerve_control_loop.reset(np.zeros((3, 1)))
            return False
        self.swerve_observer.predict(clamp_input, constants.DiffSwerveModule.kDt)
        return True

    def set_reference(self, reference_state: SwerveModuleState) -> None:
        self.reference = np.array(
            [
                [
                    reference_state.angle.radians(),
                    0.0,
                    reference_state.speed / constants.DiffSwerveModule.WHEEL_RADIUS,
                ]
            ]
        ).T

    # ---
    # State helpers
    # ---

    def get_angular_velocities(self) -> SystemVelocityStateArray:
        return self.get_differential_outputs(
            self.lo_motor.get_velocity(), self.hi_motor.get_velocity()
        )

    def update_wheel_position(self, wheel_velocity: float) -> float:
        currentTime = self.get_time()
        dt = currentTime - self.prev_position_update_time
        self.prev_position_update_time = currentTime
        self.wheel_position += wheel_velocity * dt

    def get_time(self) -> float:
        return RobotController.getFPGATime() * 1e-6

    def get_wheel_ground_velocity(self) -> float:
        return (
            self.get_angular_velocities()[1, 0]
            * constants.DiffSwerveModule.WHEEL_RADIUS
        )

    def get_differential_outputs(
        self, lo_motor_omega, hi_motor_omega
    ) -> SystemVelocityStateArray:
        return self.diff_matrix @ np.narray([[lo_motor_omega, hi_motor_omega]]).T

    # ---
    # Input helpers
    # ---

    def get_differential_inputs(
        self, azimuth_velocity: float, wheel_velocity: float
    ) -> SystemInputArray:
        return self.inv_diff_matrix @ np.array([[azimuth_velocity, wheel_velocity]]).T

    def compute_error_wrapped(
        self,
        reference: SystemStateArray,
        x_hat: SystemStateArray,
        min_angle: float,
        max_angle: float,
    ) -> SystemStateArray:
        angle_error = reference[0, 0] - self.get_module_angle().radians()
        position_error = input_modulus(angle_error, min_angle, max_angle)
        error = reference - x_hat
        return np.array([[position_error, error[1, 0], error[2, 0]]]).T
