from ctre import (
    TalonFX,
    ErrorCode,
    NeutralMode,
    StatusFrame,
    FeedbackDevice,
    SupplyCurrentLimitConfiguration,
    TalonFXControlMode,
)
from diffswerve import constants


class DiffSwerveMotor:
    def __init__(self, can_id: int) -> None:
        self.can_id = can_id
        self.motor = TalonFX(self.hi_can_id)
        self.initialize_falcons()

    def initialize_falcons(self):
        is_ready = False
        while not is_ready:
            try:
                self.set_falcon_parameters(self.motor)
            except BaseException as e:
                print(f"Failed to initialize Falcon motor. Trying again. {e}")
                self.motor = TalonFX(self.can_id)
                continue
            is_ready = True

    def set_falcon_parameters(self, motor: TalonFX) -> None:
        self.check_error_code(motor.configFactoryDefault())
        motor.setInverted(False)
        motor.setSensorPhase(False)
        motor.setNeutralMode(NeutralMode.Brake)
        self.check_error_code(
            motor.configSelectedFeedbackSensor(
                FeedbackDevice.IntegratedSensor, 0, constants.DiffSwerveModule.TIMEOUT
            )
        )
        self.check_error_code(motor.configForwardSoftLimitEnable(False))
        self.check_error_code(
            motor.configVoltageCompSaturation(
                constants.DiffSwerveModule.VOLTAGE, constants.DiffSwerveModule.TIMEOUT
            )
        )
        motor.enableVoltageCompensation(True)
        self.check_error_code(
            motor.setStatusFramePeriod(
                StatusFrame.Status_1_General_, 5, constants.DiffSwerveModule.TIMEOUT
            )
        )
        self.check_error_code(
            motor.setStatusFramePeriod(
                StatusFrame.Status_2_Feedback0_, 20, constants.DiffSwerveModule.TIMEOUT
            )
        )
        self.check_error_code(motor.configForwardSoftLimitEnable(False))
        self.check_error_code(
            motor.configNeutralDeadband(
                constants.DiffSwerveModule.NEUTRAL_DEADBAND_PERCENT,
                constants.DiffSwerveModule.TIMEOUT,
            )
        )
        self.check_error_code(
            motor.configOpenloopRamp(0, constants.DiffSwerveModule.TIMEOUT)
        )
        self.check_error_code(
            motor.configClosedloopRamp(0, constants.DiffSwerveModule.TIMEOUT)
        )
        self.check_error_code(
            motor.configSupplyCurrentLimit(
                SupplyCurrentLimitConfiguration(
                    constants.DiffSwerveModule.ENABLE_CURRENT_LIMIT,
                    constants.DiffSwerveModule.CURRENT_LIMIT,
                    constants.DiffSwerveModule.CURRENT_THRESHOLD,
                    constants.DiffSwerveModule.CURRENT_TRIGGER_TIME,
                )
            )
        )
        self.check_error_code(
            motor.configVoltageMeasurementFilter(0, constants.DiffSwerveModule.TIMEOUT)
        )
        self.check_error_code(
            motor.configMotionProfileTrajectoryInterpolationEnable(
                False, constants.DiffSwerveModule.TIMEOUT
            )
        )

    def check_error_code(self, code: ErrorCode) -> None:
        if code != ErrorCode.OK:
            raise RuntimeError(f"Talon FX motor encountered an error: {str(code)}")

    def set_coast(self, coast: bool) -> None:
        if coast:
            self.motor.setNeutralMode(NeutralMode.Coast)
        else:
            self.motor.setNeutralMode(NeutralMode.Brake)

    def set_voltage(self, voltage: float) -> None:
        voltage = min(
            constants.DiffSwerveModule.VOLTAGE,
            max(-constants.DiffSwerveModule.VOLTAGE, voltage),
        )
        self.motor.set(
            TalonFXControlMode.PercentOutput,
            voltage / constants.DiffSwerveModule.VOLTAGE,
        )

    def get_velocity(self) -> float:
        # get velocity in radians per second
        return (
            self.motor.getSelectedSensorVelocity()
            * constants.DiffSwerveModule.FALCON_TICKS_TO_ROTATIONS
            * constants.DiffSwerveModule.FALCON_MAX_SPEED_RPS
        )
