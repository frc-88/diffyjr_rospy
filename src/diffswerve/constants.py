import math
from enum import Enum
from wpimath import units


class TalonIds(Enum):
    FL_HI_FALCON = 16
    FL_LO_FALCON = 1
    BL_HI_FALCON = 2
    BL_LO_FALCON = 3
    BR_HI_FALCON = 12
    BR_LO_FALCON = 13
    FR_HI_FALCON = 15
    FR_LO_FALCON = 14


class CanIds(Enum):
    CANIFIER = 1


class DioIDs(Enum):
    ENCODER_FL = 0
    ENCODER_BL = 1
    ENCODER_BR = 2
    ENCODER_FR = 3


EPSILON = 0.00001


class DriveTrain(Enum):
    WIDTH = 0.30861  # meters
    LENGTH = 0.30861  # meters
    FRONT_LEFT_ENCODER_OFFSET = math.radians(140.449)  # module 0 offset degrees
    BACK_LEFT_ENCODER_OFFSET = math.radians(-105.205)  # module 1 offset degrees
    BACK_RIGHT_ENCODER_OFFSET = math.radians(-46.055)  # module 2 offset degrees
    FRONT_RIGHT_ENCODER_OFFSET = math.radians(-163.652)  # module 3 offset degrees

    LINEAR_DEADBAND = 0.01  # Linear command velocity deadband (m/s)
    ANG_DEADBAND = 0.01  # Angular command velocity deadband (rad/s)

    MAX_CHASSIS_SPEED = 4.48  # Maximum chassis speed (m/s)
    MAX_CHASSIS_ANG_VEL = 20.5  # Maximum chassis rotational velocity (rad/s)
    MAX_CHASSIS_LINEAR_ACCEL = 6.0  # Maximum chassis linear acceleration (m/s^2)
    MAX_CHASSIS_ANG_ACCEL = 30.0  # Maximum chassis angular acceleration (m/s^2)
    MIN_CHASSIS_SPEED = 0.05  # Minimum chassis speed that isn't zero (m/s)
    MIN_CHASSIS_ANG_VEL = (
        0.1  # Minimum chassis rotational velocity that isn't zero (rad/s)
    )

    MAX_BATTERY_SLEW_RATE = (
        2.0  # How much to dampen the max voltage limit (Volts/second)
    )
    BROWNOUT_ZONE = 7.0  # Voltage below which velocities should be severely throttled
    BROWNOUT_ZONE_MAX_VOLTAGE = 2.5  # Voltage scale to set while in the brown out zone
    ENABLE_BATTERY_CONSTRAINT = True

    # Chassis angle PID control constants
    ANGLE_kP = 3.0
    ANGLE_kI = 0.0
    ANGLE_kD = 0.05

    CONSTRAINT_LINEAR_VEL = 4.5  # PID Controller max velocity (m/s)
    CONSTRAINT_LINEAR_ACCEL = 12.0  # Maximum artificial linear acceleration (m/s^2)
    CONSTRAINT_ANG_ACCEL = 30.0  # Maximum artificial angular acceleration (m/s^2)
    ENABLE_LINEAR_ACCEL_CONSTRAINT = True
    ENABLE_ANG_ACCEL_CONSTRAINT = False


class DiffSwerveModule(Enum):
    kDt = 0.005  # update rate of our modules 5ms.
    FALCON_FREE_SPEED = units.rotationsPerMinuteToRadiansPerSecond(6380.0)
    TIMEOUT = 500  # CAN sensor timeout

    # Differential swerve matrix constants. Matrix shape:
    # | M11   M12 |
    # | M21   M22 |
    GEAR_M11 = 1.0 / 24.0
    GEAR_M12 = -1.0 / 24.0
    GEAR_M21 = 5.0 / 72.0
    GEAR_M22 = 7.0 / 72.0

    FALCON_MAX_SPEED_RPS = units.rotationsPerMinuteToRadiansPerSecond(
        600.0
    )  # radians per second
    WHEEL_RADIUS = 0.04445  # Meters with wheel compression.
    FALCON_TICKS_TO_ROTATIONS = 1.0 / 2048.0  # rotations per tick
    AZIMUTH_ROTATIONS_TO_RADIANS = 2.0 * math.pi
    # radians per rotation of azimuth sensor
    VOLTAGE = 12.0  # volts
    FEED_FORWARD = VOLTAGE / FALCON_FREE_SPEED
    NEUTRAL_DEADBAND_PERCENT = 0.001

    ENABLE_CURRENT_LIMIT = False
    CURRENT_LIMIT = 60.0  # amps
    CURRENT_THRESHOLD = 60.0
    CURRENT_TRIGGER_TIME = 0.0

    # Create Parameters for DiffSwerve State Space
    INERTIA_WHEEL = 0.003  # kg * m^2
    INERTIA_AZIMUTH = 0.005  # kg * m^2

    # A weight for how aggressive each state should be ie. 0.08 radians will try to control the
    # angle more aggressively than the wheel angular velocity.
    Q_AZIMUTH = 0.095  # radians
    Q_AZIMUTH_ANG_VELOCITY = 1.1  # radians per sec
    Q_WHEEL_ANG_VELOCITY = 1.0  # radians per sec

    # This is for Kalman filter which isn't used for azimuth angle due to angle wrapping.
    # Model noise are assuming that our model isn't as accurate as our sensors.
    MODEL_AZIMUTH_ANGLE_NOISE = 0.1  # radians
    MODEL_AZIMUTH_ANG_VELOCITY_NOISE = 5.0  # radians per sec
    MODEL_WHEEL_ANG_VELOCITY_NOISE = 5.0  # radians per sec

    # Noise from sensors. Falcon With Gearbox causes us to have more uncertainty so we increase
    # the noise.
    SENSOR_AZIMUTH_ANGLE_NOISE = 0.05  # radians
    SENSOR_AZIMUTH_ANG_VELOCITY_NOISE = 0.01  # radians per sec
    SENSOR_WHEEL_ANG_VELOCITY_NOISE = 0.01  # radians per sec
    CONTROL_EFFORT = VOLTAGE
