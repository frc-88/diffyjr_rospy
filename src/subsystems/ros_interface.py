from typing import List
import rospy
from wpimath.geometry import Quaternion, Rotation3d

from wpimath import units
from wpilib import DriverStation
from commands2 import SubsystemBase

from std_msgs.msg import Float64, Bool, String
from nav_msgs.msg import Odometry
from vision_msgs.msg import Detection3DArray, ObjectHypothesisWithPose
from tj2_interfaces.msg import NavX, Labels

from subsystems.drivetrain import Drivetrain


class RosInterface(SubsystemBase):
    GRAVITY = 9.81

    def __init__(self, drive: Drivetrain) -> None:
        super().__init__()
        self.drive = drive

        self.class_names: List[str] = []

        self.odom_publisher = rospy.Publisher("/tj2/odom", Odometry, queue_size=10)
        self.imu_publisher = rospy.Publisher("/tj2/imu", NavX, queue_size=10)

        self.joint_names = [
            "base_link_to_wheel_0_joint",
            "base_link_to_wheel_1_joint",
            "base_link_to_wheel_2_joint",
            "base_link_to_wheel_3_joint",
        ]
        self.joint_pubs = [
            rospy.Publisher("/tj2/joint/%s" % name, Float64, queue_size=10)
            for name in self.joint_names
        ]

        # fmt: off
        self.pose_covariance = [
            5e-2, 0.0, 0.0, 0.0, 0.0, 0.0,
            0.0, 5e-2, 0.0, 0.0, 0.0, 0.0,
            0.0, 0.0, 5e-2, 0.0, 0.0, 0.0,
            0.0, 0.0, 0.0, 5e-2, 0.0, 0.0,
            0.0, 0.0, 0.0, 0.0, 5e-2, 0.0,
            0.0, 0.0, 0.0, 0.0, 0.0, 5e-2
        ]
        self.twist_covariance = [
            10e-2, 0.0, 0.0, 0.0, 0.0, 0.0,
            0.0, 10e-2, 0.0, 0.0, 0.0, 0.0,
            0.0, 0.0, 10e-2, 0.0, 0.0, 0.0,
            0.0, 0.0, 0.0, 10e-2, 0.0, 0.0,
            0.0, 0.0, 0.0, 0.0, 10e-2, 0.0,
            0.0, 0.0, 0.0, 0.0, 0.0, 10e-2
        ]
        # fmt: on

        self.match_time_publisher = rospy.Publisher(
            "/tj2/match_time", Float64, queue_size=10
        )
        self.is_autonomous_publisher = rospy.Publisher(
            "/tj2/is_autonomous", Bool, queue_size=10
        )
        self.team_color_publisher = rospy.Publisher(
            "/tj2/team_color", String, queue_size=10
        )

        self.labels_subscriber = rospy.Subscriber(
            "/tj2/labels", Labels, self.labels_callback, queue_size=10
        )
        self.detections_subscriber = rospy.Subscriber(
            "/tj2/detections", Detection3DArray, self.detections_callback, queue_size=10
        )

    def slow_periodic(self) -> None:
        for index, module in enumerate(self.drive.chassis.modules):
            state = module.get_state()
            self.publish_joint_state(index, state.angle.radians())

    def periodic(self) -> None:
        self.publish_odom()
        self.publish_match()

    def publish_odom(self) -> None:
        pose = self.drive.chassis.get_odometry_pose()
        speeds = self.drive.chassis.get_chassis_speeds()

        quat = Rotation3d(0.0, 0.0, pose.rotation().radians()).getQuaternion()

        odom = Odometry()
        odom.header.frame_id = "odom"
        odom.child_frame_id = "base_link"
        odom.pose.pose.position.x = pose.x
        odom.pose.pose.position.y = pose.y
        odom.pose.pose.orientation.x = quat.X()
        odom.pose.pose.orientation.y = quat.Y()
        odom.pose.pose.orientation.z = quat.Z()
        odom.pose.pose.orientation.w = quat.W()
        odom.pose.covariance = self.pose_covariance
        odom.twist.twist.linear.x = speeds.vx
        odom.twist.twist.linear.y = speeds.vy
        odom.twist.twist.angular = speeds.omega
        odom.twist.covariance = self.twist_covariance

        self.odom_publisher.publish(odom)

    def publish_imu(self) -> None:
        msg = NavX()
        msg.header.frame_id = "imu"

        quat = Rotation3d(
            units.degreesToRadians(self.drive.imu.getRoll()),
            units.degreesToRadians(self.drive.imu.getPitch()),
            units.degreesToRadians(-self.drive.imu.getYaw()),
        ).getQuaternion()

        msg.orientation.x = quat.X()
        msg.orientation.y = quat.Y()
        msg.orientation.z = quat.Z()
        msg.orientation.w = quat.W()

        msg.angular_velocity.z = units.degreesToRadians(-self.drive.imu.getRate())
        msg.linear_acceleration.x = self.drive.imu.getWorldLinearAccelX() * self.GRAVITY
        msg.linear_acceleration.y = self.drive.imu.getWorldLinearAccelY() * self.GRAVITY

        self.imu_publisher.publish(msg)

    def publish_joint_state(self, index: int, position: float) -> None:
        self.joint_pubs[index].publish(Float64(position))

    def publish_match(self) -> None:
        if DriverStation.getAlliance() == DriverStation.Alliance.kRed:
            alliance = "red"
        elif DriverStation.getAlliance() == DriverStation.Alliance.kBlue:
            alliance = "blue"
        else:
            alliance = ""

        self.match_time_publisher.publish(Float64(DriverStation.getMatchTime()))
        self.is_autonomous_publisher.publish(Bool(DriverStation.isAutonomous()))
        self.team_color_publisher.publish(String(alliance))

    def detections_callback(self, msg: Detection3DArray) -> None:
        objects = set()
        for detection in msg.detections:
            hyp: ObjectHypothesisWithPose = detection.results[0]
            name = self.class_names[hyp.id]
            objects.add(name)
        if len(objects) > 0:
            print(f"Visible objects: {objects}")

    def labels_callback(self, msg: Labels) -> None:
        self.class_names = msg.labels
