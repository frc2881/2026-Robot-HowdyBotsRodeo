import wpilib
from wpimath import units
from wpimath.geometry import Pose3d, Transform3d, Translation3d, Rotation3d, Translation2d, Rotation2d, Rectangle2d
from wpimath.kinematics import SwerveDrive4Kinematics
from robotpy_apriltag import AprilTagFieldLayout
import navx
from rev import SparkLowLevel, AbsoluteEncoderConfig
from pathplannerlib.config import RobotConfig
from pathplannerlib.controller import PPHolonomicDriveController, PIDConstants
from lib import logger, telemetry, utils
from lib.classes import (
  RobotType,
  Alliance, 
  PID,
  State,
  SpeedMode,
  DriveOrientation,
  MotorModel,
  SwerveDriveModuleGearKit,
  SwerveDriveModuleConfigConstants, 
  SwerveDriveModuleConfig, 
  SwerveDriveModuleLocation, 
  PoseAlignmentConstants,
  HeadingAlignmentConstants,
  XboxControllerConfig,
  PoseSensorConfig
)
from core.classes import Target, Zone, LaunchMetric
import lib.constants

_aprilTagFieldLayout = AprilTagFieldLayout(f'{ wpilib.getDeployDirectory() }/localization/2026-hayharvest.json')

class Subsystems:
  class Drive:
    BUMPER_LENGTH: units.meters = units.inchesToMeters(37.0)
    BUMPER_WIDTH: units.meters = units.inchesToMeters(31.0)
    WHEEL_BASE: units.meters = units.inchesToMeters(26.5)
    TRACK_WIDTH: units.meters = units.inchesToMeters(20.5)

    _drivingMotorModel = MotorModel.NEO
    _swerveDriveModuleGearKit = SwerveDriveModuleGearKit.HIGH # TODO: confirm actual gearing kit installed with swerve drive
    _swerveDriveModuleConstants = SwerveDriveModuleConfigConstants(
      drivingControllerType = SparkLowLevel.SparkModel.kSparkMax,
      drivingMotorType = SparkLowLevel.MotorType.kBrushless,
      drivingFreeSpeed = lib.constants.Motors.FREE_SPEEDS[_drivingMotorModel],
      drivingGearReduction = lib.constants.Drive.Swerve.GEAR_RATIOS[_swerveDriveModuleGearKit],
      drivingCurrentLimit = 60,
      drivingControlPID = PID(0.04, 0, 0),
      turningCurrentLimit = 20,
      turningControlPID = PID(1.0, 0, 0),
      turningEncoderConfig = AbsoluteEncoderConfig.Presets.REV_ThroughBoreEncoder(),
      wheelDiameter = units.inchesToMeters(3.0),
      telemetryName = "Robot/Subsystems/Drive/Modules"
    )
    SWERVE_DRIVE_MODULE_CONFIGS: tuple[SwerveDriveModuleConfig, SwerveDriveModuleConfig, SwerveDriveModuleConfig, SwerveDriveModuleConfig] = (
      SwerveDriveModuleConfig(SwerveDriveModuleLocation.FRONT_LEFT, 2, 3, -90, Translation2d(WHEEL_BASE / 2, TRACK_WIDTH / 2), _swerveDriveModuleConstants),
      SwerveDriveModuleConfig(SwerveDriveModuleLocation.FRONT_RIGHT, 4, 5, 0, Translation2d(WHEEL_BASE / 2, -TRACK_WIDTH / 2), _swerveDriveModuleConstants),
      SwerveDriveModuleConfig(SwerveDriveModuleLocation.REAR_LEFT, 6, 7, 180, Translation2d(-WHEEL_BASE / 2, TRACK_WIDTH / 2), _swerveDriveModuleConstants),
      SwerveDriveModuleConfig(SwerveDriveModuleLocation.REAR_RIGHT, 8, 9, 90, Translation2d(-WHEEL_BASE / 2, -TRACK_WIDTH / 2), _swerveDriveModuleConstants)
    )
    SWERVE_DRIVE_KINEMATICS = SwerveDrive4Kinematics(*(c.chassisTranslation for c in SWERVE_DRIVE_MODULE_CONFIGS))

    TRANSLATION_MAX_VELOCITY: units.meters_per_second = lib.constants.Drive.Swerve.FREE_SPEEDS[_drivingMotorModel][_swerveDriveModuleGearKit] * 1.0
    ROTATION_MAX_VELOCITY: units.degrees_per_second = 720.0

    TARGET_POSE_ALIGNMENT_CONSTANTS = PoseAlignmentConstants(
      translationControlPID = PID(4.0, 0, 0),
      translationMaxVelocity = 3.2,
      translationPositionTolerance = 0.15,
      rotationControlPID = PID(4.0, 0, 0),
      rotationMaxVelocity = 720.0,
      rotationPositionTolerance = 5.0
    )

    TARGET_HEADING_ALIGNMENT_CONSTANTS = HeadingAlignmentConstants(
      rotationControlPID = PID(0.01, 0, 0), 
      rotationPositionTolerance = 1.0
    )

    DRIFT_CORRECTION_CONSTANTS = HeadingAlignmentConstants(
      rotationControlPID = PID(0.01, 0, 0), 
      rotationPositionTolerance = 0.5
    )

    PATHPLANNER_ROBOT_CONFIG = RobotConfig.fromGUISettings()
    PATHPLANNER_CONTROLLER = PPHolonomicDriveController(PIDConstants(5.0, 0, 0), PIDConstants(5.0, 0, 0))

    INPUT_LIMIT_DEMO: units.percent = 0.5
    INPUT_RATE_LIMIT_DEMO: units.percent = 0.5

    SPEED_MODE = SpeedMode.COMPETITION
    DRIVE_ORIENTATION = DriveOrientation.FIELD
    DRIFT_CORRECTION = State.ENABLED

  class Intake:
    pass # TODO: implement intake subsystem once designed in CAD

  class Launcher:
    pass # TODO: implement launcher subsystem once designed in CAD

class Services:
  class Localization:
    MAX_TARGET_AMBIGUITY: units.percent = 0.2
    MAX_TARGET_REPROJECTION_ERROR: float = 1.0
    MAX_TARGET_DISTANCE: units.meters = 5.0
    MAX_POSE_CHANGE: units.meters = 1.0
    STDDEV_XY_COEFF: float = 0.08
    STDDEV_Z_COEFF: float = 0.1
    STDDEV_TARGET_AMBIGUITY_SCALE_FACTOR: float = 5.0
    STDDEV_TARGET_REPROJECTION_ERROR_SCALE_FACTOR: float = 2.5
    VALID_POSE_SENSOR_RESULT_TIMEOUT: units.seconds = 0.3

  # TODO: calculate all launch metrics with physical testing on robot once available
  class Targeting:
    LAUNCH_METRICS: tuple[LaunchMetric, ...] = (
      LaunchMetric(distance = 2.0, speed = 0.25),
      LaunchMetric(distance = 3.0, speed = 0.35)
    )

class Sensors: 
  class Gyro:
    NAVX_PORT = navx.AHRS.NavXComType.kUSB1 # TODO: select correct com type based on gyro installation

  # TODO: calculate correct camera transforms once installed on robot chassis
  class Pose:
    POSE_SENSOR_CONFIGS: tuple[PoseSensorConfig, ...] = (
      # PoseSensorConfig(
      #   cameraName = "FrontLeft", 
      #   transform = Transform3d(
      #     Translation3d(x = units.inchesToMeters(-0.5), y = units.inchesToMeters(14.5), z = units.inchesToMeters(18.0)),
      #     Rotation3d(roll = units.degreesToRadians(0), pitch = units.degreesToRadians(-5.5), yaw = units.degreesToRadians(88.0))
      #   ),
      #   stream = "http://10.28.81.6:1186/?action=stream",
      #   aprilTagFieldLayout = _aprilTagFieldLayout,
      #   telemetryName = "Robot/Sensors/Pose"
      # ),
      # PoseSensorConfig(
      #   cameraName = "FrontRight",
      #   transform = Transform3d(
      #   Translation3d(x = units.inchesToMeters(1.0), y = units.inchesToMeters(-14.0), z = units.inchesToMeters(8.75)),
      #   Rotation3d(roll = units.degreesToRadians(0), pitch = units.degreesToRadians(-17.0), yaw = units.degreesToRadians(-90.0))
      # ),
      #   stream = "http://10.28.81.7:1184/?action=stream",
      #   aprilTagFieldLayout = _aprilTagFieldLayout,
      #   telemetryName = "Robot/Sensors/Pose"
      # )
    )

class Cameras:
  DRIVER_STREAM = "http://10.28.81.6:1182/?action=stream"

class Controllers:
  DRIVER_CONTROLLER_CONFIG = XboxControllerConfig(port = 0, inputDeadband = 0.1, telemetryName = "Robot/Controllers/Driver")
  OPERATOR_CONTROLLER_CONFIG = XboxControllerConfig(port = 1, inputDeadband = 0.1, telemetryName = "Robot/Controllers/Operator")
  INPUT_DEADBAND: units.percent = 0.1

class Game:
  class Robot:
    TYPE = RobotType.COMPETITION
    NAME: str = "TBD" # TODO: provide chosen robot name from team

  class Commands:
    pass

  class Field:
    LENGTH = _aprilTagFieldLayout.getFieldLength()
    WIDTH = _aprilTagFieldLayout.getFieldWidth()
    BOUNDS = Rectangle2d(Translation2d(0, 0), Translation2d(LENGTH, WIDTH))

    # TODO: calculate all target poses from field layout
    TARGETS: dict[Alliance, dict[Target, Pose3d]] = {
      Alliance.BLUE: {
        Target.STABLE_LEFT: Pose3d(8.3, 5.15, 0, Rotation3d(Rotation2d.fromDegrees(0))),
        Target.STABLE_RIGHT: Pose3d(8.3, 1.65, 0, Rotation3d(Rotation2d.fromDegrees(0))),
        Target.HAYBINE_LEFT: Pose3d(0.6, 7.2, 0, Rotation3d(Rotation2d.fromDegrees(0))),
        Target.HAYBINE_RIGHT: Pose3d(0.6, 0.8, 0, Rotation3d(Rotation2d.fromDegrees(0))),
        Target.CROP_CIRCLE_LEFT: Pose3d(1.75, 5.7, 0, Rotation3d(Rotation2d.fromDegrees(0))),
        Target.CROP_CIRCLE_RIGHT: Pose3d(1.75, 2.35, 0, Rotation3d(Rotation2d.fromDegrees(0)))
      },
      Alliance.RED: {
        Target.STABLE_LEFT: Pose3d(8.300, 2.950, 0, Rotation3d(Rotation2d.fromDegrees(180))),
        Target.STABLE_RIGHT: Pose3d(8.300, 6.450, 0, Rotation3d(Rotation2d.fromDegrees(180))),
        Target.HAYBINE_LEFT: Pose3d(15.9, 0.80, 0, Rotation3d(Rotation2d.fromDegrees(180))),
        Target.HAYBINE_RIGHT: Pose3d(15.9, 7.20, 0, Rotation3d(Rotation2d.fromDegrees(180))),
        Target.CROP_CIRCLE_LEFT: Pose3d(14.800, 2.350, 0, Rotation3d(Rotation2d.fromDegrees(180))),
        Target.CROP_CIRCLE_RIGHT: Pose3d(14.800, 5.700, 0, Rotation3d(Rotation2d.fromDegrees(180)))
      }
    }

    # TODO: calculate alliance zone sections (right / left) from field layout
    ZONES: dict[Alliance, dict[Zone, Rectangle2d]] = {
      Alliance.BLUE: {
        Zone.ALLIANCE_ZONE_RIGHT: Rectangle2d(Translation2d(0, 0), Translation2d(0, 0)),
        Zone.ALLIANCE_ZONE_LEFT: Rectangle2d(Translation2d(0, 0), Translation2d(0, 0))
      },
      Alliance.RED: {
        Zone.ALLIANCE_ZONE_RIGHT: Rectangle2d(Translation2d(0, 0), Translation2d(0, 0)),
        Zone.ALLIANCE_ZONE_LEFT: Rectangle2d(Translation2d(0, 0), Translation2d(0, 0))
      }
    }
