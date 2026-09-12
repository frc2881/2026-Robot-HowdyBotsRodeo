import wpilib
from wpimath import units
from wpimath.geometry import Pose3d, Transform3d, Translation3d, Rotation3d, Translation2d, Rotation2d
from wpimath.kinematics import SwerveDrive4Kinematics
from robotpy_apriltag import AprilTagFieldLayout
from navx import AHRS
from rev import SparkLowLevel, AbsoluteEncoderConfig
from pathplannerlib.config import RobotConfig
from pathplannerlib.controller import PPHolonomicDriveController, PIDConstants
from lib import logger, utils
from lib.classes import (
  RobotType,
  Alliance, 
  PID,
  Zone,
  MotorModel,
  SwerveModuleGearKit,
  SwerveModuleConstants, 
  SwerveModuleConfig, 
  SwerveModuleLocation, 
  PoseAlignmentConstants,
  HeadingAlignmentConstants,
  PoseSensorConfig
)
from core.classes import Target, LaunchMetric
import lib.constants

_aprilTagFieldLayout = AprilTagFieldLayout(f'{ wpilib.getDeployDirectory() }/localization/2026-hayharvest.json')

class Subsystems:
  class Drive:
    BUMPER_LENGTH: units.meters = units.inchesToMeters(37.0)
    BUMPER_WIDTH: units.meters = units.inchesToMeters(31.0)
    WHEEL_BASE: units.meters = units.inchesToMeters(26.5)
    TRACK_WIDTH: units.meters = units.inchesToMeters(20.5)

    _drivingMotorModel = MotorModel.NEO
    _swerveModuleGearKit = SwerveModuleGearKit.High # TODO: confirm actual gearing kit installed with swerve drive
    
    _swerveModuleConstants = SwerveModuleConstants(
      wheelDiameter = units.inchesToMeters(3.0),
      drivingMotorControllerType = SparkLowLevel.SparkModel.kSparkMax,
      drivingMotorType = SparkLowLevel.MotorType.kBrushless,
      drivingMotorFreeSpeed = lib.constants.Motors.MOTOR_FREE_SPEEDS[_drivingMotorModel],
      drivingMotorReduction = lib.constants.Drive.SWERVE_MODULE_GEAR_RATIOS[_swerveModuleGearKit],
      drivingMotorCurrentLimit = 60,
      drivingMotorPID = PID(0.04, 0, 0),
      turningMotorCurrentLimit = 20,
      turningMotorPID = PID(1.0, 0, 0),
      turningMotorAbsoluteEncoderConfig = AbsoluteEncoderConfig.Presets.REV_ThroughBoreEncoder()
    )

    SWERVE_MODULE_CONFIGS: tuple[SwerveModuleConfig, SwerveModuleConfig, SwerveModuleConfig, SwerveModuleConfig] = (
      SwerveModuleConfig(SwerveModuleLocation.FrontLeft, 2, 3, -90, Translation2d(WHEEL_BASE / 2, TRACK_WIDTH / 2), _swerveModuleConstants),
      SwerveModuleConfig(SwerveModuleLocation.FrontRight, 4, 5, 0, Translation2d(WHEEL_BASE / 2, -TRACK_WIDTH / 2), _swerveModuleConstants),
      SwerveModuleConfig(SwerveModuleLocation.RearLeft, 6, 7, 180, Translation2d(-WHEEL_BASE / 2, TRACK_WIDTH / 2), _swerveModuleConstants),
      SwerveModuleConfig(SwerveModuleLocation.RearRight, 8, 9, 90, Translation2d(-WHEEL_BASE / 2, -TRACK_WIDTH / 2), _swerveModuleConstants)
    )

    DRIVE_KINEMATICS = SwerveDrive4Kinematics(*(c.translation for c in SWERVE_MODULE_CONFIGS))

    TRANSLATION_MAX_VELOCITY: units.meters_per_second = lib.constants.Drive.SWERVE_MODULE_FREE_SPEEDS[_drivingMotorModel][_swerveModuleGearKit] * 1.0
    ROTATION_MAX_VELOCITY: units.degrees_per_second = 540.0

    TARGET_POSE_ALIGNMENT_CONSTANTS = PoseAlignmentConstants(
      translationPID = PID(4.0, 0, 0),
      translationMaxVelocity = 2.4,
      translationPositionTolerance = 0.1,
      rotationPID = PID(4.0, 0, 0),
      rotationMaxVelocity = 720.0,
      rotationPositionTolerance = 3.0
    )

    TARGET_HEADING_ALIGNMENT_CONSTANTS = HeadingAlignmentConstants(
      rotationPID = PID(0.01, 0, 0), 
      rotationPositionTolerance = 0.5
    )

    DRIFT_CORRECTION_CONSTANTS = HeadingAlignmentConstants(
      rotationPID = PID(0.01, 0, 0), 
      rotationPositionTolerance = 0.5
    )

    PATHPLANNER_ROBOT_CONFIG = RobotConfig.fromGUISettings()
    PATHPLANNER_CONTROLLER = PPHolonomicDriveController(PIDConstants(5.0, 0, 0), PIDConstants(5.0, 0, 0))

    INPUT_LIMIT_DEMO: units.percent = 0.5
    INPUT_RATE_LIMIT_DEMO: units.percent = 0.5

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
    class NAVX2:
      COM_TYPE = AHRS.NavXComType.kUSB1 # TODO: select correct com type based on gyro installation

  # TODO: calculate correct camera transforms once installed on robot chassis
  class Pose:
    POSE_SENSOR_CONFIGS: tuple[PoseSensorConfig, ...] = (
      # PoseSensorConfig(
      #   name = "Left", 
      #   transform = Transform3d(
      #     Translation3d(x = units.inchesToMeters(-0.5), y = units.inchesToMeters(14.5), z = units.inchesToMeters(18.0)),
      #     Rotation3d(roll = units.degreesToRadians(0), pitch = units.degreesToRadians(-5.5), yaw = units.degreesToRadians(88.0))
      #   ),
      #   stream = "http://10.28.81.6:1182/?action=stream",
      #   aprilTagFieldLayout = _aprilTagFieldLayout
      # ),
      # PoseSensorConfig(
      #   name = "Right",
      #   transform = Transform3d(
      #   Translation3d(x = units.inchesToMeters(1.0), y = units.inchesToMeters(-14.0), z = units.inchesToMeters(8.75)),
      #   Rotation3d(roll = units.degreesToRadians(0), pitch = units.degreesToRadians(-17.0), yaw = units.degreesToRadians(-90.0))
      # ),
      #   stream = "http://10.28.81.6:1184/?action=stream",
      #   aprilTagFieldLayout = _aprilTagFieldLayout
      # )
    )

class Cameras:
  DRIVER_STREAM = "http://10.28.81.6:1182/?action=stream"

class Controllers:
  DRIVER_CONTROLLER_PORT: int = 0
  OPERATOR_CONTROLLER_PORT: int = 1
  INPUT_DEADBAND: units.percent = 0.1

class Game:
  class Robot:
    TYPE = RobotType.Competition
    NAME: str = "TBD" # TODO: provide chosen robot name from team

  class Commands:
    pass

  class Field:
    LENGTH = _aprilTagFieldLayout.getFieldLength()
    WIDTH = _aprilTagFieldLayout.getFieldWidth()
    ZONE = Zone(start = Translation2d(0, 0), end = Translation2d(LENGTH, WIDTH))

    # TODO: calculate all target poses from field layout
    class Targets:
      TARGETS: dict[Alliance, dict[Target, Pose3d]] = {
        Alliance.Blue: {
          Target.StableLeft: Pose3d(8.3, 5.15, 0, Rotation3d(Rotation2d.fromDegrees(0))),
          Target.StableRight: Pose3d(8.3, 1.65, 0, Rotation3d(Rotation2d.fromDegrees(0))),
          Target.HaybineLeft: Pose3d(0.6, 7.2, 0, Rotation3d(Rotation2d.fromDegrees(0))),
          Target.HaybineRight: Pose3d(0.6, 0.8, 0, Rotation3d(Rotation2d.fromDegrees(0))),
          Target.CropCircleLeft: Pose3d(1.75, 5.7, 0, Rotation3d(Rotation2d.fromDegrees(0))),
          Target.CropCircleRight: Pose3d(1.75, 2.35, 0, Rotation3d(Rotation2d.fromDegrees(0)))
        },
        Alliance.Red: {
          Target.StableLeft: Pose3d(8.300, 2.950, 0, Rotation3d(Rotation2d.fromDegrees(180))),
          Target.StableRight: Pose3d(8.300, 6.450, 0, Rotation3d(Rotation2d.fromDegrees(180))),
          Target.HaybineLeft: Pose3d(15.9, 0.80, 0, Rotation3d(Rotation2d.fromDegrees(180))),
          Target.HaybineRight: Pose3d(15.9, 7.20, 0, Rotation3d(Rotation2d.fromDegrees(180))),
          Target.CropCircleLeft: Pose3d(14.800, 2.350, 0, Rotation3d(Rotation2d.fromDegrees(180))),
          Target.CropCircleRight: Pose3d(14.800, 5.700, 0, Rotation3d(Rotation2d.fromDegrees(180))),
        }
      }

      TARGET_ZONES: dict[Alliance, dict[Target, Zone]] = {
        Alliance.Blue: {},
        Alliance.Red: {}
      }
