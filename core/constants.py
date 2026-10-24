import wpilib
from wpimath import units
from wpimath.geometry import Pose3d, Transform3d, Translation3d, Rotation3d, Translation2d, Rotation2d, Rectangle2d
from wpimath.kinematics import SwerveDrive4Kinematics
from robotpy_apriltag import AprilTagFieldLayout
import navx
from rev import SparkLowLevel, AbsoluteEncoderConfig
from pathplannerlib.config import RobotConfig
from pathplannerlib.controller import PPHolonomicDriveController, PIDConstants
from pathplannerlib.path import FlippingUtil
from lib import logger, telemetry, utils
from lib.classes import (
  RobotType,
  Alliance, 
  PID,
  Range,
  State,
  SpeedMode,
  DriveOrientation,
  MotorModel,
  FeedForwardGains,
  SwerveDriveModuleGearKit,
  SwerveDriveModuleConfigConstants, 
  SwerveDriveModuleConfig, 
  SwerveDriveModuleLocation, 
  PoseAlignmentConstants,
  HeadingAlignmentConstants,
  RelativePositionControlModuleConfig,
  VelocityControlModuleConfig,
  FollowerControlModuleConfig,
  CatapultModuleConfig,
  XboxControllerConfig,
  PoseSensorConfig,
  BinarySensorConfig,
  CurrentSensorConfig,
  ButtonControllerConfig
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
    _swerveDriveModuleGearKit = SwerveDriveModuleGearKit.HIGH
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
      translationMaxVelocity = 2.0,
      translationPositionTolerance = 0.25,
      rotationControlPID = PID(4.0, 0, 0),
      rotationMaxVelocity = 540.0,
      rotationPositionTolerance = 3.0
    )

    TARGET_HEADING_ALIGNMENT_CONSTANTS = HeadingAlignmentConstants(
      rotationControlPID = PID(0.01, 0, 0), 
      rotationPositionTolerance = 2.5
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
    ARM_LEADER_CONFIG = RelativePositionControlModuleConfig(
      id = 10, 
      controllerType = SparkLowLevel.SparkModel.kSparkMax,
      motorType = SparkLowLevel.MotorType.kBrushless,
      currentLimit = 60,
      isInverted = True,
      softLimitForward = 22.0,
      softLimitReverse = 0,
      controlPID = PID(0.2, 0, 0),
      outputRange = Range(-1.0, 1.0),
      feedForwardGains = FeedForwardGains(velocity = 12.0 / lib.constants.Motors.FREE_SPEEDS[MotorModel.NEO]),
      cruiseVelocity = 6000.0,
      maxAcceleration = 12000.0,
      allowedProfileError = 0.5,
      homingPosition = 0,
      homingSpeed = 0.1,
      positionConversionFactor = 1.0,
      telemetryName = "Robot/Subsystems/Intake/Arm/Leader"
    )

    ARM_FOLLOWER_CONFIG = FollowerControlModuleConfig(
      id = 11,
      leaderId = 10,
      controllerType = ARM_LEADER_CONFIG.controllerType,
      motorType = ARM_LEADER_CONFIG.motorType,
      currentLimit = ARM_LEADER_CONFIG.currentLimit,
      isInverted = True,
      telemetryName = "Robot/Subsystems/Intake/Arm/Follower"
    )

    ROLLERS_BOTTOM_CONFIG = VelocityControlModuleConfig(
      id = 12,
      controllerType = SparkLowLevel.SparkModel.kSparkMax,
      motorType = SparkLowLevel.MotorType.kBrushless,
      currentLimit = 60,
      isInverted = False,
      controlPID = PID(0.0001, 0, 0),
      feedForwardGains = FeedForwardGains(velocity = 12.0 / lib.constants.Motors.FREE_SPEEDS[MotorModel.NEO]),
      cruiseVelocity = 6000.0,
      maxAcceleration = 12000.0,
      allowedProfileError = 0.1,
      telemetryName = "Robot/Subsystems/Intake/Rollers/Bottom"
    )

    ROLLERS_TOP_CONFIG = VelocityControlModuleConfig(
      id = 13,
      controllerType = SparkLowLevel.SparkModel.kSparkMax,
      motorType = SparkLowLevel.MotorType.kBrushless,
      currentLimit = 60,
      isInverted = True,
      controlPID = PID(0.0001, 0, 0),
      feedForwardGains = FeedForwardGains(velocity = 12.0 / lib.constants.Motors.FREE_SPEEDS[MotorModel.NEO]),
      cruiseVelocity = 6000.0,
      maxAcceleration = 12000.0,
      allowedProfileError = 0.1,
      telemetryName = "Robot/Subsystems/Intake/Rollers/Top"
    )

    ARM_INTAKE_POSITION: float = 20.5
    ROLLERS_BOTTOM_INTAKE_SPEED: units.percent = 0.5
    ROLLERS_TOP_INTAKE_SPEED: units.percent = 0.2
    ROLLERS_HOLD_SPEED: units.percent = 0.15
    ROLLERS_PLACE_SPEED: units.percent = 0.15

    ROLLERS_CURRENT_SENSOR_CONFIG = CurrentSensorConfig(
      targetCurrent = 35,
      changeTime = 0.2,
      telemetryName = "Robot/Sensors/Current/Intake/Rollers"
    )

  class Launcher:
    LAUNCHER_LEADER_CONFIG = CatapultModuleConfig(
      id = 14,
      controllerType = SparkLowLevel.SparkModel.kSparkFlex,
      motorType = SparkLowLevel.MotorType.kBrushless,
      currentLimit = 120,
      isInverted = True,
      softLimitReverse = 0.25,
      softLimitForward = 5.5,
      controlPID = PID(0.0001, 0, 0),
      feedForwardGains = FeedForwardGains(
        velocity = 12.0 / lib.constants.Motors.FREE_SPEEDS[MotorModel.NEO_VORTEX],
        acceleration = 12.0 / lib.constants.Motors.FREE_SPEEDS[MotorModel.NEO_VORTEX]
      ),
      cruiseVelocity = 6000.0,
      maxAcceleration = 30000.0,
      allowedProfileError = 0.1,
      homingSpeed = 0.1,
      retractSpeed = 0.05,
      launchPosition = 5.4,
      positionTolerance = 0.1,
      telemetryName = "Robot/Subsystems/Launcher/Leader"
    )

    LAUNCHER_FOLLOWER_CONFIG = FollowerControlModuleConfig(
      id = 15,
      leaderId = 14,
      controllerType = LAUNCHER_LEADER_CONFIG.controllerType,
      motorType = LAUNCHER_LEADER_CONFIG.motorType,
      currentLimit = LAUNCHER_LEADER_CONFIG.currentLimit,
      isInverted = True,
      telemetryName = "Robot/Subsystems/Launcher/Follower"
    )

class Services:
  class Localization:
    MAX_TARGET_AMBIGUITY: units.percent = 0.2
    MAX_TARGET_REPROJECTION_ERROR: float = 1.0
    MAX_TARGET_DISTANCE: units.meters = 8.0
    MAX_POSE_CHANGE: units.meters = 1.0
    STDDEV_XY_COEFF: float = 0.08
    STDDEV_Z_COEFF: float = 0.1
    STDDEV_TARGET_AMBIGUITY_SCALE_FACTOR: float = 5.0
    STDDEV_TARGET_REPROJECTION_ERROR_SCALE_FACTOR: float = 2.5
    VALID_POSE_SENSOR_RESULT_TIMEOUT: units.seconds = 0.3

  class Targeting:
    LAUNCH_METRICS: tuple[LaunchMetric, ...] = (
      LaunchMetric(distance = 1.5, speed = 0.35),
      LaunchMetric(distance = 2.5, speed = 0.40),
      LaunchMetric(distance = 3.5, speed = 0.45),
      LaunchMetric(distance = 4.5, speed = 0.50),
      LaunchMetric(distance = 5.5, speed = 0.55),
      LaunchMetric(distance = 6.5, speed = 0.60),
      LaunchMetric(distance = 7.5, speed = 0.65),
      LaunchMetric(distance = 8.5, speed = 0.70),
      LaunchMetric(distance = 9.5, speed = 0.75)
    )

  class Lights:
    LED_CHANNEL: int = 0
    LED_LENGTH: int = 8

class Sensors: 
  class Gyro:
    NAVX_PORT = navx.AHRS.NavXComType.kUSB1

  class Pose:
    POSE_SENSOR_CONFIGS: tuple[PoseSensorConfig, ...] = (
      PoseSensorConfig(
        cameraName = "FrontLeft", 
        transform = Transform3d(
          Translation3d(x = units.inchesToMeters(13.36), y = units.inchesToMeters(1.09), z = units.inchesToMeters(11.27)),
          Rotation3d(roll = units.degreesToRadians(1.0), pitch = units.degreesToRadians(-20.0), yaw = units.degreesToRadians(20.0))
        ),
        stream = "http://10.18.82.6:1182/?action=stream",
        aprilTagFieldLayout = _aprilTagFieldLayout,
        telemetryName = "Robot/Sensors/Pose"
      ),
      PoseSensorConfig(
        cameraName = "FrontRight",
        transform = Transform3d(
        Translation3d(x = units.inchesToMeters(13.36), y = units.inchesToMeters(-1.09), z = units.inchesToMeters(11.27)),
        Rotation3d(roll = units.degreesToRadians(-1.0), pitch = units.degreesToRadians(-20.0), yaw = units.degreesToRadians(-20.0))
      ),
        stream = "http://10.18.82.6:1184/?action=stream",
        aprilTagFieldLayout = _aprilTagFieldLayout,
        telemetryName = "Robot/Sensors/Pose"
      )
    )

  class Proximity:
    LAUNCHER_SENSOR_CONFIG = BinarySensorConfig( 
      channel = 1,
      telemetryName = "Robot/Sensors/Proximity/Launcher"
    )

class Cameras:
  DRIVER_STREAM = "http://10.18.82.6:1182/?action=stream"

class Controllers:
  DRIVER_CONTROLLER_CONFIG = XboxControllerConfig(port = 0, inputDeadband = 0.1, telemetryName = "Robot/Controllers/Driver")
  OPERATOR_CONTROLLER_CONFIG = XboxControllerConfig(port = 1, inputDeadband = 0.1, telemetryName = "Robot/Controllers/Operator")
  HOMING_BUTTON_CONFIG = ButtonControllerConfig(channel = 0, telemetryName = "Robot/Controllers/Homing")

class Game:
  class Robot:
    TYPE = RobotType.COMPETITION
    NAME: str = "SPUD"

  class Commands:
    pass

  class Field:
    LENGTH = _aprilTagFieldLayout.getFieldLength()
    WIDTH = _aprilTagFieldLayout.getFieldWidth()
    BOUNDS = Rectangle2d(Translation2d(0, 0), Translation2d(LENGTH, WIDTH))

    TARGETS: dict[Alliance, dict[Target, Pose3d]] = {
      Alliance.BLUE: {
        Target.SCORE_RIGHT: Pose3d(5.80, 1.65, 0, Rotation3d(Rotation2d.fromDegrees(0))),
        Target.SCORE_LEFT: Pose3d(5.80, 5.15, 0, Rotation3d(Rotation2d.fromDegrees(0))),
        Target.STABLE_RIGHT: Pose3d(8.30, 1.65, 0, Rotation3d(Rotation2d.fromDegrees(0))),
        Target.STABLE_LEFT: Pose3d(8.30, 5.15, 0, Rotation3d(Rotation2d.fromDegrees(0))),
        Target.CROP_CIRCLE_RIGHT: Pose3d(1.68, 2.42, 0, Rotation3d(Rotation2d.fromDegrees(140.0))),
        Target.CROP_CIRCLE_LEFT: Pose3d(1.70, 5.70, 0, Rotation3d(Rotation2d.fromDegrees(-120.0))),
        Target.BALE_RIGHT: Pose3d(1.25, 2.78, 0, Rotation3d(Rotation2d.fromDegrees(140.0))),
        Target.BALE_LEFT: Pose3d(1.38, 5.12, 0, Rotation3d(Rotation2d.fromDegrees(-120.0)))
      },
      Alliance.RED: {}
    }

    for target in TARGETS[Alliance.BLUE]:
      pose = FlippingUtil.flipFieldPose(TARGETS[Alliance.BLUE][target].toPose2d())
      TARGETS[Alliance.RED][target] = Pose3d(pose.X(), pose.Y(), TARGETS[Alliance.BLUE][target].Z(), Rotation3d(pose.rotation()))

    ZONES: dict[Alliance, dict[Zone, Rectangle2d]] = {
      Alliance.BLUE: {
        Zone.ALLIANCE_ZONE_RIGHT: Rectangle2d(Translation2d(0, 0), Translation2d(8.27, 4.035)),
        Zone.ALLIANCE_ZONE_LEFT: Rectangle2d(Translation2d(0, 4.036), Translation2d(8.27, 8.07))
      },
      Alliance.RED: {}
    }

    for zone in ZONES[Alliance.BLUE]:
      rectangle = ZONES[Alliance.BLUE][zone]
      ZONES[Alliance.RED][zone] = Rectangle2d(FlippingUtil.flipFieldPose(rectangle.center()), rectangle.xwidth, rectangle.ywidth)
