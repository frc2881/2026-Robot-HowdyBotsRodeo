from wpilib import DriverStation
from commands2 import cmd
from lib import logger, telemetry, utils
from lib.classes import RobotState
from lib.controllers.xbox import XboxController
from lib.controllers.button import ButtonController
from lib.sensors.gyro_navx2 import Gyro
from lib.sensors.pose import PoseSensor
from lib.sensors.binary import BinarySensor
from core.commands.auto import Auto
from core.commands.game import Game
from core.subsystems.drive import Drive
from core.subsystems.intake import Intake
from core.subsystems.launcher import Launcher
from core.services.localization import Localization
from core.services.targeting import Targeting
from core.services.match import Match
from core.services.lights import Lights
from core.classes import Target, AutoPath
import core.constants as constants

class RobotCore:
  def __init__(self) -> None:
    self._initSensors()
    self._initSubsystems()
    self._initServices()
    self._initCommands()
    self._initControllers()
    self._initTriggers()
    self._initTelemetry()
    utils.addRobotPeriodic(self._periodic)

  def _initSensors(self) -> None:
    self.gyro = Gyro(constants.Sensors.Gyro.NAVX_PORT)
    self.poseSensors = tuple(PoseSensor(c) for c in constants.Sensors.Pose.POSE_SENSOR_CONFIGS)
    self.launcherSensor = BinarySensor(constants.Sensors.Proximity.LAUNCHER_SENSOR_CONFIG)

  def _initSubsystems(self) -> None:
    self.drive = Drive(lambda: self.gyro.getHeading())
    self.intake = Intake()
    self.launcher = Launcher(lambda: self.launcherSensor.hasTarget())
    
  def _initServices(self) -> None:
    self.localization = Localization(lambda: self.gyro.getHeading(), lambda: self.drive.getModulePositions(), self.poseSensors)
    self.targeting = Targeting(lambda: self.localization.getRobotPose(), lambda: self.localization.getRobotZone(), lambda: self.drive.getChassisSpeeds())
    self.match = Match()
    self.lights = Lights(
      lambda: self.isHoming(), 
      lambda: self.isHomed(), 
      lambda: self.localization.hasValidPoseSensorResult(),
      lambda: self.game.isReadyToLaunch()
    )

  def _initCommands(self) -> None:
    self.game = Game(self)
    self.auto = Auto(self)

  def _initControllers(self) -> None:
    DriverStation.silenceJoystickConnectionWarning(not utils.isCompetitionMode())
    self.driver = XboxController(constants.Controllers.DRIVER_CONTROLLER_CONFIG)
    self.operator = XboxController(constants.Controllers.OPERATOR_CONTROLLER_CONFIG)
    self.homingButton = ButtonController(constants.Controllers.HOMING_BUTTON_CONFIG)

  def _initTriggers(self) -> None:
    self._setupDriver()
    self._setupOperator()

    self.homingButton.pressed().debounce(1.0).whileTrue(
      cmd.parallel(
        self.intake.resetToHome(), 
        self.launcher.resetToHome(),
        self.drive.holdCoastMode()
      ).onlyWhile(lambda: utils.getRobotState() == RobotState.DISABLED)
      .ignoringDisable(True)
      .withName("HomingButton:Pressed")
    )

  def _setupDriver(self) -> None:
    self.drive.setDefaultCommand(self.drive.drive(self.driver.getLeftY, self.driver.getLeftX, self.driver.getRightX))
    self.driver.leftStick().whileTrue(self.drive.lockSwerveModules())
    # self.driver.rightStick().whileTrue(cmd.none())
    self.driver.leftTrigger().whileTrue(self.game.scoreLeft())
    self.driver.rightTrigger().whileTrue(self.game.scoreRight())
    self.driver.leftBumper().whileTrue(self.auto.followPath(AutoPath.HAYBINE_LEFT))
    self.driver.rightBumper().whileTrue(self.auto.followPath(AutoPath.HAYBINE_RIGHT))
    # self.driver.a().whileTrue(cmd.none())
    # self.driver.b().whileTrue(cmd.none())
    # self.driver.y().whileTrue(cmd.none())
    # self.driver.x().whileTrue(cmd.none())
    self.driver.povLeft().whileTrue(self.game.alignRobotToTargetPose(Target.BALE_LEFT))
    self.driver.povRight().whileTrue(self.game.alignRobotToTargetPose(Target.BALE_RIGHT))
    # self.driver.povUp().whileTrue(cmd.none())
    # self.driver.povDown().whileTrue(cmd.none())
    # self.driver.start().whileTrue(cmd.none())
    self.driver.back().debounce(0.5).whileTrue(self.game.resetGyro())

  def _setupOperator(self) -> None:
    # self.operator.leftStick().whileTrue(cmd.none())
    # self.operator.rightStick().whileTrue(cmd.none())
    self.operator.leftTrigger().whileTrue(self.game.loadLauncher())
    self.operator.rightTrigger().whileTrue(self.launcher.test())
    self.operator.leftBumper().whileTrue(self.game.holdHay())
    self.operator.rightBumper().whileTrue(self.game.placeHay())
    # self.operator.a().whileTrue(cmd.none())
    # self.operator.b().whileTrue(cmd.none())
    # self.operator.y().whileTrue(cmd.none())
    # self.operator.x().whileTrue(cmd.none())
    # self.operator.povLeft().whileTrue(cmd.none())
    # self.operator.povRight().whileTrue(cmd.none())
    self.operator.povUp().debounce(0.5).whileTrue(self.launcher.resetToHome())
    self.operator.povDown().debounce(0.5).whileTrue(self.intake.resetToHome())
    # self.operator.start().whileTrue(cmd.none())
    # self.operator.back().whileTrue(cmd.none())

  def _initTelemetry(self) -> None:
    telemetry.log("Game/Robot/Type", constants.Game.Robot.TYPE.name)
    telemetry.log("Game/Robot/Name", constants.Game.Robot.NAME)
    telemetry.log("Game/Field/Length", constants.Game.Field.LENGTH)
    telemetry.log("Game/Field/Width", constants.Game.Field.WIDTH)
    telemetry.log("Robot/Cameras/Driver", constants.Cameras.DRIVER_STREAM)

  def _periodic(self) -> None:
    self._updateTelemetry()

  def disabledInit(self) -> None:
    self.reset()

  def autoInit(self) -> None:
    self.reset()

  def autoExit(self) -> None: 
    self.gyro.resetRobotToField(self.localization.getRobotPose())

  def teleopInit(self) -> None:
    self.reset()

  def testInit(self) -> None:
    self.reset()

  def simulationInit(self) -> None:
    self.reset()

  def reset(self) -> None:
    self.drive.reset()

  def isHoming(self) -> bool:
    return self.intake.isHoming() or self.launcher.isHoming()

  def isHomed(self) -> bool:
    return self.intake.isHomed() and self.launcher.isHomed()

  def _updateTelemetry(self) -> None:
    telemetry.log("Robot/Status/IsHoming", self.isHoming())
    telemetry.log("Robot/Status/IsHomed", self.isHomed())
