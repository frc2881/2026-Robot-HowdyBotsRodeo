from typing import TYPE_CHECKING
from wpilib import RobotBase, SmartDashboard
from commands2 import Command, cmd
from lib import logger, telemetry, utils
from lib.classes import ControllerRumbleMode, ControllerRumblePattern
from core.classes import Target, Zone
import core.constants as constants
if TYPE_CHECKING: from core.robot import RobotCore

class Game:
  def __init__(self, robot: "RobotCore") -> None:
    self._robot = robot

    SmartDashboard.putNumber("Robot/Launcher/TestSpeed", 0.3)

  def alignRobotToTargetPose(self, target: Target, alignRotationOnly: bool = False) -> Command:
    return (
      self._robot.drive.alignToTargetPose(self._robot.localization.getRobotPose, lambda: self._robot.targeting.getTargetPose(target), alignRotationOnly)
    ).withName(f'Game:AlignRobotToTargetPose:{ target.name }')

  def alignRobotToNearestTargetPose(self, targets: list[Target], alignRotationOnly: bool = False) -> Command:
    return (
      self._robot.drive.alignToTargetPose(self._robot.localization.getRobotPose, lambda: self._robot.targeting.getNearestTargetPose(targets), alignRotationOnly)
    ).withName("Game:AlignRobotToNearestTargetPose")

  def alignRobotToTargetHeading(self, target: Target) -> Command:
    return (
      self._robot.drive.alignToTargetHeading(self._robot.localization.getRobotPose, lambda: self._robot.targeting.getTargetPose(target))
    ).withName(f'Game:AlignRobotToTargetHeading:{ target.name }')

  def extend(self) -> Command:
    return (
      self._robot.intake.extend()
    ).withName("Game:Extend")

  def load(self) -> Command:
    return (
      self._robot.launcher.retract()
      .andThen(self._robot.intake.run_().until(lambda: self._robot.launcher.isLoaded()))
      .andThen(self.rumbleControllers(ControllerRumbleMode.BOTH))
    ).withName("Game:Load")

  def scoreRight(self) -> Command:
    return (
      cmd.either(
        self.alignRobotToTargetPose(Target.SCORE_RIGHT),
        self.alignRobotToTargetHeading(Target.STABLE_RIGHT).until(lambda: self._robot.drive.isAlignedToTargetHeading()),
        lambda: self._robot.localization.getRobotZone() == Zone.ALLIANCE_ZONE_RIGHT
      )
      .andThen(cmd.waitSeconds(0.5))
      .andThen(self._robot.launcher.launch(lambda: self._robot.targeting.getScoringTargetInfo(Target.STABLE_RIGHT).speed))
      .andThen(self._robot.launcher.retract())
      .andThen(self.rumbleControllers(ControllerRumbleMode.BOTH))
    ).withName("Game:ScoreRight")

  def scoreLeft(self) -> Command:
    return (
      cmd.either(
        self.alignRobotToTargetPose(Target.SCORE_LEFT),
        self.alignRobotToTargetHeading(Target.STABLE_LEFT).until(lambda: self._robot.drive.isAlignedToTargetHeading()),
        lambda: self._robot.localization.getRobotZone() == Zone.ALLIANCE_ZONE_LEFT
      )
      .andThen(cmd.waitSeconds(0.5))
      .andThen(self._robot.launcher.launch(lambda: self._robot.targeting.getScoringTargetInfo(Target.STABLE_LEFT).speed))
      .andThen(self._robot.launcher.retract())
      .andThen(self.rumbleControllers(ControllerRumbleMode.BOTH))
    ).withName("Game:ScoreLeft")

  def hold(self) -> Command:
    return (
      self._robot.intake.hold()
      .until(lambda: self._robot.intake.isHolding())
    ).withName("Game:Hold")

  def place(self) -> Command:
    return (
      self._robot.intake.place()
    ).withName("Game:Place")

  def testLauncher(self) -> Command:
    return (
      self._robot.launcher.launch(lambda: SmartDashboard.getNumber("Robot/Launcher/TestSpeed", 0))
      .onlyIf(lambda: self.isReadyToLaunch())
      .andThen(self._robot.launcher.retract())
      .andThen(self.rumbleControllers(ControllerRumbleMode.BOTH))
    ).withName("Game:TestLauncher")

  def isReadyToLaunch(self) -> bool:
    return self._robot.intake.isExtended() and self._robot.launcher.isLoaded()
  
  def resetGyro(self) -> Command:
    return (
      self._robot.gyro.reset()
      .andThen(self.rumbleControllers(ControllerRumbleMode.DRIVER))
      .ignoringDisable(True)
    ).withName("Game:ResetGyro")

  def rumbleControllers(
    self, 
    mode: ControllerRumbleMode = ControllerRumbleMode.BOTH, 
    pattern: ControllerRumblePattern = ControllerRumblePattern.SHORT
  ) -> Command:
    return (
      cmd.parallel(
        self._robot.driver.rumble(pattern).onlyIf(lambda: mode != ControllerRumbleMode.OPERATOR),
        self._robot.operator.rumble(pattern).onlyIf(lambda: mode != ControllerRumbleMode.DRIVER)
      )
      .onlyIf(lambda: RobotBase.isReal() and not utils.isAutonomousMode())
    ).withName(f'Game:RumbleControllers:{ mode.name }:{ pattern.name }')
