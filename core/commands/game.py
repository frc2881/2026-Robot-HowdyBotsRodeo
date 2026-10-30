from typing import TYPE_CHECKING
from wpilib import RobotBase, SmartDashboard
from commands2 import Command, cmd
from lib import logger, telemetry, utils
from lib.classes import Alignment, ControllerRumbleMode, ControllerRumblePattern
from core.classes import Target, Zone
import core.constants as constants
if TYPE_CHECKING: from core.robot import RobotCore

class Game:
  def __init__(self, robot: "RobotCore") -> None:
    self._robot = robot

    SmartDashboard.putNumber("Robot/Launcher/TestSpeed", 0.4)

  def alignRobotToTargetPose(self, target: Target, alignment = Alignment.POSE) -> Command:
    return (
      self._robot.drive.alignToTargetPose(lambda: self._robot.localization.getRobotPose(), lambda: self._robot.targeting.getTargetPose(target), alignment)
    ).withName(f'Game:AlignRobotToTargetPose:{ target.name }')

  def alignRobotToNearestTargetPose(self, targets: list[Target], alignment = Alignment.POSE) -> Command:
    return (
      self._robot.drive.alignToTargetPose(lambda: self._robot.localization.getRobotPose(), lambda: self._robot.targeting.getNearestTargetPose(targets), alignment)
    ).withName("Game:AlignRobotToNearestTargetPose")

  def lockRobotToTargetHeading(self, target: Target) -> Command:
    return (
      self._robot.drive.lockToTargetHeading(lambda: self._robot.localization.getRobotPose(), lambda: self._robot.targeting.getTargetPose(target))
    ).withName(f'Game:LockRobotToTargetHeading:{ target.name }')

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
      cmd.select({
        Zone.ALLIANCE_ZONE_RIGHT: self.alignRobotToTargetPose(Target.SCORE_RIGHT, Alignment.POSE),
        Zone.ALLIANCE_ZONE_LEFT: self.alignRobotToTargetPose(Target.STABLE_RIGHT, Alignment.HEADING)
      }, lambda: self._robot.localization.getRobotZone())
      .andThen(cmd.waitSeconds(0.75))
      .andThen(self._robot.launcher.launch(lambda: self._robot.targeting.getScoringTargetInfo(Target.STABLE_RIGHT).speed))
      .andThen(self._robot.launcher.retract())
      .andThen(self.rumbleControllers(ControllerRumbleMode.BOTH))
    ).withName("Game:ScoreRight")

  def scoreLeft(self) -> Command:
    return (
      cmd.select({
        Zone.ALLIANCE_ZONE_LEFT: self.alignRobotToTargetPose(Target.SCORE_LEFT, Alignment.POSE),
        Zone.ALLIANCE_ZONE_RIGHT: self.alignRobotToTargetPose(Target.STABLE_LEFT, Alignment.HEADING)
      }, lambda: self._robot.localization.getRobotZone())
      .andThen(cmd.waitSeconds(0.75))
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
