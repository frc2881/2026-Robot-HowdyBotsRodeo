from typing import TYPE_CHECKING
from wpilib import RobotBase
from wpimath import units
from commands2 import Command, cmd
from lib import logger, telemetry, utils
from lib.classes import ControllerRumbleMode, ControllerRumblePattern
from core.classes import Target
import core.constants as constants
if TYPE_CHECKING: from core.robot import RobotCore

class Game:
  def __init__(self, robot: "RobotCore") -> None:
    self._robot = robot

  def alignRobotToTargetPose(self, target: Target, alignRotationOnly: bool = False) -> Command:
    return (
      self._robot.drive.alignToTargetPose(self._robot.localization.getRobotPose, lambda: self._robot.targeting.getTargetPose(target), alignRotationOnly)
      .withName(f'Game:AlignRobotToTargetPose:{ target.name }')
    )

  def alignRobotToNearestTargetPose(self, targets: list[Target], alignRotationOnly: bool = False) -> Command:
    return (
      self._robot.drive.alignToTargetPose(self._robot.localization.getRobotPose, lambda: self._robot.targeting.getNearestTargetPose(targets), alignRotationOnly)
      .withName("Game:AlignRobotToNearestTargetPose")
    )

  def alignRobotToTargetHeading(self, target: Target) -> Command:
    return (
      self._robot.drive.alignToTargetHeading(self._robot.localization.getRobotPose, lambda: self._robot.targeting.getTargetPose(target))
      .withName(f'Game:AlignRobotToTargetHeading:{ target.name }')
    )

  def extend(self) -> Command:
    return (
      self._robot.intake.extend()
      .withName("Game:Extend")
    )

  def load(self) -> Command:
    return (
      self._robot.intake.run_()
      .until(lambda: self.isReadyToLaunch())
      .andThen(self.rumbleControllers(ControllerRumbleMode.BOTH))
      .withName("Game:Load")
    )

  def alignToTargetPoseAndScore(self, target: Target) -> Command:
    return (
      (
        self.alignRobotToTargetPose(target)
        .andThen(cmd.waitSeconds(0.5))
        .andThen(
          self._robot.launcher.launch(lambda: self._robot.targeting.getScoringTargetInfo(Target.STABLE_RIGHT if target == Target.SCORE_RIGHT else Target.STABLE_LEFT).speed)
          .deadlineFor(self._robot.intake.run_())
        )
        .andThen(self.rumbleControllers(ControllerRumbleMode.BOTH))
      )
      .withName("Game:Score")
    )

  def alignToTargetHeadingAndScore(self, target: Target) -> Command:
    return (
      (
        self.alignRobotToTargetHeading(target).until(lambda: self._robot.drive.isAlignedToTargetHeading())
        .andThen(cmd.waitSeconds(0.5))
        .andThen(
          self._robot.launcher.launch(lambda: self._robot.targeting.getScoringTargetInfo(target).speed)
          .deadlineFor(self._robot.intake.run_())
        )
        .andThen(self.rumbleControllers(ControllerRumbleMode.BOTH))
      )
      .withName("Game:Score")
    )

  def hold(self) -> Command:
    return (
      self._robot.intake.hold()
      .until(lambda: self._robot.intake.isHolding())
      .withName("Game:Hold")
    )

  def place(self) -> Command:
    return (
      self._robot.intake.place()
      .withName("Game:Place")
    )

  def launchOverride(self) -> Command:
    return (
      self._robot.launcher.test()
      .onlyIf(lambda: self.isReadyToLaunch())
    )

  def _isReadyToLoad(self) -> bool:
    return self._robot.launcher.isRetracted()

  def isReadyToLaunch(self) -> bool:
    return self._robot.launcher.isReady() and self._robot.intake.isExtended()
  
  def resetGyro(self) -> Command:
    return (
      self._robot.gyro.reset()
      .andThen(self.rumbleControllers(ControllerRumbleMode.DRIVER))
      .ignoringDisable(True)
      .withName("Game:ResetGyro")
    )

  def rumbleControllers(
    self, 
    mode: ControllerRumbleMode = ControllerRumbleMode.BOTH, 
    pattern: ControllerRumblePattern = ControllerRumblePattern.SHORT
  ) -> Command:
    return cmd.parallel(
      self._robot.driver.rumble(pattern).onlyIf(lambda: mode != ControllerRumbleMode.OPERATOR),
      self._robot.operator.rumble(pattern).onlyIf(lambda: mode != ControllerRumbleMode.DRIVER)
    ).onlyIf(
      lambda: RobotBase.isReal() and not utils.isAutonomousMode()
    ).withName(f'Game:RumbleControllers:{ mode.name }:{ pattern.name }')
