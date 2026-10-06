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

  def runIntake(self) -> Command:
    return (
      self._robot.intake.run_()
      .onlyIf(lambda: self._robot.launcher.isReset())
      # .until(lambda: "launcher has hay ready for launch based on sensor state")
      .withName("Game:RunIntake")
    )

  def scoreHay(self, target: Target) -> Command:
    return (
      self.alignRobotToTargetHeading(target).until(lambda: self._robot.drive.isAlignedToTargetHeading())
      .andThen(self._robot.launcher.launch(self._robot.targeting.getScoringTargetInfo(target).speed))
      .andThen(self.rumbleControllers(ControllerRumbleMode.BOTH))
      .withName("Game:LaunchHay")
    )
  
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
