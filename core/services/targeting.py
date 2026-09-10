from typing import TYPE_CHECKING, Callable, Optional
from wpilib import SmartDashboard
from wpimath import units
from wpimath.geometry import Pose2d, Pose3d
from lib import logger, utils
from lib.classes import Alliance
from core.classes import Target
import core.constants as constants

class Targeting():
  def __init__(
      self,
      getRobotPose: Callable[[], Pose2d]
    ) -> None:
    self._constants = constants.Services.Targeting
    self._getRobotPose = getRobotPose

    self._alliance: Optional[Alliance] = None
    self._targets: dict[Target, Pose3d] = {}

    self._launchDistances = tuple(t.distance for t in self._constants.LAUNCH_METRICS)
    self._launchSpeeds = tuple(t.speed for t in self._constants.LAUNCH_METRICS)

    utils.addRobotPeriodic(self._periodic)

  def _periodic(self) -> None:
    self._updateTargets()
    self._updateTelemetry()

  def _updateTargets(self) -> None:
    if utils.getAlliance() != self._alliance:
      self._alliance = utils.getAlliance()
      self._targets = constants.Game.Field.Targets.TARGETS[self._alliance]

  def getTargetPose(self, target: Target) -> Pose3d:
    return self._targets.get(target, Pose3d(self._getRobotPose()))
  
  def getNearestTargetPose(self, targets: list[Target]) -> Pose3d:
    return Pose3d(self._getRobotPose()).nearest([self._targets[target] for target in self._targets if target in targets])

  def _updateTelemetry(self) -> None:
    pass
