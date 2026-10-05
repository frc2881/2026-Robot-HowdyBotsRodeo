from typing import TYPE_CHECKING, Callable, Optional
from wpimath.geometry import Pose2d, Pose3d
from wpimath.kinematics import ChassisSpeeds
from lib import logger, telemetry, utils
from lib.classes import Alliance
from core.classes import Target, Zone, TargetInfo
import core.constants as constants

class Targeting():
  def __init__(
      self,
      getRobotPose: Callable[[], Pose2d],
      getRobotZone: Callable[[], Optional[Zone]],
      getChassisSpeeds: Callable[[], ChassisSpeeds]
    ) -> None:
    self._constants = constants.Services.Targeting
    self._getRobotPose = getRobotPose
    self._getRobotZone = getRobotZone
    self._getChassisSpeeds = getChassisSpeeds

    self._telemetryName = "Robot/Services/Targeting"

    self._alliance: Optional[Alliance] = None
    self._targets: dict[Target, Pose3d] = {}

    self._scoringTargets = [ Target.STABLE_RIGHT, Target.STABLE_LEFT ]
    self._scoringTargetInfos = dict((target, TargetInfo()) for target in self._scoringTargets)

    self._launchDistances = tuple(t.distance for t in self._constants.LAUNCH_METRICS)
    self._launchSpeeds = tuple(t.speed for t in self._constants.LAUNCH_METRICS)

    utils.addRobotPeriodic(self._periodic)

  def _periodic(self) -> None:
    self._updateTargets()
    self._updateScoringTargetInfos()
    self._updateTelemetry()

  def _updateTargets(self) -> None:
    if utils.getAlliance() != self._alliance:
      self._alliance = utils.getAlliance()
      self._targets = constants.Game.Field.TARGETS[self._alliance]

  def _updateScoringTargetInfos(self) -> None:
    robotPose = self._getRobotPose()
    for scoringTarget in self._scoringTargets:
      targetPose = self.getTargetPose(scoringTarget)
      distance = utils.getTargetDistance(robotPose, targetPose)
      self._scoringTargetInfos[scoringTarget].distance = distance
      self._scoringTargetInfos[scoringTarget].speed = utils.getInterpolatedValue(distance, self._launchDistances, self._launchSpeeds)
      self._scoringTargetInfos[scoringTarget].heading = utils.getTargetHeading(robotPose, targetPose)

  def getTargetPose(self, target: Target) -> Pose3d:
    return self._targets.get(target, Pose3d(self._getRobotPose()))
  
  def getNearestTargetPose(self, targets: list[Target]) -> Pose3d:
    return Pose3d(self._getRobotPose()).nearest([self._targets[target] for target in self._targets if target in targets])

  def getScoringTargetInfo(self, scoringTarget: Target) -> TargetInfo:
    return self._scoringTargetInfos[scoringTarget]

  def _updateTelemetry(self) -> None:
    for scoringTarget in self._scoringTargets:
      telemetry.log(f'{self._telemetryName}/Targets/{scoringTarget.name}/Distance', self._scoringTargetInfos[scoringTarget].distance)
      telemetry.log(f'{self._telemetryName}/Targets/{scoringTarget.name}/Speed', self._scoringTargetInfos[scoringTarget].speed)
      telemetry.log(f'{self._telemetryName}/Targets/{scoringTarget.name}/Heading', self._scoringTargetInfos[scoringTarget].heading)
