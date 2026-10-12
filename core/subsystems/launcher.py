from typing import Callable
from wpilib import SmartDashboard
from wpimath import units
from commands2 import Subsystem, Command
from lib import logger, telemetry, utils
from lib.classes import IdleMode
from lib.modules.catapult import CatapultModule
from lib.modules.follower_control import FollowerControlModule
import core.constants as constants

class Launcher(Subsystem):
  def __init__(
      self,
      getLauncherSensorHasTarget: Callable[[], bool]
    ) -> None:
    super().__init__()
    self._constants = constants.Subsystems.Launcher
    self._getLauncherSensorHasTarget = getLauncherSensorHasTarget

    self._telemetryName = "Robot/Subsystems/Launcher"

    self._launcherLeader = CatapultModule(self._constants.LAUNCHER_LEADER_CONFIG)
    self._launcherFollower = FollowerControlModule(self._constants.LAUNCHER_FOLLOWER_CONFIG)

    self.setDefaultCommand(self._launcherLeader.retract(self))

    SmartDashboard.putNumber("Robot/Launcher/TestSpeed", 0.3)

  def periodic(self) -> None:
    self._updateTelemetry()

  def launch(self, getSpeed: Callable[[], units.percent]) -> Command:
    return self._launcherLeader.launch(lambda: getSpeed(), self)

  def test(self) -> Command:
    return self._launcherLeader.launch(lambda: SmartDashboard.getNumber("Robot/Launcher/TestSpeed", 0), self)

  def isReady(self) -> bool:
    return self._getLauncherSensorHasTarget()

  def isRetracted(self) -> bool:
    return self._launcherLeader.isRetracted()

  def resetToHome(self) -> Command:
    return self._launcherLeader.resetToHome(self).withName("Launcher:ResetToHome")

  def isHoming(self) -> bool:
    return self._launcherLeader.isHoming()

  def isHomed(self) -> bool:
    return self._launcherLeader.isHomed()

  def reset(self) -> None:
    self._launcherLeader.reset()

  def _updateTelemetry(self) -> None:
    telemetry.log(f'{self._telemetryName}/IsRetracted', self.isRetracted())