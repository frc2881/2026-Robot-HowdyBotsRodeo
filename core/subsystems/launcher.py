from wpimath import units
from commands2 import Subsystem, Command
from lib import logger, telemetry, utils
from lib.modules.catapult import CatapultModule
from lib.modules.follower_control import FollowerControlModule
import core.constants as constants

class Launcher(Subsystem):
  def __init__(self) -> None:
    super().__init__()
    self._constants = constants.Subsystems.Launcher

    self._telemetryName = "Robot/Subsystems/Launcher"

    self._launcherLeader = CatapultModule(self._constants.LAUNCHER_LEADER_CONFIG)
    self._launcherFollower = FollowerControlModule(self._constants.LAUNCHER_FOLLOWER_CONFIG)

  def periodic(self) -> None:
    self._updateTelemetry()

  def launch(self, speed: units.percent) -> Command:
    return self._launcherLeader.launch(speed, self)

  def isReset(self) -> bool:
    return self._launcherLeader.isReset()

  def resetToHome(self) -> Command:
    return self._launcherLeader.resetToHome(self).withName("Launcher:ResetToHome")

  def isHoming(self) -> bool:
    return self._launcherLeader.isHoming()

  def isHomed(self) -> bool:
    return self._launcherLeader.isHomed()

  def reset(self) -> None:
    self._launcherLeader.reset()

  def _updateTelemetry(self) -> None:
    telemetry.log(f'{self._telemetryName}/IsReset', self.isReset())