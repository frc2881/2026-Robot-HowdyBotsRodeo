from wpimath import units
from commands2 import Subsystem, Command
from rev import SparkBaseConfig
from lib import logger, telemetry, utils
from lib.modules.velocity_control import VelocityControlModule
from lib.modules.follower_control import FollowerControlModule
import core.constants as constants

class Launcher(Subsystem):
  def __init__(self) -> None:
    super().__init__()
    self._constants = constants.Subsystems.Launcher

    self._telemetryName = "Robot/Subsystems/Launcher"

    self._launcherLeader = VelocityControlModule(self._constants.LAUNCHER_LEADER_CONFIG)
    self._launcherFollower = FollowerControlModule(self._constants.LAUNCHER_FOLLOWER_CONFIG)

    sparkConfig = SparkBaseConfig()
    (sparkConfig.softLimit
      .forwardSoftLimit(self._constants.FORWARD_SOFT_LIMIT)
      .forwardSoftLimitEnabled(True)
      .reverseSoftLimit(self._constants.REVERSE_SOFT_LIMIT)
      .reverseSoftLimitEnabled(True)
    )
    utils.configureSparkController(self._launcherLeader._controller, sparkConfig, isPersisted = True)

  def periodic(self) -> None:
    self._updateTelemetry()

  def launch(self, speed: units.percent) -> Command:
    return (
      self.run(lambda: self._launcherLeader.setSpeed(speed)).until(lambda: self._launcherLeader._controller.getForwardSoftLimit().isReached())
      .andThen(self.run(lambda: self._launcherLeader.setSpeed(self._constants.RESET_SPEED)).until(lambda: self.isReset()))
      .finallyDo(lambda end: self._launcherLeader.setSpeed(self._constants.HOLD_SPEED))
    )

  def isReset(self) -> bool:
    return self._launcherLeader._encoder.getPosition() <= self._constants.REVERSE_SOFT_LIMIT

  def reset(self) -> None:
    self._launcherLeader.reset()

  def _updateTelemetry(self) -> None:
    pass