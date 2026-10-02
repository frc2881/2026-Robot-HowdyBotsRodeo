from wpimath import units
from commands2 import Subsystem, Command
from rev import SparkBaseConfig
from lib import logger, telemetry, utils
from lib.components.velocity_control_module import VelocityControlModule
from lib.components.follower_control_module import FollowerControlModule
import core.constants as constants

# TODO: implement launcher subsystem once designed in CAD
class Launcher(Subsystem):
  def __init__(self) -> None:
    super().__init__()
    self._constants = constants.Subsystems.Launcher

    self._telemetryName = "Robot/Subsystems/Launcher"

    self._launcherLeader = VelocityControlModule(self._constants.LAUNCHER_LEADER_CONFIG)
    self._launcherFollower = FollowerControlModule(self._constants.LAUNCHER_FOLLOWER_CONFIG)

    sparkConfig = SparkBaseConfig()
    (sparkConfig.softLimit
      .reverseSoftLimitEnabled(True)
      .reverseSoftLimit(0)
      .forwardSoftLimitEnabled(True)
      .forwardSoftLimit(10)
    )
    utils.configureSparkController(self._launcherLeader._controller, sparkConfig, isPersisted = True)

  def periodic(self) -> None:
    self._updateTelemetry()

  # TODO: implement launch command with a check for forward soft limit reached, pauses, and then resets the catapult for next launch

  def reset(self) -> None:
    self._launcherLeader.reset()

  def _updateTelemetry(self) -> None:
    pass