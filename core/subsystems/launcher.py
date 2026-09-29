from wpimath import units
from commands2 import Subsystem, Command
from lib import logger, telemetry, utils
import core.constants as constants

# TODO: implement launcher subsystem once designed in CAD
class Launcher(Subsystem):
  def __init__(self) -> None:
    super().__init__()
    self._constants = constants.Subsystems.Launcher

    self._telemetryName = "Robot/Subsystems/Launcher"

  def periodic(self) -> None:
    self._updateTelemetry()

  def reset(self) -> None:
    pass

  def _updateTelemetry(self) -> None:
    pass