from commands2 import Subsystem, Command
from wpimath import units
from lib import logger, telemetry, utils
import core.constants as constants

# TODO: implement intake subsystem once designed in CAD
class Intake(Subsystem):
  def __init__(self) -> None:
    super().__init__()
    self._constants = constants.Subsystems.Intake

  def periodic(self) -> None:
    self._updateTelemetry()

  def reset(self) -> None:
    pass

  def _updateTelemetry(self) -> None:
    pass