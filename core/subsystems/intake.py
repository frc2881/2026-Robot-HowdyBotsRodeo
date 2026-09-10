from commands2 import Subsystem, Command
from wpilib import SmartDashboard
from wpimath import units
from lib import logger, utils
import core.constants as constants

class Intake(Subsystem):
  def __init__(self) -> None:
    super().__init__()
    self._constants = constants.Subsystems.Launcher

  def periodic(self) -> None:
    self._updateTelemetry()

  def reset(self) -> None:
    pass

  def _updateTelemetry(self) -> None:
    pass