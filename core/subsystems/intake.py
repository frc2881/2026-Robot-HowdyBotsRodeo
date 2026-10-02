from wpimath import units
from commands2 import Subsystem, Command
from lib import logger, telemetry, utils
from lib.components.relative_position_control_module import RelativePositionControlModule
from lib.components.velocity_control_module import VelocityControlModule
from lib.components.follower_control_module import FollowerControlModule
import core.constants as constants

# TODO: implement intake subsystem once designed in CAD
class Intake(Subsystem):
  def __init__(self) -> None:
    super().__init__()
    self._constants = constants.Subsystems.Intake

    self._telemetryName = "Robot/Subsystems/Intake"

    self._armLeader = RelativePositionControlModule(self._constants.ARM_LEADER_CONFIG)
    self._armFollower = FollowerControlModule(self._constants.ARM_FOLLOWER_CONFIG)
    self._rollersTop = VelocityControlModule(self._constants.ROLLERS_TOP_CONFIG)
    self._rollersBottom = VelocityControlModule(self._constants.ROLLERS_BOTTOM_CONFIG)

  def periodic(self) -> None:
    self._updateTelemetry()

  def run_(self) -> Command:
    return self.startEnd(
      lambda: self._run(),
      lambda: self.reset()
    )

  # TODO: implement run logic for arm and rollers
  def _run(self) -> None:
    pass

  def isExtended(self) -> bool:
    return self._armLeader.getPosition() > self._constants.ARM_INTAKE_POSITION * 0.75
  
  def isRunning(self) -> bool:
    return self._rollersTop.getSpeed() > 0.1

  def resetToHome(self) -> Command:
    return self._armLeader.resetToHome(self).withName("Intake:ResetToHome")

  def isHoming(self) -> bool:
    return self._armLeader.isHoming()

  def isHomed(self) -> bool:
    return self._armLeader.isHomed()

  def reset(self) -> None:
    self._armLeader.reset()
    self._rollersTop.reset()
    self._rollersBottom.reset()

  def _updateTelemetry(self) -> None:
    telemetry.log(f'{self._telemetryName}/IsExtended', self.isExtended())
    telemetry.log(f'{self._telemetryName}/IsRunning', self.isRunning())