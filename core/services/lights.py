from typing import Callable
from wpilib import AddressableLED, LEDPattern, Color, DriverStation
from lib import logger, telemetry, utils
from lib.classes import RobotState
from core.classes import LightsMode
import core.constants as constants

class Lights():
  def __init__(
      self,
      isHoming: Callable[[], bool],
      isHomed: Callable[[], bool],
      hasValidPoseSensorResult: Callable[[], bool],
      isLaunchReady: Callable[[], bool]
    ) -> None:
    self._constants = constants.Services.Lights
    self._isHoming = isHoming
    self._isHomed = isHomed
    self._hasValidPoseSensorResult = hasValidPoseSensorResult
    self._isLaunchReady = isLaunchReady

    self._telemetryName = "Robot/Services/Lights"

    self._lightsMode = LightsMode.DEFAULT

    self._ledColors: dict[LightsMode, Color] = {
      LightsMode.DEFAULT: Color(255, 20, 147),
      LightsMode.ROBOT_NOT_CONNECTED: Color.kWhite,
      LightsMode.ROBOT_NOT_HOMED: Color.kRed,
      LightsMode.ROBOT_IS_HOMING: Color.kYellow,
      LightsMode.VISION_NOT_READY: Color.kBlue,
      LightsMode.LAUNCH_READY: Color.kGreen
    }

    self._led = AddressableLED(self._constants.LED_CHANNEL)
    self._ledData = [ AddressableLED.LEDData(255, 20, 147) for _ in range(self._constants.LED_LENGTH) ]
    self._led.setLength(self._constants.LED_LENGTH)
    self._led.setData(self._ledData)
    self._led.start()

    utils.addRobotPeriodic(self._periodic)

  def _periodic(self) -> None:
    self._updateLightsMode()
    self._updateLights()
    self._updateTelemetry()

  def _updateLightsMode(self) -> None:
    if not DriverStation.isDSAttached():
      self._lightsMode = LightsMode.ROBOT_NOT_CONNECTED
      return
    
    if utils.getRobotState() == RobotState.DISABLED:
      if self._isHoming():
        self._lightsMode = LightsMode.ROBOT_IS_HOMING
        return
      if not self._isHomed():
        self._lightsMode = LightsMode.ROBOT_NOT_HOMED
        return
      if not self._hasValidPoseSensorResult():
        self._lightsMode = LightsMode.VISION_NOT_READY
        return
      
    if utils.getRobotState() == RobotState.ENABLED:
      if self._isLaunchReady():
        self._lightsMode = LightsMode.LAUNCH_READY
        return
      
    self._lightsMode = LightsMode.DEFAULT

  def _updateLights(self) -> None:
    for i in range(self._constants.LED_LENGTH):
      self._ledData[i].setLED(self._ledColors[self._lightsMode])
    self._led.setData(self._ledData)

  def _updateTelemetry(self) -> None:
    telemetry.log(f'{self._telemetryName}/Mode', self._lightsMode.name)