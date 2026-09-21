from enum import Enum, auto
from dataclasses import dataclass
from wpimath import units

class AutoPath(Enum):
  CUSTOM = auto()

class Target(Enum):
  StableLeft = auto()
  StableRight = auto()
  HaybineLeft = auto()
  HaybineRight = auto()
  CropCircleLeft = auto()
  CropCircleRight = auto()

class Zone(Enum):
  AllianceZoneRight = auto()
  AllianceZoneLeft = auto()

@dataclass(frozen=True, slots=True)
class LaunchMetric:
  distance: units.meters
  speed: units.percent

class MatchState(Enum):
  Stopped = auto()
  Auto = auto()
  Teleop = auto()
  EndGame = auto()

class LightsMode(Enum):
  Default = auto()
  RobotNotConnected = auto()
  RobotNotHomed = auto()
  RobotIsHoming = auto()
  VisionNotReady = auto()