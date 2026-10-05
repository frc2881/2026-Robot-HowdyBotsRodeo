from enum import Enum, auto
from dataclasses import dataclass
from wpimath import units

class AutoPath(Enum):
  CUSTOM = auto()

class Target(Enum):
  STABLE_LEFT = auto()
  STABLE_RIGHT = auto()
  HAYBINE_LEFT = auto()
  HAYBINE_RIGHT = auto()
  CROP_CIRCLE_LEFT = auto()
  CROP_CIRCLE_RIGHT = auto()

class Zone(Enum):
  ALLIANCE_ZONE_RIGHT = auto()
  ALLIANCE_ZONE_LEFT = auto()

@dataclass(frozen=True, slots=True)
class LaunchMetric:
  distance: units.meters
  speed: units.percent

@dataclass(frozen=False, slots=True)
class TargetInfo:
  distance: units.meters = 0
  speed: units.percent = 0
  heading: units.degrees = 0

class MatchState(Enum):
  STOPPED = auto()
  AUTO = auto()
  TELEOP = auto()
  END_GAME = auto()

class LightsMode(Enum):
  DEFAULT = auto()
  ROBOT_NOT_CONNECTED = auto()
  ROBOT_NOT_HOMED = auto()
  ROBOT_IS_HOMING = auto()
  VISION_NOT_READY = auto()