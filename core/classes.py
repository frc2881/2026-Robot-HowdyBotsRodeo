from enum import Enum, auto
from dataclasses import dataclass
from wpimath import units

class Target(Enum):
  StableLeft = auto()
  StableRight = auto()
  HaybineLeft = auto()
  HaybineRight = auto()
  CropCircleLeft = auto()
  CropCircleRight = auto()

@dataclass(frozen=True, slots=True)
class LaunchMetric:
  distance: units.meters
  speed: units.percent
