from typing import TYPE_CHECKING
from wpilib import SendableChooser, SmartDashboard
from wpimath.geometry import Transform2d, Rotation2d
from commands2 import Command, cmd
from pathplannerlib.auto import AutoBuilder
from pathplannerlib.path import PathPlannerPath, PathConstraints, GoalEndState
from lib import logger, telemetry, utils
from lib.classes import Alliance
from core.classes import AutoPath, Target
import core.constants as constants
if TYPE_CHECKING: from core.robot import RobotCore

class Auto:
  def __init__(self, robot: "RobotCore") -> None:
    self._robot = robot

    self._paths = { path: PathPlannerPath.fromPathFile(path.name) for path in AutoPath }
    self._auto = cmd.none()

    AutoBuilder.configure(
      self._robot.localization.getRobotPose, 
      self._robot.localization.resetRobotPose,
      self._robot.drive.getChassisSpeeds, 
      self._robot.drive.setChassisSpeeds, 
      constants.Subsystems.Drive.PATHPLANNER_CONTROLLER,
      constants.Subsystems.Drive.PATHPLANNER_ROBOT_CONFIG,
      lambda: utils.getAlliance() == Alliance.RED,
      self._robot.drive
    )

    self._autos = SendableChooser()
    self._autos.setDefaultOption("0: None", self.auto_NONE)
    self._autos.addOption("1: Right Score 2", self.auto_RIGHT_SCORE_2)
    self._autos.addOption("2: Right Score 3", self.auto_RIGHT_SCORE_3)
    self._autos.addOption("3: Right Score 4", self.auto_RIGHT_SCORE_4)
    self._autos.addOption("4: Left Score 2", self.auto_LEFT_SCORE_2)
    self._autos.addOption("5: Left Score 3", self.auto_LEFT_SCORE_3)
    self._autos.addOption("6: Left Score 4", self.auto_LEFT_SCORE_4)
    # self._autos.addOption("99: Custom", self.auto_CUSTOM)

    self._autos.onChange(lambda auto: self.set(auto()))
    SmartDashboard.putData("Robot/Auto", self._autos)

  def get(self) -> Command:
    return self._auto
  
  def set(self, auto: Command) -> None:
    self._auto = auto
    telemetry.log("Robot/Auto/command", auto.getName().replace("Auto:", ""))

  def _getPath(self, path: AutoPath) -> PathPlannerPath:
    return self._paths.get(path, PathPlannerPath([], PathConstraints(0, 0, 0, 0), None, GoalEndState(0, Rotation2d())))
  
  def _resetRobot(self, path: AutoPath) -> Command:
    return (
      AutoBuilder.resetOdom(self._getPath(path).getPathPoses()[0].transformBy(Transform2d(0, 0, self._getPath(path).getInitialHeading())))
      .andThen(cmd.waitSeconds(0.1))
    ).deadlineFor(logger.log_("Auto:Reset"))
  
  def followPath(self, path: AutoPath) -> Command:
    return (
      AutoBuilder.followPath(self._getPath(path))
    ).deadlineFor(logger.log_(f'Auto:Move:{path.name}'))
  
  def auto_NONE(self) -> Command:
    return cmd.none().withName("Auto:NONE")

  def auto_RIGHT_SCORE_2(self) -> Command:
    return cmd.sequence(
      self._robot.game.scoreHayFromLauncher(Target.STABLE_RIGHT),
      self.followPath(AutoPath.RIGHT_PICKUP_1).deadlineFor(self._robot.game.loadHayIntoLauncher()),
      self._robot.game.alignRobotToTargetPose(Target.CROP_CIRCLE_RIGHT),
      self._robot.game.scoreHayFromLauncher(Target.STABLE_RIGHT)
    ).withName("Auto:RIGHT_SCORE_2")

  def auto_RIGHT_SCORE_3(self) -> Command:
    return cmd.sequence(
      self.auto_RIGHT_SCORE_2(),
      self.followPath(AutoPath.RIGHT_PICKUP_2).deadlineFor(self._robot.game.loadHayIntoLauncher()),
      self._robot.game.alignRobotToTargetPose(Target.CROP_CIRCLE_RIGHT),
      self._robot.game.scoreHayFromLauncher(Target.STABLE_RIGHT)
    ).withName("Auto:RIGHT_SCORE_3")

  def auto_RIGHT_SCORE_4(self) -> Command:
    return cmd.sequence(
      self.auto_RIGHT_SCORE_3(),
      self.followPath(AutoPath.RIGHT_PICKUP_3).deadlineFor(self._robot.game.loadHayIntoLauncher()),
      self._robot.game.alignRobotToTargetPose(Target.CROP_CIRCLE_RIGHT),
      self._robot.game.scoreHayFromLauncher(Target.STABLE_RIGHT)
    ).withName("Auto:RIGHT_SCORE_4")

  def auto_LEFT_SCORE_2(self) -> Command:
    return cmd.sequence(
      self._robot.game.scoreHayFromLauncher(Target.STABLE_LEFT),
      self.followPath(AutoPath.LEFT_PICKUP_1).deadlineFor(self._robot.game.loadHayIntoLauncher()),
      self._robot.game.alignRobotToTargetPose(Target.CROP_CIRCLE_LEFT),
      self._robot.game.scoreHayFromLauncher(Target.STABLE_LEFT)
    ).withName("Auto:LEFT_SCORE_2")

  def auto_LEFT_SCORE_3(self) -> Command:
    return cmd.sequence(
      self.auto_LEFT_SCORE_2(),
      self.followPath(AutoPath.LEFT_PICKUP_2).deadlineFor(self._robot.game.loadHayIntoLauncher()),
      self._robot.game.alignRobotToTargetPose(Target.CROP_CIRCLE_LEFT),
      self._robot.game.scoreHayFromLauncher(Target.STABLE_LEFT)
    ).withName("Auto:LEFT_SCORE_3")

  def auto_LEFT_SCORE_4(self) -> Command:
    return cmd.sequence(
      self.auto_LEFT_SCORE_3(),
      self.followPath(AutoPath.LEFT_PICKUP_3).deadlineFor(self._robot.game.loadHayIntoLauncher()),
      self._robot.game.alignRobotToTargetPose(Target.CROP_CIRCLE_LEFT),
      self._robot.game.scoreHayFromLauncher(Target.STABLE_LEFT)
    ).withName("Auto:LEFT_SCORE_4")

  def auto_CUSTOM(self) -> Command:
    return cmd.sequence(
      self.followPath(AutoPath.CUSTOM)
    ).withName("Auto:CUSTOM")
