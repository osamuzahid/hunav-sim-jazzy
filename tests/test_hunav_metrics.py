"""Hall space and arena_* regression on synthetic ticks (no Isaac Sim)."""

from __future__ import annotations

import importlib
import math
import sys
import types
from pathlib import Path
from types import SimpleNamespace

import pytest
import yaml

ROOT = Path(__file__).resolve().parents[1]


def _ensure_module(name: str, module: types.ModuleType) -> None:
    if name in sys.modules:
        return
    try:
        importlib.import_module(name)
    except ImportError:
        sys.modules[name] = module


def _install_ros_stubs() -> None:
    """Provide duck-typed ROS modules when rclpy / hunav_msgs / angles are absent."""

    class _Stamp:
        def __init__(self, sec: int = 0, nanosec: int = 0) -> None:
            self.sec = int(sec)
            self.nanosec = int(nanosec)

    class _Duration:
        def __init__(self, nanoseconds: int) -> None:
            self.nanoseconds = int(nanoseconds)
            self.sec = self.nanoseconds // 1_000_000_000
            self.nanosec = self.nanoseconds % 1_000_000_000

    class _Time:
        def __init__(self, nanoseconds: int) -> None:
            self.nanoseconds = int(nanoseconds)

        @classmethod
        def from_msg(cls, stamp: _Stamp) -> "_Time":
            return cls(int(stamp.sec) * 1_000_000_000 + int(stamp.nanosec))

        def __sub__(self, other: "_Time") -> "_Time":
            return _Time(self.nanoseconds - other.nanoseconds)

        def to_msg(self) -> _Duration:
            return _Duration(self.nanoseconds)

    class _Logger:
        def debug(self, *args, **kwargs) -> None:
            return None

    class _Logging:
        @staticmethod
        def get_logger(_name: str) -> _Logger:
            return _Logger()

    rclpy = types.ModuleType("rclpy")
    rclpy.logging = _Logging()
    rclpy.time = types.ModuleType("rclpy.time")
    rclpy.time.Time = _Time
    _ensure_module("rclpy", rclpy)
    _ensure_module("rclpy.logging", types.ModuleType("rclpy.logging"))
    sys.modules["rclpy"].logging = _Logging()
    _ensure_module("rclpy.time", rclpy.time)
    sys.modules["rclpy.time"].Time = _Time

    class _Point:
        def __init__(self, x: float = 0.0, y: float = 0.0, z: float = 0.0) -> None:
            self.x = float(x)
            self.y = float(y)
            self.z = float(z)

    class _Quat:
        def __init__(
            self, x: float = 0.0, y: float = 0.0, z: float = 0.0, w: float = 1.0
        ) -> None:
            self.x = float(x)
            self.y = float(y)
            self.z = float(z)
            self.w = float(w)

    class Pose:
        def __init__(self, x: float = 0.0, y: float = 0.0, yaw: float = 0.0) -> None:
            self.position = _Point(x, y, 0.0)
            half = yaw / 2.0
            self.orientation = _Quat(0.0, 0.0, math.sin(half), math.cos(half))

    geometry_msgs = types.ModuleType("geometry_msgs")
    geometry_msgs.msg = types.ModuleType("geometry_msgs.msg")
    geometry_msgs.msg.Pose = Pose
    _ensure_module("geometry_msgs", geometry_msgs)
    _ensure_module("geometry_msgs.msg", geometry_msgs.msg)
    sys.modules["geometry_msgs.msg"].Pose = Pose

    class AgentBehavior:
        def __init__(self, type_: int = 1, state: int = 0) -> None:
            self.type = int(type_)
            self.state = int(state)

    class Agent:
        PERSON = 1
        ROBOT = 2
        OTHER = 3

        def __init__(
            self,
            x: float = 0.0,
            y: float = 0.0,
            yaw: float = 0.0,
            radius: float = 0.4,
            linear_vel: float = 0.0,
            agent_type: int = 1,
        ) -> None:
            self.id = 0
            self.type = int(agent_type)
            self.position = Pose(x, y, yaw)
            self.yaw = float(yaw)
            self.radius = float(radius)
            self.linear_vel = float(linear_vel)
            self.goals = []
            self.group_id = -1
            self.behavior = AgentBehavior()

    class Agents:
        def __init__(self, agents=None, sec: int = 0, nanosec: int = 0) -> None:
            self.header = SimpleNamespace(stamp=_Stamp(sec, nanosec))
            self.agents = list(agents) if agents else []

    hunav_msgs = types.ModuleType("hunav_msgs")
    hunav_msgs.msg = types.ModuleType("hunav_msgs.msg")
    hunav_msgs.msg.Agent = Agent
    hunav_msgs.msg.Agents = Agents
    _ensure_module("hunav_msgs", hunav_msgs)
    _ensure_module("hunav_msgs.msg", hunav_msgs.msg)
    if not hasattr(sys.modules["hunav_msgs.msg"], "Agent"):
        sys.modules["hunav_msgs.msg"].Agent = Agent
        sys.modules["hunav_msgs.msg"].Agents = Agents

    angles = types.ModuleType("angles")

    def normalize_angle(angle: float) -> float:
        a = math.fmod(angle + math.pi, 2.0 * math.pi)
        if a < 0.0:
            a += 2.0 * math.pi
        return a - math.pi

    def shortest_angular_distance(from_angle: float, to_angle: float) -> float:
        return normalize_angle(to_angle - from_angle)

    angles.normalize_angle = normalize_angle
    angles.shortest_angular_distance = shortest_angular_distance
    _ensure_module("angles", angles)


_install_ros_stubs()

from hunav_evaluator import hunav_metrics  # noqa: E402
from hunav_msgs.msg import Agent, Agents  # noqa: E402
from geometry_msgs.msg import Pose  # noqa: E402

# Hall / Arena constants used by hunav_metrics
HALL_INTIMATE_M = 0.45
HALL_PERSONAL_M = 1.2
HALL_SOCIAL_M = 3.6
RADIUS = 0.4


def _pose(x: float, y: float, yaw: float = 0.0) -> Pose:
    pose = Pose()
    pose.position.x = float(x)
    pose.position.y = float(y)
    pose.position.z = 0.0
    if not hasattr(pose, "orientation") or pose.orientation is None:
        pose.orientation = SimpleNamespace(x=0.0, y=0.0, z=0.0, w=1.0)
    half = yaw / 2.0
    pose.orientation.x = 0.0
    pose.orientation.y = 0.0
    pose.orientation.z = math.sin(half)
    pose.orientation.w = math.cos(half)
    return pose


def _agent(
    x: float,
    y: float,
    yaw: float = 0.0,
    radius: float = RADIUS,
    linear_vel: float = 0.0,
    agent_type: int = Agent.PERSON,
) -> Agent:
    a = Agent()
    a.position = _pose(x, y, yaw)
    a.yaw = float(yaw)
    a.radius = float(radius)
    a.linear_vel = float(linear_vel)
    a.type = int(agent_type)
    if not hasattr(a, "goals") or a.goals is None:
        a.goals = []
    if not hasattr(a, "group_id"):
        a.group_id = -1
    return a


def _tick(person_x: float, person_y: float, robot_x: float, robot_y: float,
          sec: int, robot_yaw: float = 0.0, person_yaw: float = math.pi,
          robot_speed: float = 0.0) -> tuple[Agents, Agent]:
    person = _agent(person_x, person_y, yaw=person_yaw)
    robot = _agent(robot_x, robot_y, yaw=robot_yaw, linear_vel=robot_speed,
                   agent_type=Agent.ROBOT)
    agents = Agents()
    agents.header = SimpleNamespace(stamp=SimpleNamespace(sec=sec, nanosec=0))
    agents.agents = [person]
    return agents, robot


def _hall_series() -> tuple[list, list]:
    """Five ticks: Arena-private+intimate, intimate, personal, social, public.

    Robot at origin, radius 0.4; person radius 0.4. Surface gap = cc - 0.8.
    """
    # centre-to-centre distances
    ccs = (
        0.30,  # Arena private (<0.5) and Hall intimate (gap clamped to 0)
        0.90,  # intimate (gap 0.10) not Arena private
        1.60,  # personal (gap 0.80)
        2.80,  # social (gap 2.00)
        5.80,  # public (gap 5.00)
    )
    speeds = (1.2, 0.8, 0.3, 0.3, 0.3)
    agents_list = []
    robot_list = []
    for i, (cc, spd) in enumerate(zip(ccs, speeds)):
        ag, rob = _tick(cc, 0.0, 0.0, 0.0, sec=i, robot_speed=spd)
        agents_list.append(ag)
        robot_list.append(rob)
    return agents_list, robot_list


def test_indicator_function_space_hall_bands() -> None:
    st = hunav_metrics.SpaceType
    assert hunav_metrics.indicator_function_space(0.0, st.INTIMATE)
    assert hunav_metrics.indicator_function_space(0.449, st.INTIMATE)
    assert not hunav_metrics.indicator_function_space(0.45, st.INTIMATE)
    assert hunav_metrics.indicator_function_space(0.45, st.PERSONAL)
    assert hunav_metrics.indicator_function_space(1.199, st.PERSONAL)
    assert hunav_metrics.indicator_function_space(1.2, st.SOCIAL)
    assert hunav_metrics.indicator_function_space(3.599, st.SOCIAL)
    assert hunav_metrics.indicator_function_space(3.6, st.PUBLIC)


def test_hall_space_intrusions_percentages() -> None:
    agents, robot = _hall_series()
    intimate = hunav_metrics.intimate_space_intrusions(agents, robot)[0]
    personal = hunav_metrics.personal_space_intrusions(agents, robot)[0]
    social = hunav_metrics.social_space_intrusions(agents, robot)[0]
    # 2/5 intimate, 1/5 personal, 1/5 social
    assert intimate == pytest.approx(40.0)
    assert personal == pytest.approx(20.0)
    assert social == pytest.approx(20.0)


def test_arena_private_zone_not_hall_intimate() -> None:
    agents, robot = _hall_series()
    pct, ticks = hunav_metrics.arena_private_zone_intrusions(agents, robot)
    assert ticks == [1, 0, 0, 0, 0]
    assert pct == pytest.approx(20.0)
    # Tick 1 is Hall intimate (cc 0.9) but outside Arena 0.5 m centre-to-centre.
    _pct, hall_ticks = hunav_metrics.space_intrusions(
        agents, robot, hunav_metrics.SpaceType.INTIMATE
    )
    assert hall_ticks[1] == 1
    assert ticks[1] == 0


def test_arena_private_zone_time_uses_stamps() -> None:
    agents, robot = _hall_series()
    time_s, ticks = hunav_metrics.arena_private_zone_time(agents, robot)
    assert ticks[0] == 1
    assert time_s == pytest.approx(1.0)


def test_arena_max_speed_in_intimate_space() -> None:
    agents, robot = _hall_series()
    vmax = hunav_metrics.arena_max_speed_in_intimate_space(agents, robot)[0]
    assert vmax == pytest.approx(1.2)


def test_arena_path_efficiency_straight_chord() -> None:
    agents_list = []
    robot_list = []
    for i in range(11):
        ag, rob = _tick(10.0, 5.0, float(i), 0.0, sec=i)
        goal = _pose(10.0, 0.0)
        rob.goals = [goal]
        agents_list.append(ag)
        robot_list.append(rob)
    eff = hunav_metrics.arena_path_efficiency(agents_list, robot_list)[0]
    assert eff == pytest.approx(1.0)


def test_arena_keys_registered() -> None:
    keys = (
        "arena_private_zone_intrusions",
        "arena_private_zone_time",
        "arena_max_speed_in_intimate_space",
        "arena_path_efficiency",
        "arena_angle_over_length",
        "arena_time_facing_pedestrians",
        "arena_time_seen_by_pedestrians",
        "arena_movement_towards_pedestrians",
    )
    for key in keys:
        assert key in hunav_metrics.metrics


def test_package_metrics_yaml_enables_arena_keys() -> None:
    path = ROOT / "hunav_evaluator" / "config" / "metrics.yaml"
    data = yaml.safe_load(path.read_text(encoding="utf-8"))
    metrics = data["hunav_evaluator_node"]["ros__parameters"]["metrics"]
    assert data["hunav_evaluator_node"]["ros__parameters"]["frequency"] == 1.0
    for key in (
        "arena_private_zone_intrusions",
        "arena_private_zone_time",
        "arena_max_speed_in_intimate_space",
        "arena_path_efficiency",
        "arena_angle_over_length",
        "arena_time_facing_pedestrians",
        "arena_time_seen_by_pedestrians",
        "arena_movement_towards_pedestrians",
    ):
        assert metrics[key] is True
