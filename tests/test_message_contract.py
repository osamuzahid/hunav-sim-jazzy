"""hunav_msgs IDL contract (no Isaac Sim). Generated types are optional."""

from __future__ import annotations

from pathlib import Path

import pytest

ROOT = Path(__file__).resolve().parents[1]
MSG = ROOT / "hunav_msgs" / "msg"
SRV = ROOT / "hunav_msgs" / "srv"


def _read(path: Path) -> str:
    return path.read_text(encoding="utf-8")


def _generated_agent():
    try:
        from hunav_msgs.msg import Agent
    except ImportError:
        pytest.skip("hunav_msgs not importable (package not built / ROS not sourced)")
    if not (hasattr(Agent, "SLOT_TYPES") or hasattr(Agent, "_TYPE_SUPPORT")):
        pytest.skip("hunav_msgs is a test stub, not generated ROS types")
    return Agent


def test_agent_msg_type_constants_and_fields() -> None:
    text = _read(MSG / "Agent.msg")
    for line in (
        "uint8 PERSON=1",
        "uint8 ROBOT=2",
        "uint8 OTHER=3",
        "int32 id",
        "uint8 type",
        "geometry_msgs/Pose position",
        "float32 yaw",
        "float32 radius",
        "float32 linear_vel",
        "hunav_msgs/AgentBehavior behavior",
    ):
        assert line in text, line


def test_agents_msg_header_and_array() -> None:
    text = _read(MSG / "Agents.msg")
    assert "std_msgs/Header header" in text
    assert "hunav_msgs/Agent[] agents" in text


def test_agent_behavior_msg_types() -> None:
    text = _read(MSG / "AgentBehavior.msg")
    for line in (
        "uint8 BEH_REGULAR=1",
        "uint8 BEH_IMPASSIVE=2",
        "uint8 BEH_CURIOUS=5",
        "uint8 BEH_THREATENING=6",
        "uint8 type",
        "uint8 state",
    ):
        assert line in text, line


def test_compute_agents_srv() -> None:
    req, resp = _read(SRV / "ComputeAgents.srv").split("---", 1)
    assert "hunav_msgs/Agents current_agents" in req
    assert "hunav_msgs/Agent robot" in req
    assert "hunav_msgs/Agents updated_agents" in resp


def test_start_evaluation_srv() -> None:
    req, resp = _read(SRV / "StartEvaluation.srv").split("---", 1)
    assert "geometry_msgs/PoseStamped robot_goal" in req
    assert "string experiment_tag" in req
    assert "int32 run_id" in req
    assert "bool success" in resp


def test_generated_agent_constants_optional() -> None:
    Agent = _generated_agent()
    assert int(Agent.PERSON) == 1
    assert int(Agent.ROBOT) == 2
    assert int(Agent.OTHER) == 3
