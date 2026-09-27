"""UR5e kinematics and whole-arm clearance (pure math, no robot)."""

from math import pi

import numpy as np
import pytest

from system_arm import (ArmRefused, cartesian_joint_path, check_arm, frames, joint_path,
                        link_capsules, pose_matrix, rotation_matrix, rotvec, solve_ik, tcp_matrix)
from system_geometry import Box, Station, Workspace
from system_model import Step

Q0 = np.array([0.3, -1.9, 1.8, -1.47, -1.57, 0.2])   # tool down, in front of the base
TOOL = pose_matrix([0, 0, 0.2, 0, 0, 0])


def tcp_pose(q):
    T = tcp_matrix(q, TOOL)
    return [*T[:3, 3], *rotvec(T[:3, :3])]


def workspace(*boxes):
    stations = tuple(Station(f"s{i}", box) for i, box in enumerate(boxes))
    return Workspace(stations, 0.08, 0.02, -0.1, 0.8)


def test_forward_kinematics_matches_the_ur5e_zero_pose():
    # UR5e documentation: flange at (-817.2, -232.9, 62.8) mm with all joints at 0
    assert np.allclose(frames([0] * 6)[-1][:3, 3], [-0.8172, -0.2329, 0.0628], atol=1e-4)


@pytest.mark.parametrize("angle", [0.0, 0.4, 2.0, 3.1, pi])
def test_rotvec_inverts_rotation_matrix(angle):
    R = rotation_matrix(np.array([1.0, -2.0, 0.5]) / np.sqrt(5.25) * angle)
    assert np.allclose(rotation_matrix(rotvec(R)), R, atol=1e-7)


def test_ik_finds_the_nearby_solution():
    target = tcp_matrix(Q0 + [0.1, -0.05, 0.08, 0.05, 0.02, -0.1], TOOL)
    q = solve_ik(target, Q0, TOOL)
    assert q is not None
    reached = tcp_matrix(q, TOOL)
    assert np.linalg.norm(reached[:3, 3] - target[:3, 3]) < 1e-4            # IK_POSITION_TOL
    assert np.linalg.norm(rotvec(target[:3, :3] @ reached[:3, :3].T)) < 1e-3  # IK_ROTATION_TOL
    assert np.max(np.abs(q - Q0)) < 0.2


def test_straight_move_gives_continuous_joints():
    start = tcp_pose(Q0)
    end = [start[0] + 0.1, start[1] - 0.15, start[2] + 0.05, *start[3:]]
    samples = cartesian_joint_path(Q0, [end], TOOL)
    assert len(samples) > 15
    assert max(np.max(np.abs(b - a)) for a, b in zip(samples, samples[1:])) < 0.1
    assert np.allclose(tcp_pose(samples[-1])[:3], end[:3], atol=1e-4)


def test_unreachable_pose_is_refused():
    start = tcp_pose(Q0)
    with pytest.raises(ArmRefused, match="not reachable"):
        cartesian_joint_path(Q0, [[1.5, 0.0, start[2], *start[3:]]], TOOL)


def test_keepalive_is_called_on_long_paths():
    start = tcp_pose(Q0)
    calls = []
    cartesian_joint_path(Q0, [[start[0] + 0.2, *start[1:]]], TOOL, lambda: calls.append(1))
    assert calls


def test_joint_limits_are_enforced():
    with pytest.raises(ArmRefused):
        joint_path(Q0, [*Q0[:2], 3.3, *Q0[3:]])
    assert len(joint_path(Q0, [*Q0[:5], Q0[5] - pi])) > 10


def test_forearm_in_a_station_is_refused_even_with_the_tool_clear():
    _, start, end, _, _ = link_capsules(Q0, TOOL)[1]      # forearm
    middle = (start + end) / 2
    box = Box(tuple(middle - 0.02), tuple(middle + 0.02))
    tcp = tcp_matrix(Q0, TOOL)[:3, 3]
    assert box.segment_interval(tuple(tcp), tuple(tcp), 0.1) is None   # the tool sphere misses it
    with pytest.raises(ArmRefused, match="forearm"):
        check_arm([Q0], workspace(box), Step.LIFT, TOOL)


def test_clear_arm_passes():
    check_arm([Q0], workspace(Box((1.0, 1.0, 0.0), (1.2, 1.2, 0.2))), Step.LIFT, TOOL)
