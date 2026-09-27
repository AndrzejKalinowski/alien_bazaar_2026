"""Whole-arm checks for planned UR5e motion: joint paths and link clearance.

SafeControl only limits the TCP height and system_geometry only checks the
tool+glass sphere. This module reconstructs the joint angles along a planned
path and checks the arm links against the station boxes before any motion:

  moveL  the controller moves the TCP on a straight line and solves the
         inverse kinematics near the current joints. cartesian_joint_path()
         does the same: damped least squares seeded with the previous sample,
         every PATH_STEP / ANGLE_STEP. A pose it cannot reach, a joint jump
         (branch switch, near a singularity) or a joint limit refuses the plan
         (the real robot would protective-stop half way).
  moveJ  joints move linearly; joint_path() samples that, and forward
         kinematics gives the links (this covers the FLIP arc).

Links are capsules between points of the nominal UR5e DH chain: upper arm
(offset SHOULDER_OFFSET along the shoulder axis, as on the real arm),
forearm, wrist 1-3 and flange -> TCP (the gripper body). Each must stay
clear of every station box inflated by the link radius + clearance, except
the wrist/gripper links inside the active station's taught work corridor
during its approach, work and withdrawal (the same rule as the tool).

LIMITS OF THIS MODEL (write them down before trusting it):
  * nominal DH parameters, not the robot's calibration (a few mm off);
  * link radii and the shoulder offset are conservative estimates, TO BE
    MEASURED; the base and shoulder housing are not checked (stations must
    not be there anyway);
  * the table and unmodelled objects are not checked, only station boxes;
  * blends are not used by the planner, so each moveL is a separate line.
The robot's own safety configuration (planes, joint limits) stays the
final guard. Requires: numpy.
"""

from math import pi

import numpy as np

from system_geometry import STATION_ACCESS, CollisionRefused

# --- UR5e nominal DH (Universal Robots, "DH parameters for calculations") ------------
DH_A = (0.0, -0.425, -0.3922, 0.0, 0.0, 0.0)          # m
DH_D = (0.1625, 0.0, 0.0, 0.1333, 0.0997, 0.0996)     # m
DH_ALPHA = (pi / 2, 0.0, 0.0, pi / 2, -pi / 2, 0.0)   # rad

# --- link capsules: conservative estimates, TO BE MEASURED ---------------------------
SHOULDER_OFFSET = 0.138      # m, upper arm centre line off the DH line, along the shoulder axis
UPPER_ARM_RADIUS = 0.075     # m
FOREARM_RADIUS = 0.065       # m
WRIST_RADIUS = 0.06          # m
GRIPPER_RADIUS = 0.05        # m, flange -> TCP body (the glass is the tool sphere)
LINK_SAMPLE = 0.02           # m between checked points along a capsule

# --- path reconstruction ---------------------------------------------------------
PATH_STEP = 0.01             # m of TCP travel between IK samples
ANGLE_STEP = 0.05            # rad of TCP rotation between IK samples
JOINT_STEP = 0.05            # rad between moveJ samples
IK_ITERATIONS = 30
IK_POSITION_TOL = 1e-4       # m
IK_ROTATION_TOL = 1e-3       # rad
IK_DAMPING = 0.01
MAX_JOINT_JUMP = 0.2         # rad between neighbouring samples: a branch switch / singularity
JOINT_LIMIT = 2 * pi         # rad, every joint (UR5e +-360 deg)
ELBOW_LIMIT = pi             # rad, the elbow cannot pass through the upper arm


class ArmRefused(CollisionRefused):
    """The arm (not only the tool) cannot follow the planned path safely."""


def rotation_matrix(rotvec):
    r = np.asarray(rotvec, dtype=float)
    theta = np.linalg.norm(r)
    if theta < 1e-12:
        return np.eye(3)
    k = r / theta
    K = np.array([[0, -k[2], k[1]], [k[2], 0, -k[0]], [-k[1], k[0], 0]])
    return np.eye(3) + np.sin(theta) * K + (1 - np.cos(theta)) * K @ K


def rotvec(R):
    """3x3 rotation matrix -> axis-angle vector (inverse of rotation_matrix)."""
    cos_theta = np.clip((np.trace(R) - 1) / 2, -1.0, 1.0)
    theta = np.arccos(cos_theta)
    if theta < 1e-9:
        return np.zeros(3)
    if theta > pi - 1e-6:
        # Near 180 deg R = 2 a a^T - I: the antisymmetric part vanishes.
        i = int(np.argmax(np.diag(R)))
        axis = np.empty(3)
        axis[i] = np.sqrt(max((R[i, i] + 1) / 2, 0.0))
        for j in range(3):
            if j != i:
                axis[j] = (R[i, j] + R[j, i]) / (4 * axis[i])
        return axis / np.linalg.norm(axis) * theta
    axis = np.array([R[2, 1] - R[1, 2], R[0, 2] - R[2, 0], R[1, 0] - R[0, 1]]) / (2 * np.sin(theta))
    return axis * theta


def pose_matrix(pose):
    T = np.eye(4)
    T[:3, :3] = rotation_matrix(pose[3:])
    T[:3, 3] = pose[:3]
    return T


def frames(q):
    """Base, then frames 1..6 (6 = flange) of the DH chain, as 4x4 matrices."""
    T = np.eye(4)
    result = [T]
    for theta, a, d, alpha in zip(q, DH_A, DH_D, DH_ALPHA):
        ct, st, ca, sa = np.cos(theta), np.sin(theta), np.cos(alpha), np.sin(alpha)
        A = np.array([[ct, -st * ca, st * sa, a * ct],
                      [st, ct * ca, -ct * sa, a * st],
                      [0.0, sa, ca, d],
                      [0.0, 0.0, 0.0, 1.0]])
        T = T @ A
        result.append(T)
    return result


def tcp_matrix(q, T_flange_tcp):
    return frames(q)[-1] @ T_flange_tcp


def _error(T, target):
    """6-vector (translation, rotation) taking pose T to target, in the base frame."""
    return np.concatenate([target[:3, 3] - T[:3, 3], rotvec(target[:3, :3] @ T[:3, :3].T)])


def solve_ik(target, seed, T_flange_tcp):
    """Joint angles near seed that put the TCP at target (4x4), or None."""
    q = np.array(seed, dtype=float)
    for _ in range(IK_ITERATIONS):
        T = tcp_matrix(q, T_flange_tcp)
        error = _error(T, target)
        if np.linalg.norm(error[:3]) < IK_POSITION_TOL and np.linalg.norm(error[3:]) < IK_ROTATION_TOL:
            return q
        J = np.empty((6, 6))   # column i: pose change per rad of joint i (finite difference)
        for i in range(6):
            dq = q.copy()
            dq[i] += 1e-6
            J[:, i] = _error(T, tcp_matrix(dq, T_flange_tcp)) / 1e-6
        # Damped least squares: stays bounded near singularities.
        step = J.T @ np.linalg.solve(J @ J.T + IK_DAMPING ** 2 * np.eye(6), error)
        q += step
    return None


def check_limits(q):
    if np.any(np.abs(q) > JOINT_LIMIT) or abs(q[2]) > ELBOW_LIMIT:
        raise ArmRefused(f"joint limit on the planned path (q = {np.degrees(q).round(0).tolist()} deg)")


def _interpolate(start, end, t):
    """Pose t in [0, 1] of the straight TCP line between two 4x4 poses."""
    T = np.eye(4)
    T[:3, 3] = start[:3, 3] + (end[:3, 3] - start[:3, 3]) * t
    T[:3, :3] = rotation_matrix(rotvec(end[:3, :3] @ start[:3, :3].T) * t) @ start[:3, :3]
    return T


KEEPALIVE_SAMPLES = 20       # samples between keepalive() calls (watchdog kick)


def cartesian_joint_path(q0, poses, T_flange_tcp, keepalive=None):
    """Joint samples for consecutive moveL targets starting at joints q0.

    keepalive: called every KEEPALIVE_SAMPLES samples; long paths take longer
    than the 0.2 s RTDE watchdog period (about 1 ms per sample).
    """
    q = np.array(q0, dtype=float)
    samples = [q.copy()]
    current = tcp_matrix(q, T_flange_tcp)
    for pose in poses:
        target = pose_matrix(pose)
        distance = np.linalg.norm(target[:3, 3] - current[:3, 3])
        angle = np.linalg.norm(rotvec(target[:3, :3] @ current[:3, :3].T))
        count = max(1, int(np.ceil(max(distance / PATH_STEP, angle / ANGLE_STEP))))
        for k in range(1, count + 1):
            solution = solve_ik(_interpolate(current, target, k / count), q, T_flange_tcp)
            if solution is None:
                raise ArmRefused(f"TCP pose {np.round(pose[:3], 3).tolist()} is not reachable "
                                 "on a straight line from the current joints")
            if np.max(np.abs(solution - q)) > MAX_JOINT_JUMP:
                raise ArmRefused("joint jump on a straight move (singularity or configuration change)")
            check_limits(solution)
            q = solution
            samples.append(q.copy())
            if keepalive is not None and len(samples) % KEEPALIVE_SAMPLES == 0:
                keepalive()
        current = target
    return samples


def joint_path(q0, q1):
    q0, q1 = np.asarray(q0, dtype=float), np.asarray(q1, dtype=float)
    count = max(1, int(np.ceil(np.max(np.abs(q1 - q0)) / JOINT_STEP)))
    samples = [q0 + (q1 - q0) * k / count for k in range(count + 1)]
    for q in samples:
        check_limits(q)
    return samples


def link_capsules(q, T_flange_tcp):
    """[(name, start, end, radius, is_wrist_or_tool)] in base-frame metres."""
    F = frames(q)
    shoulder_axis = F[1][:3, 2]
    offset = shoulder_axis * SHOULDER_OFFSET
    p = [T[:3, 3] for T in F]
    tcp = (F[6] @ T_flange_tcp)[:3, 3]
    return [("upper arm", p[1] + offset, p[2] + offset, UPPER_ARM_RADIUS, False),
            ("forearm", p[2], p[3], FOREARM_RADIUS, False),
            ("wrist 1", p[3], p[4], WRIST_RADIUS, True),
            ("wrist 2", p[4], p[5], WRIST_RADIUS, True),
            ("wrist 3", p[5], p[6], WRIST_RADIUS, True),
            ("gripper", p[6], tcp, GRIPPER_RADIUS, True)]


def _points(start, end):
    count = max(1, int(np.ceil(np.linalg.norm(end - start) / LINK_SAMPLE)))
    return [start + (end - start) * k / count for k in range(count + 1)]


def check_arm(samples, workspace, step, T_flange_tcp):
    """Refuse when a link comes too close to a station on any joint sample."""
    access = STATION_ACCESS.get(step)
    active = workspace.station(access[0]) if access else None
    for q in samples:
        for name, start, end, radius, wrist in link_capsules(q, T_flange_tcp):
            padding = radius + workspace.clearance
            for station in workspace.stations:
                for solid in station.solid_parts:
                    if solid.segment_interval(tuple(start), tuple(end), padding) is not None:
                        raise ArmRefused(f"{name} would hit a solid part of {station.id}")
                if station.bounds.segment_interval(tuple(start), tuple(end), padding) is None:
                    continue
                if wrist and station is active and all(
                        station.work_corridor.contains_envelope(tuple(point), padding)
                        or station.bounds.segment_interval(tuple(point), tuple(point), padding) is None
                        for point in _points(start, end)):
                    continue  # inside the taught corridor like the tool itself
                raise ArmRefused(f"{name} would enter station {station.id}")
