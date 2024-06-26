# https://raw.githubusercontent.com/ros-industrial/universal_robot/kinetic-devel/ur_kinematics/src/ur_kin.cpp
# https://smartech.gatech.edu/bitstream/handle/1853/50782/ur_kin_tech_report_1.pdf
# http://rasmusan.blog.aau.dk/files/ur5_kinematics.pdf
import math

import numpy as np
import pytransform3d.rotations as pr
import pytransform3d.transformations as pt


class UR3Params:
    d1 = 0.1519
    a2 = -0.24365
    a3 = -0.21325
    d4 = 0.11235
    d5 = 0.08535
    d6 = 0.0819


class UR5Params:
    d1 = 0.089159
    a2 = -0.42500
    a3 = -0.39225
    d4 = 0.10915
    d5 = 0.09465
    d6 = 0.0823


class UR10Params:
    d1 = 0.1273
    a2 = -0.612
    a3 = -0.5723
    d4 = 0.163941
    d5 = 0.1157
    d6 = 0.0922


# TODO determine parameters for UR10e

ZERO_THRESH = 1e-8
TWO_PI = 2.0 * math.pi
# see experimental/test_ur5_mia_cartesian.py

ee_kin2ee_options = {
    # see experimental/test_ur5_mia_cartesian.py
    "palm": np.array(
        [
            [5.87586868e-05, -9.99999998e-01, -1.83652792e-16, -2.47027865e-05],
            [9.68930791e-01, 5.69331010e-05, 2.47332003e-01, 1.72392460e-02],
            [-2.47332002e-01, -1.45329037e-05, 9.68930792e-01, -4.44860194e-03],
            [0.00000000e00, 0.00000000e00, 0.00000000e00, 1.00000000e00],
        ]
    ),
    "ur5_tool0": np.array(
        [
            [5.87586868e-05, -9.99999998e-01, -1.83652792e-16, -2.47027865e-05],
            [7.22061953e-04, 4.24274124e-08, -9.99999739e-01, 5.94256999e-05],
            [9.99999738e-01, 5.87586715e-05, 7.22061954e-04, -1.96120385e-04],
            [0.00000000e00, 0.00000000e00, 0.00000000e00, 1.00000000e00],
        ]
    ),
    "rh_forearm": np.array(
        [  # TODO: configure this?
            [6.34136230e-07, -9.99999683e-01, -7.96326458e-04, 0.00000000e00],
            [7.96326458e-04, 7.96326711e-04, -9.99999366e-01, 0.00000000e00],
            [9.99999683e-01, 0.00000000e00, 7.96326711e-04, 0.00000000e00],
            [0.00000000e00, 0.00000000e00, 0.00000000e00, 1.00000000e00],
        ]
    ),
}

urdf_base2kin_base = np.array(
    [
        [1.0, 0.0, 0.0, 0.0],
        [0.0, 1.0, 0.0, 0.0],
        [0.0, 0.0, 1.0, -1.0],
        [0.0, 0.0, 0.0, 1.0],
    ]
)
robot_params = {"ur3": UR3Params, "ur5": UR5Params, "ur10": UR10Params}


joint_names = [
    "shoulder_pan_joint",
    "shoulder_lift_joint",
    "elbow_joint",
    "wrist_1_joint",
    "wrist_2_joint",
    "wrist_3_joint",
]
link_names = [
    "base_link",
    "shoulder_link",
    "upper_arm_link",
    "forearm_link",
    "wrist_1_link",
    "wrist_2_link",
    "wrist_3_link",
    "ee_link",
]
joint_min = [
    -3.141592653589793,
    -3.141592653589793,
    0.0,
    -3.141592653589793,
    -3.141592653589793,
    -0.7853981633974483,
]
joint_max = [
    3.141592653589793,
    0.7400196028455958,
    2.93005874824808,
    3.141592653589793,
    3.141592653589793,
    5.497787143782138,
]


def analytical_ik(
    pose,
    last_joint_angles,
    params=UR5Params,
    urdf_base2kin_base=urdf_base2kin_base,
    ee_kin2ee=ee_kin2ee_options["ur5_tool0"],
):
    """Analytical inverse kinematics for universal robot arms."""
    ee_link2urdf_base = pt.transform_from(
        R=pr.matrix_from_quaternion([pose[-1], pose[3], pose[4], pose[5]]),
        p=pose[:3],
    )
    return analytical_ik_homogeneous(
        ee_link2urdf_base,
        last_joint_angles,
        params,
        urdf_base2kin_base,
        ee_kin2ee,
    )


def analytical_ik_homogeneous(
    ee_link2urdf_base,
    last_joint_angles,
    params=UR5Params,
    urdf_base2kin_base=urdf_base2kin_base,
    ee_kin2ee=ee_kin2ee_options["ur5_tool0"],
):
    """Analytical inverse kinematics for universal robot arms."""
    ee_link2kin_base = pt.concat(ee_link2urdf_base, urdf_base2kin_base)
    ee2kin_base = pt.concat(ee_kin2ee, ee_link2kin_base)
    q6_des = 0.0
    solutions = _inverse(ee2kin_base, q6_des, params)
    n_joints = solutions.shape[1]
    valid_solutions = []

    for solution in solutions:
        if all(joint_min <= solution) and all(solution <= joint_max):
            valid_solutions.append(solution)

    if len(valid_solutions) > 0:
        solution = np.copy(last_joint_angles)
        best_solution_idx = np.argmin(
            np.linalg.norm(
                np.array(valid_solutions)
                - last_joint_angles[np.newaxis, :n_joints],
                axis=1,
            )
        )
        solution[:n_joints] = valid_solutions[best_solution_idx]
        return solution
    else:
        return None


def _forward(q, params):
    """Forward kinematics for universal robot arms."""
    s1 = math.sin(q[0])
    c1 = math.cos(q[0])
    s2 = math.sin(q[1])
    c2 = math.cos(q[1])
    s4 = math.sin(q[3])
    c4 = math.cos(q[3])
    s5 = math.sin(q[4])
    c5 = math.cos(q[4])
    s6 = math.sin(q[5])
    c6 = math.cos(q[5])
    q23 = q[1] + q[2]
    s23 = math.sin(q23)
    c23 = math.cos(q23)
    q234 = q[1] + q[2] + q[3]
    s234 = math.sin(q234)
    c234 = math.cos(q234)

    T = np.empty((4, 4))
    T[0, 0] = c234 * c1 * s5 - c5 * s1
    T[0, 1] = c6 * (s1 * s5 + c234 * c1 * c5) - s234 * c1 * s6
    T[0, 2] = -s6 * (s1 * s5 + c234 * c1 * c5) - s234 * c1 * c6
    T[0, 3] = (
        params.d6 * c234 * c1 * s5
        - params.a3 * c23 * c1
        - params.a2 * c1 * c2
        - params.d6 * c5 * s1
        - params.d5 * s234 * c1
        - params.d4 * s1
    )
    T[1, 0] = c1 * c5 + c234 * s1 * s5
    T[1, 1] = -c6 * (c1 * s5 - c234 * c5 * s1) - s234 * s1 * s6
    T[1, 2] = s6 * (c1 * s5 - c234 * c5 * s1) - s234 * c6 * s1
    T[1, 3] = (
        params.d6 * (c1 * c5 + c234 * s1 * s5)
        + params.d4 * c1
        - params.a3 * c23 * s1
        - params.a2 * c2 * s1
        - params.d5 * s234 * s1
    )
    T[2, 0] = -s234 * s5
    T[2, 1] = -c234 * s6 - s234 * c5 * c6
    T[2, 2] = s234 * c5 * s6 - c234 * c6
    T[2, 3] = (
        params.d1
        + params.a3 * s23
        + params.a2 * s2
        - params.d5 * (c23 * c4 - s23 * s4)
        - params.d6 * s5 * (c23 * s4 + s23 * c4)
    )
    T[3, 0] = 0.0
    T[3, 1] = 0.0
    T[3, 2] = 0.0
    T[3, 3] = 1.0
    return T


def _inverse(T, q6_des, params):
    """Inverse kinematics for universal robot arms."""
    q_sols = np.empty((8, 6))
    n_sols = 0
    T02 = -T[0, 0]
    T00 = T[0, 1]
    T01 = T[0, 2]
    T03 = -T[0, 3]
    T12 = -T[1, 0]
    T10 = T[1, 1]
    T11 = T[1, 2]
    T13 = -T[1, 3]
    T22 = T[2, 0]
    T20 = -T[2, 1]
    T21 = -T[2, 2]
    T23 = T[2, 3]

    # shoulder rotate joint (q1)
    q1 = np.empty(2)
    A = params.d6 * T12 - T13
    B = params.d6 * T02 - T03
    R = A * A + B * B
    if abs(A) < ZERO_THRESH:
        if abs(abs(params.d4) - abs(B)) < ZERO_THRESH:
            div = -_fast_sign(params.d4) * _fast_sign(B)
        else:
            div = -params.d4 / B
        div = max(min(div, 1.0), -1.0)  # TODO check if this is OK
        arcsin = math.asin(div)
        if abs(arcsin) < ZERO_THRESH:
            arcsin = 0.0
        if arcsin < 0.0:
            q1[0] = arcsin + TWO_PI
        else:
            q1[0] = arcsin
        q1[1] = math.pi - arcsin
    elif abs(B) < ZERO_THRESH:
        if abs(abs(params.d4) - abs(A)) < ZERO_THRESH:
            div = _fast_sign(params.d4) * _fast_sign(A)
        else:
            div = params.d4 / A
        arccos = math.acos(div)
        q1[0] = arccos
        q1[1] = TWO_PI - arccos
    elif params.d4 * params.d4 > R:
        return _postprocess_angles(q_sols[:n_sols])
    else:
        arccos = math.acos(params.d4 / math.sqrt(R))
        arctan = math.atan2(-B, A)
        pos = arccos + arctan
        neg = -arccos + arctan
        if abs(pos) < ZERO_THRESH:
            pos = 0.0
        if abs(neg) < ZERO_THRESH:
            neg = 0.0
        if pos >= 0.0:
            q1[0] = pos
        else:
            q1[0] = TWO_PI + pos
        if neg >= 0.0:
            q1[1] = neg
        else:
            q1[1] = TWO_PI + neg

    # wrist 2 joint (q5)
    q5 = np.empty((2, 2))

    for i in range(2):
        numer = T03 * math.sin(q1[i]) - T13 * math.cos(q1[i]) - params.d4
        if abs(abs(numer) - abs(params.d6)) < ZERO_THRESH:
            div = _fast_sign(numer) * _fast_sign(params.d6)
        else:
            div = numer / params.d6
        div = max(min(div, 1.0), -1.0)  # TODO check if this is OK
        arccos = math.acos(div)
        q5[i, 0] = arccos
        q5[i, 1] = TWO_PI - arccos

    for i in range(2):
        for j in range(2):
            c1 = math.cos(q1[i])
            s1 = math.sin(q1[i])
            c5 = math.cos(q5[i, j])
            s5 = math.sin(q5[i, j])
            # wrist 3 joint (q6)
            if abs(s5) < ZERO_THRESH:
                q6 = q6_des
            else:
                q6 = math.atan2(
                    _fast_sign(s5) * -(T01 * s1 - T11 * c1),
                    _fast_sign(s5) * (T00 * s1 - T10 * c1),
                )
                if abs(q6) < ZERO_THRESH:
                    q6 = 0.0

            q2 = np.empty(2)
            q3 = np.empty(2)
            q4 = np.empty(2)
            # RRR joints (q2,q3,q4)
            c6 = math.cos(q6)
            s6 = math.sin(q6)
            x04x = -s5 * (T02 * c1 + T12 * s1) - c5 * (
                s6 * (T01 * c1 + T11 * s1) - c6 * (T00 * c1 + T10 * s1)
            )
            x04y = c5 * (T20 * c6 - T21 * s6) - T22 * s5
            p13x = (
                params.d5
                * (s6 * (T00 * c1 + T10 * s1) + c6 * (T01 * c1 + T11 * s1))
                - params.d6 * (T02 * c1 + T12 * s1)
                + T03 * c1
                + T13 * s1
            )
            p13y = (
                T23
                - params.d1
                - params.d6 * T22
                + params.d5 * (T21 * c6 + T20 * s6)
            )

            c3 = (
                p13x * p13x
                + p13y * p13y
                - params.a2 * params.a2
                - params.a3 * params.a3
            ) / (2.0 * params.a2 * params.a3)
            if abs(abs(c3) - 1.0) < ZERO_THRESH:
                c3 = _fast_sign(c3)
            elif abs(c3) > 1.0:
                continue  # NO SOLUTION
            arccos = math.acos(c3)
            q3[0] = arccos
            q3[1] = TWO_PI - arccos
            denom = (
                params.a2 * params.a2
                + params.a3 * params.a3
                + 2 * params.a2 * params.a3 * c3
            )
            s3 = math.sin(arccos)
            A = params.a2 + params.a3 * c3
            B = params.a3 * s3
            q2[0] = math.atan2(
                (A * p13y - B * p13x) / denom, (A * p13x + B * p13y) / denom
            )
            q2[1] = math.atan2(
                (A * p13y + B * p13x) / denom, (A * p13x - B * p13y) / denom
            )
            c23_0 = math.cos(q2[0] + q3[0])
            s23_0 = math.sin(q2[0] + q3[0])
            c23_1 = math.cos(q2[1] + q3[1])
            s23_1 = math.sin(q2[1] + q3[1])
            q4[0] = math.atan2(
                c23_0 * x04y - s23_0 * x04x, x04x * c23_0 + x04y * s23_0
            )
            q4[1] = math.atan2(
                c23_1 * x04y - s23_1 * x04x, x04x * c23_1 + x04y * s23_1
            )

            for k in range(2):
                if abs(q2[k]) < ZERO_THRESH:
                    q2[k] = 0.0
                if abs(q4[k]) < ZERO_THRESH:
                    q4[k] = 0.0
                q_sols[n_sols, 0] = q1[i]
                q_sols[n_sols, 1] = q2[k]
                q_sols[n_sols, 2] = q3[k]
                q_sols[n_sols, 3] = q4[k]
                q_sols[n_sols, 4] = q5[i, j]
                q_sols[n_sols, 5] = q6
                n_sols += 1
    return _postprocess_angles(q_sols[:n_sols])


def _fast_sign(x):
    if x > 0.0:
        return 1.0
    elif x < 0.0:
        return -1.0
    else:
        return 0.0


def _postprocess_angles(a):
    a = -((math.pi - np.asarray(a)) % TWO_PI - math.pi)
    a[a[:, 5] < joint_min[5], 5] += TWO_PI  # HACK
    return a
