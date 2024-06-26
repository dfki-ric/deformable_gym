import numpy as np
import pytransform3d.rotations as pr
import pytransform3d.transformations as pt
from pybullet_utils import bullet_client as bc


def draw_transform(
    pose2origin,
    s,
    pb_client: bc.BulletClient,
    lw=1,
    replace_item_unique_ids=(-1, -1, -1),
):
    """Draw transformation matrix.

    Parameters
    ----------
    pose2origin : array-like, shape (4, 4)
        Homogeneous transformation matrix

    s : float
        Scale, length of the coordinate axes

    client_id : int, optional (default: 0)
        Physics client ID

    lw : int, optional (default: 1)
        Line width

    replace_item_unique_ids : list
        User data ids that identify lines that will be replaced in order
        x-axis, y-axis, z-axis.

    Returns
    -------
    userDataIds : list
        User data ids that identify the created lines.
    """
    line_x = pb_client.addUserDebugLine(
        pose2origin[:3, 3],
        pose2origin[:3, 3] + s * pose2origin[:3, 0],
        [1, 0, 0],
        lw,
        replaceItemUniqueId=replace_item_unique_ids[0],
    )
    line_y = pb_client.addUserDebugLine(
        pose2origin[:3, 3],
        pose2origin[:3, 3] + s * pose2origin[:3, 1],
        [0, 1, 0],
        lw,
        replaceItemUniqueId=replace_item_unique_ids[1],
    )
    line_z = pb_client.addUserDebugLine(
        pose2origin[:3, 3],
        pose2origin[:3, 3] + s * pose2origin[:3, 2],
        [0, 0, 1],
        lw,
        replaceItemUniqueId=replace_item_unique_ids[2],
    )
    return [line_x, line_y, line_z]


def draw_pose(
    pos,
    rot,
    s,
    pb_client: bc.BulletClient,
    lw=1,
    replace_item_unique_ids=(-1, -1, -1),
):
    """Draw transformation matrix.

    Parameters
    ----------
    pos : array-like, shape (3,)
        Position: (x, y, z)

    rot : array-like, shape (4,)
        Quaternion: (qx, qy, qz, qw)

    s : float
        Scale, length of the coordinate axes

    client_id : int, optional (default: 0)
        Physics client ID

    lw : int, optional (default: 1)
        Line width

    replace_item_unique_ids : list
        User data ids that identify lines that will be replaced in order
        x-axis, y-axis, z-axis.

    Returns
    -------
    userDataIds : list
        User data ids that identify the created lines.
    """
    q_scalar_first = pr.quaternion_wxyz_from_xyzw(rot)
    pose2origin = pt.transform_from(
        R=pr.matrix_from_quaternion(q_scalar_first), p=pos
    )
    return draw_transform(
        pose2origin, s, pb_client, lw, replace_item_unique_ids
    )


def draw_box(
    corners_in_world, pb_client: bc.BulletClient, replace_item_unique_ids=None
):
    """Draw box.

    Parameters
    ----------
    corners_in_world : array-like, shape (8, 3)
        Corners of the box in world coordinates.

    replace_item_unique_ids : list
        User data ids that identify edges.
    """
    if replace_item_unique_ids is None:
        replace_item_unique_ids = [-1] * 12
    for line_idx, (i, j) in enumerate(
        [
            (0, 1),
            (0, 2),
            (1, 3),
            (2, 3),
            (4, 5),
            (4, 6),
            (5, 7),
            (6, 7),
            (0, 4),
            (1, 5),
            (2, 6),
            (3, 7),
        ]
    ):
        replace_item_unique_ids[line_idx] = pb_client.addUserDebugLine(
            corners_in_world[i],
            corners_in_world[j],
            (0, 0, 0),
            replaceItemUniqueId=replace_item_unique_ids[line_idx],
        )
    return replace_item_unique_ids


def draw_limits(limits, pb_client):
    x_lo = limits[0][0]
    y_lo = limits[0][1]
    z_lo = limits[0][2]
    x_hi = limits[1][0]
    y_hi = limits[1][1]
    z_hi = limits[1][2]
    task_space_corners = np.array(
        [
            [x_lo, y_lo, z_hi],
            [x_lo, y_hi, z_hi],
            [x_hi, y_lo, z_hi],
            [x_hi, y_hi, z_hi],
            [x_lo, y_lo, z_lo],
            [x_lo, y_hi, z_lo],
            [x_hi, y_lo, z_lo],
            [x_hi, y_hi, z_lo],
        ]
    )
    draw_box(task_space_corners, pb_client)
