import numpy as np
import pybullet as pb
from pytransform3d.rotations import quaternion_wxyz_from_xyzw, matrix_from_quaternion
from pytransform3d.transformations import transform_from, transform, vectors_to_points, invert_transform
from deformable_gym.robots.bullet_utils import draw_box


class RigidBag:
    def __init__(self, world_pos=(0, 0, 0), world_rot=(0, 0, 0), bag_height=0.022,
                 transparent=True, fixed=False):
        base_height = 0.003
        top_height = 0.02
        total_length = 0.34
        total_width = 0.14
        compartment_length = 0.3
        compartment_width = 0.1

        total_half_length = total_length / 2
        total_half_width = total_width / 2
        compartment_half_length = compartment_length / 2
        back_half_length = total_half_length - compartment_half_length
        compartment_half_width = compartment_width / 2
        side_width = (total_half_width - compartment_half_width) / 2
        side1_y = -(side_width + compartment_half_width)
        side2_y = side_width + compartment_half_width
        back_x = -(total_half_length - back_half_length)

        base = pb.createCollisionShape(
            pb.GEOM_BOX, halfExtents=(total_half_length, total_half_width, base_height / 2))
        side = pb.createCollisionShape(
            pb.GEOM_BOX, halfExtents=(total_half_length, side_width, bag_height / 2))
        back = pb.createCollisionShape(
            pb.GEOM_BOX, halfExtents=(back_half_length, compartment_half_width, bag_height / 2))
        top = pb.createCollisionShape(
            pb.GEOM_BOX, halfExtents=(total_half_length, total_half_width, top_height / 2))
        linkCollisionShapeIndices = [side, side, back, top]
        linkMasses = [0.1] * len(linkCollisionShapeIndices)
        linkVisualShapeIndices = [-1] * len(linkCollisionShapeIndices)
        linkPositions = [[0, side1_y, base_height / 2 + bag_height / 2],
                         [0, side2_y, base_height / 2 + bag_height / 2],
                         [back_x, 0, base_height / 2 + bag_height / 2],
                         [0, 0, base_height / 2 + bag_height + top_height / 2]]
        linkOrientations = [[0, 0, 0, 1]] * len(linkCollisionShapeIndices)
        linkInertialFramePositions = [[0, 0, 0]] * len(linkCollisionShapeIndices)
        linkInertialFrameOrientations = [[0, 0, 0, 1]] * len(linkCollisionShapeIndices)
        linkParentIndices = [0] * len(linkCollisionShapeIndices)
        linkJointTypes = [pb.JOINT_FIXED] * len(linkCollisionShapeIndices)
        linkJointAxis = [(0, 0, 1)] * len(linkCollisionShapeIndices)

        self.bag = pb.createMultiBody(
            baseMass=1000,
            baseCollisionShapeIndex=base,
            baseVisualShapeIndex=-1,
            basePosition=world_pos,
            baseOrientation=pb.getQuaternionFromEuler(world_rot),
            linkMasses=linkMasses,
            linkCollisionShapeIndices=linkCollisionShapeIndices,
            linkVisualShapeIndices=linkVisualShapeIndices,
            linkPositions=linkPositions,
            linkOrientations=linkOrientations,
            linkInertialFramePositions=linkInertialFramePositions,
            linkInertialFrameOrientations=linkInertialFrameOrientations,
            linkParentIndices=linkParentIndices,
            linkJointTypes=linkJointTypes,
            linkJointAxis=linkJointAxis,
            flags=pb.URDF_MERGE_FIXED_LINKS
        )
        pb.changeDynamics(
            self.bag, -1, lateralFriction=1, spinningFriction=1, rollingFriction=1,
            restitution=0, contactStiffness=1000000, contactDamping=0.1)
        bag_color = (1, 1, 1, 0.5)
        if transparent:
            for i in range(-1, len(linkCollisionShapeIndices)):
                pb.changeVisualShape(self.bag, i, rgbaColor=bag_color)

        if fixed:
            pb.createConstraint(
                self.bag, -1, -1, -1, pb.JOINT_FIXED, [0, 0, 1], [0, 0, 0],
                world_pos)

        z_lo = base_height / 2
        z_hi = z_lo + bag_height
        x_lo = back_half_length + back_x
        x_hi = total_half_length
        y_lo = side1_y + side_width
        y_hi = -y_lo
        self.limits = [[x_lo, x_hi], [y_lo, y_hi], [z_lo, z_hi]]
        self.corners_in_bag = np.array([
            [x_lo, y_lo, z_hi],
            [x_lo, y_hi, z_hi],
            [x_hi, y_lo, z_hi],
            [x_hi, y_hi, z_hi],
            [x_lo, y_lo, z_lo],
            [x_lo, y_hi, z_lo],
            [x_hi, y_lo, z_lo],
            [x_hi, y_hi, z_lo],
        ])

    def compartment_corners(self):
        pos, orn = pb.getBasePositionAndOrientation(self.bag)
        bag2world = transform_from(
            p=pos, R=matrix_from_quaternion(quaternion_wxyz_from_xyzw(orn)))
        corners_in_world = transform(
            bag2world, vectors_to_points(self.corners_in_bag))[:, :3]
        return corners_in_world

    def draw_compartment_corners(self, line_ids=None):
        corners_in_world = self.compartment_corners()
        return draw_box(corners_in_world, line_ids)

    def transform_world2bag(self, points_in_world):
        pos, orn = pb.getBasePositionAndOrientation(self.bag)
        bag2world = transform_from(
            p=pos, R=matrix_from_quaternion(quaternion_wxyz_from_xyzw(orn)))
        world2bag = invert_transform(bag2world)
        points_in_bag = transform(world2bag, vectors_to_points(points_in_world))[:, :3]
        return points_in_bag
