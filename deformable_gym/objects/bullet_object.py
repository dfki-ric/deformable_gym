import abc
import warnings
from collections.abc import Sequence
from importlib.resources import as_file, files
from typing import List, Tuple, Union

import numpy as np
import numpy.typing as npt
import pybullet as pb
import pytransform3d.rotations as pr
import pytransform3d.transformations as pt
from pybullet_utils import bullet_client as bc

from ..helpers import pybullet_helper as pbh
from ..robots.bullet_utils import draw_pose

INSOLE_PATH = None
with as_file(
    files("deformable_gym.assets.objects").joinpath("insole.vtk")
) as vtk_path:
    INSOLE_PATH = str(vtk_path)

PILLOW_PATH = None
with as_file(
    files("deformable_gym.assets.objects").joinpath("pillow_small.vtk")
) as vtk_path:
    PILLOW_PATH = str(vtk_path)


class BulletObjectBase(abc.ABC):
    """Base class of objects that can be loaded in bullet.

    :param client_id: Physics client ID for PyBullet interface calls.
    """

    object_id: int
    pb_client: bc.BulletClient

    def __init__(self, pb_client: bc.BulletClient):
        self.pb_client = pb_client

    def get_pose(self):
        """Returns the object's current pose.

        :return: 7D numpy array containing position and quaternion of the form
            [x, y, z, qw, qx, qy, qz]
        """
        pos, rot = self.pb_client.getBasePositionAndOrientation(self.object_id)
        return pbh.merge_pose(pos, rot)

    def get_vertices(self):
        """Returns a list of the object vertices.

        :return: list of vertices
        """
        return self.pb_client.getMeshData(self.object_id)[1]

    def get_id(self):
        """Get UUID of object."""
        return self.object_id

    @abc.abstractmethod
    def _load_object(self):
        """Load object in simulation."""

    @abc.abstractmethod
    def remove_anchors(self):
        """Remove anchors of fixed object."""


class PositionEulerAngleMixin:
    """Controls pose of an object through position and Euler angles."""

    init_pos: npt.ArrayLike
    init_orn: npt.ArrayLike

    def _set_init_pose(self, world_pos=None, world_orn=None):
        if world_pos is None:
            self.init_pos = [0, 0, 0]
        else:
            assert len(world_pos) == 3
            self.init_pos = world_pos

        if world_orn is None:
            self.init_orn = pb.getQuaternionFromEuler((0, 0, 0))
        else:
            assert len(world_orn) == 3
            self.init_orn = pb.getQuaternionFromEuler(world_orn)

    def reset(self):
        """Resets the object to its initial pose."""
        self.pb_client.resetBasePositionAndOrientation(
            self.object_id, self.init_pos, self.init_orn
        )


class RigidPrimitiveObject(PositionEulerAngleMixin, BulletObjectBase):
    """Simple primitive object base."""

    def __init__(
        self,
        pb_client: bc.BulletClient,
        mass: float = 1.0,
        world_pos=None,
        world_orn=None,
        fixed=False,
        lateralFriction=None,
        rollingFriction=None,
        restitution=None,
        contactStiffness=None,
        contactDamping=None,
    ):
        super().__init__(pb_client)
        self.mass = mass
        self.lateralFriction = lateralFriction
        self.rollingFriction = rollingFriction
        self.restitution = restitution
        self.contactStiffness = contactStiffness
        self.contactDamping = contactDamping
        self.fixed = fixed

        self._set_init_pose(world_pos, world_orn)
        self.object_id = self._load_object()
        self.anchored = False

    @abc.abstractmethod
    def _create_primitive(self):
        """Load primitive collision shape."""

    def _load_object(self):
        primitive = self._create_primitive()

        self.object = self.pb_client.createMultiBody(
            baseMass=self.mass,
            baseCollisionShapeIndex=primitive,
            baseVisualShapeIndex=-1,
            basePosition=self.init_pos,
            baseOrientation=self.init_orn,
        )

        if self.fixed:
            self.anchor = self.pb_client.createConstraint(
                self.object,
                -1,
                -1,
                -1,
                pb.JOINT_FIXED,
                [0, 0, 1],
                [0, 0, 0],
                self.init_pos,
            )
            self.anchored = True

        dynamics_config = {}
        if self.lateralFriction is not None:
            dynamics_config["lateralFriction"] = self.lateralFriction
        if self.rollingFriction is not None:
            dynamics_config["rollingFriction"] = self.rollingFriction
        if self.restitution is not None:
            dynamics_config["restitution"] = self.restitution
        if self.contactStiffness is not None:
            dynamics_config["contactStiffness"] = self.contactStiffness
        if self.contactDamping is not None:
            dynamics_config["contactDamping"] = self.contactDamping
        if dynamics_config:
            self.pb_client.changeDynamics(self.object, -1, **dynamics_config)

        return self.object

    def remove_anchors(self):
        if self.fixed and self.anchored:
            self.pb_client.removeConstraint(self.anchor)
            self.anchored = False

    def reset(self):
        """Resets the object to its initial position by respawning it."""
        self.remove_anchors()
        assert not self.anchored
        self.pb_client.removeBody(self.object)
        self._load_object()


class MeshObject(RigidPrimitiveObject):
    """Simple mesh object."""

    def __init__(
        self,
        filename,
        pb_client: bc.BulletClient,
        world_pos=None,
        world_orn=None,
        mass=1.0,
        scale=(1, 1, 1),
        fixed=False,
        lateralFriction=None,
        rollingFriction=None,
        restitution=None,
        contactStiffness=None,
        contactDamping=None,
    ):
        self.filename = filename
        self.scale = scale
        super().__init__(
            pb_client=pb_client,
            mass=mass,
            world_pos=world_pos,
            world_orn=world_orn,
            fixed=fixed,
            lateralFriction=lateralFriction,
            rollingFriction=rollingFriction,
            restitution=restitution,
            contactStiffness=contactStiffness,
            contactDamping=contactDamping,
        )

    def _create_primitive(self):
        return self.pb_client.createCollisionShape(
            pb.GEOM_MESH, fileName=self.filename, meshScale=self.scale
        )


class BoxObject(RigidPrimitiveObject):
    """Simple box object."""

    def __init__(
        self,
        pb_client: bc.BulletClient,
        half_extents=(1, 1, 1),
        mass=1.0,
        world_pos=None,
        world_orn=None,
        fixed=False,
        lateralFriction=None,
        rollingFriction=None,
        restitution=None,
        contactStiffness=None,
        contactDamping=None,
        client_id=0,
    ):
        self.half_extents = half_extents
        super().__init__(
            pb_client=pb_client,
            mass=mass,
            world_pos=world_pos,
            world_orn=world_orn,
            fixed=fixed,
            lateralFriction=lateralFriction,
            rollingFriction=rollingFriction,
            restitution=restitution,
            contactStiffness=contactStiffness,
            contactDamping=contactDamping,
        )

    def _create_primitive(self):
        return self.pb_client.createCollisionShape(
            pb.GEOM_BOX, halfExtents=self.half_extents
        )


class SphereObject(RigidPrimitiveObject):
    """Simple sphere object."""

    def __init__(
        self,
        pb_client: bc.BulletClient,
        radius=1.0,
        mass=1.0,
        world_pos=None,
        world_orn=None,
        fixed=False,
        lateralFriction=None,
        rollingFriction=None,
        restitution=None,
        contactStiffness=None,
        contactDamping=None,
        client_id=0,
    ):
        self.radius = radius
        super().__init__(
            pb_client=pb_client,
            mass=mass,
            world_pos=world_pos,
            world_orn=world_orn,
            fixed=fixed,
            lateralFriction=lateralFriction,
            rollingFriction=rollingFriction,
            restitution=restitution,
            contactStiffness=contactStiffness,
            contactDamping=contactDamping,
        )

    def _create_primitive(self):
        return self.pb_client.createCollisionShape(
            pb.GEOM_SPHERE, radius=self.radius
        )


class CylinderObject(RigidPrimitiveObject):
    """Simple cylinder object."""

    def __init__(
        self,
        pb_client: bc.BulletClient,
        radius=1.0,
        height=1.0,
        mass=1.0,
        world_pos=None,
        world_orn=None,
        fixed=False,
        lateralFriction=None,
        rollingFriction=None,
        restitution=None,
        contactStiffness=None,
        contactDamping=None,
    ):
        self.radius = radius
        self.height = height
        super().__init__(
            pb_client=pb_client,
            mass=mass,
            world_pos=world_pos,
            world_orn=world_orn,
            fixed=fixed,
            lateralFriction=lateralFriction,
            rollingFriction=rollingFriction,
            restitution=restitution,
            contactStiffness=contactStiffness,
            contactDamping=contactDamping,
        )

    def _create_primitive(self):
        return self.pb_client.createCollisionShape(
            pb.GEOM_CYLINDER, radius=self.radius, height=self.height
        )


class CapsuleObject(RigidPrimitiveObject):
    """Simple capsule object."""

    def __init__(
        self,
        pb_client: bc.BulletClient,
        radius=1.0,
        height=1.0,
        mass=1.0,
        world_pos=None,
        world_orn=None,
        fixed=False,
        lateralFriction=None,
        rollingFriction=None,
        restitution=None,
        contactStiffness=None,
        contactDamping=None,
    ):
        self.radius = radius
        self.height = height
        super().__init__(
            pb_client=pb_client,
            mass=mass,
            world_pos=world_pos,
            world_orn=world_orn,
            fixed=fixed,
            lateralFriction=lateralFriction,
            rollingFriction=rollingFriction,
            restitution=restitution,
            contactStiffness=contactStiffness,
            contactDamping=contactDamping,
        )

    def _create_primitive(self):
        return self.pb_client.createCollisionShape(
            pb.GEOM_CAPSULE, radius=self.radius, height=self.height
        )


class UrdfObject(PositionEulerAngleMixin, BulletObjectBase):
    """Base Bullet Object interface.

    Provides some basic meta functionality for URDF-based objects.
    """

    def __init__(
        self,
        filename,
        pb_client: bc.BulletClient,
        verbose=0,
        world_pos=None,
        world_orn=None,
        fixed=False,
    ):
        super().__init__(pb_client=pb_client)
        self.filename = filename
        self.verbose = verbose
        self.fixed = fixed

        self._set_init_pose(world_pos, world_orn)
        self.object_id = self._load_object()

    def _load_object(self):
        return self.pb_client.loadURDF(
            self.filename,
            self.init_pos,
            self.init_orn,
            useFixedBase=self.fixed,
            flags=pb.URDF_USE_SELF_COLLISION
            | pb.URDF_USE_SELF_COLLISION_EXCLUDE_PARENT,
        )

    def remove_anchors(self):
        if self.fixed:
            raise NotImplementedError("Base is fixed.")


class SoftObjectBase(BulletObjectBase):
    filename: str
    fixed: bool
    fixed_nodes: list[int]
    scale: float
    nu: float
    E: float
    damping: float
    collision_margin: float
    repulsion_stiffness: float
    mass: float
    friction_coefficient: float
    init_pos: npt.ArrayLike
    init_orn: npt.ArrayLike
    constraints: Sequence[int]

    def __init__(
        self,
        filename,
        pb_client: bc.BulletClient,
        fixed=False,
        fixed_nodes=None,
        scale=1.0,
        nu=0.2,
        E=100000.0,
        damping=0.005,
        collision_margin=0.0005,
        repulsion_stiffness=8000.0,
        mass=0.1,
        friction_coefficient=0.5,
    ):
        super().__init__(pb_client=pb_client)
        self.filename = filename
        self.fixed = fixed
        self.fixed_nodes = fixed_nodes
        self.scale = scale
        self.nu = nu
        self.E = E
        self.damping = damping
        self.collision_margin = collision_margin
        self.repulsion_stiffness = repulsion_stiffness
        self.mass = mass
        self.friction_coefficient = friction_coefficient

    def _load_object(self):
        mu, lmbda = self.__lame_parameters(self.nu, self.E)

        with pbh.stdout_redirected():
            object_id = self.pb_client.loadSoftBody(
                self.filename,
                self.init_pos,
                self.init_orn,
                scale=self.scale,
                simFileName=self.filename,
                useNeoHookean=True,
                NeoHookeanMu=mu,
                NeoHookeanLambda=lmbda,
                NeoHookeanDamping=self.damping,
                collisionMargin=self.collision_margin,
                useSelfCollision=True,
                useFaceContact=True,
                repulsionStiffness=self.repulsion_stiffness,
                mass=self.mass,
                frictionCoeff=self.friction_coefficient,
            )

        if self.fixed:
            self.__make_anchors(object_id)
        else:
            self.constraints = []

        return object_id

    @staticmethod
    def __lame_parameters(nu, E):
        mu = 0.5 * E / (1.0 + nu)
        lmbda = E * nu / ((1.0 + nu) * (1.0 - 2.0 * nu))
        return mu, lmbda

    def __make_anchors(self, object_id):
        if self.fixed_nodes is None:
            warnings.warn(
                "Object should be fixed, but no fixed nodes are given."
            )
            self.constraints = []
        else:
            self.constraints = [
                self.pb_client.createSoftBodyAnchor(
                    softBodyBodyUniqueId=object_id,
                    nodeIndex=i,
                    bodyUniqueId=-1,
                    linkIndex=-1,
                )
                for i in self.fixed_nodes
            ]
            for constraint in self.constraints:
                self.pb_client.changeConstraint(constraint, maxForce=1)

    def remove_anchors(self):
        for anchor in self.constraints:
            self.pb_client.removeConstraint(anchor)


class SoftObject(PositionEulerAngleMixin, SoftObjectBase):
    """Soft Bullet Object."""

    def __init__(
        self,
        filename,
        pb_client: bc.BulletClient,
        world_pos=None,
        world_orn=None,
        fixed=False,
        scale=1.0,
        nu=0.2,
        E=100000.0,
        damping=0.005,
        repulsion_stiffness=8000.0,
        mass=0.1,
        fixed_nodes=None,
    ):
        super().__init__(
            filename=filename,
            pb_client=pb_client,
            fixed=fixed,
            fixed_nodes=fixed_nodes,
            scale=scale,
            nu=nu,
            E=E,
            damping=damping,
            repulsion_stiffness=repulsion_stiffness,
            mass=mass,
        )
        self._set_init_pose(world_pos, world_orn)
        self.object_id = self._load_object()

    def reset(self, pos=None, orn=None):
        """
        Resets the object to its initial position by removing and respawning it.
        This way the initial state of the soft-body is obtained as well.
        """
        # remove constraints
        if self.fixed:
            for constraint in self.constraints:
                self.pb_client.removeConstraint(constraint)

        # remove the body
        self.pb_client.removeBody(self.object_id)

        # if necessary, set new position and orientation
        if pos is not None:
            self.init_pos = pos

        if orn is not None:
            self.init_orn = orn

        self.object_id = self._load_object()


class MocapObjectMixin:
    def reset(self, object_markers2world=None, center_camera=False, pos=None):
        """
        Resets the object to its initial position by removing and respawning it.
        This way the initial state of the soft-body is obtained as well.
        """
        if object_markers2world is not None:
            self.object_markers2world = object_markers2world
        if pos is not None:
            self.object_markers2world[:3, 3] = pos
        if self.fixed:
            self.remove_anchors()
        self.pb_client.removeBody(self.object_id)
        self.init_pos, self.init_orn = self.mesh_pose(self.object_markers2world)
        self.object_id = self._load_object()
        if center_camera:
            self.pb_client.resetDebugVisualizerCamera(
                0.35, -60, -30, self.object_markers2world[:3, 3]
            )


class Insole(MocapObjectMixin, SoftObjectBase):
    def __init__(
        self,
        insole_markers2world,
        pb_client: bc.BulletClient,
        scale=1.0,
        E=100000.0,
        fixed=False,
    ):
        super().__init__(
            INSOLE_PATH,
            pb_client=pb_client,
            fixed=fixed,
            fixed_nodes=[0, 40, 45],
            scale=scale,
            nu=0.2,
            E=E,
            damping=0.005,
            collision_margin=0.0005,
            repulsion_stiffness=8000.0,
            mass=0.1,
            friction_coefficient=1.5,
        )
        self.object_markers2world = insole_markers2world
        self.init_pos, self.init_orn = self.mesh_pose(self.object_markers2world)
        self.object_id = self._load_object()

    @staticmethod
    def mesh_pose(object_markers2world):
        """
        Returns the mesh pose.
        """
        markers2mesh = pt.transform_from(
            R=pr.active_matrix_from_extrinsic_roll_pitch_yaw(
                np.deg2rad([180, 0, -4.5])
            ),
            p=np.array([0.04, 0.07, -0.007]),
        )
        pq = pt.pq_from_transform(
            pt.concat(pt.invert_transform(markers2mesh), object_markers2world)
        )

        return pq[:3], pr.quaternion_xyzw_from_wxyz(pq[3:])


class PillowSmall(MocapObjectMixin, SoftObjectBase):
    def __init__(
        self,
        pillow_markers2world,
        pb_client: bc.BulletClient,
        scale=1.0,
        fixed=False,
    ):
        super().__init__(
            PILLOW_PATH,
            pb_client=pb_client,
            fixed=fixed,
            fixed_nodes=[0, 10, 20, 30, 40, 50, 60, 70, 80, 90, 100, 110, 120],
            scale=scale,
            nu=0.2,
            E=10000.0,
            damping=0.005,
            collision_margin=0.0001,
            repulsion_stiffness=1000.0,
            mass=0.1,
            friction_coefficient=1.5,
        )
        self.object_markers2world = pillow_markers2world
        self.init_pos, self.init_orn = self.mesh_pose(self.object_markers2world)
        self.object_id = self._load_object()

    @staticmethod
    def mesh_pose(insole_markers2world):
        markers2mesh = pt.transform_from(
            R=pr.active_matrix_from_extrinsic_roll_pitch_yaw(
                np.deg2rad([0, 0, 90])
            ),
            p=np.array([0.0, -0.02, 0.095]),
        )

        pq = pt.pq_from_transform(
            pt.concat(pt.invert_transform(markers2mesh), insole_markers2world)
        )

        return pq[:3], pr.quaternion_xyzw_from_wxyz(pq[3:])


class InsoleOnConveyorBelt(Insole):
    def __init__(
        self,
        insole_markers2world,
        pb_client: bc.BulletClient,
        grasp_point_name="back",
        scale=1.0,
        E=100000.0,
    ):
        self.conveyor = None
        assert grasp_point_name in ["front", "middle", "back"]
        self.grasp_point_name = grasp_point_name
        super().__init__(
            insole_markers2world=insole_markers2world,
            pb_client=pb_client,
            scale=scale,
            E=E,
            fixed=False,
        )

    def _load_object(self):
        assert not self.fixed
        object_id = super()._load_object()

        # extents: width, length, height
        conveyor_extents = np.array([0.5, 1.5, self.init_pos[2] - 0.02])

        if self.grasp_point_name == "back":
            conveyor_pos = (0, self.init_pos[1] + 0.88, self.init_pos[2] / 2)
        elif self.grasp_point_name == "front":
            conveyor_pos = (0, self.init_pos[1] - 0.59, self.init_pos[2] / 2)
        else:
            raise NotImplementedError("We would need two boxes for this.")
        self.conveyor = BoxObject(
            pb_client=self.pb_client,
            half_extents=0.5 * conveyor_extents,
            world_pos=conveyor_pos,
            world_orn=(0, 0, 0),
            fixed=True,
            lateralFriction=0.3,
            rollingFriction=0.3,
        )
        return object_id

    def remove_anchors(self):
        if self.conveyor is not None:
            self.pb_client.removeBody(self.conveyor.get_id())
            self.conveyor = None

    def reset(self, object_markers2world=None, center_camera=False, pos=None):
        """
        Resets the object to its initial position by removing and respawning it.
        This way the initial state of the soft-body is obtained as well.
        """
        if object_markers2world is not None:
            self.object_markers2world = object_markers2world
        if pos is not None:
            self.object_markers2world[:3, 3] = pos

        self.remove_anchors()

        self.pb_client.removeBody(self.object_id)
        self.init_pos, self.init_orn = self.mesh_pose(self.object_markers2world)
        self.object_id = self._load_object()
        if center_camera:
            self.pb_client.resetDebugVisualizerCamera(
                0.35, -60, -30, self.object_markers2world[:3, 3]
            )


class ObjectFactory:
    """Creates objects to grasp."""

    OBJECT_POSITIONS = {
        "insole": [-0.15, 0.1, 1.03],
        "pillow_small": [0.00, 0.29, 1.05],
        "insole2": [0.0, 0.12, 1.03],
        "pillow_small2": [0.00, 0.29, 1.05],
        "insole_on_conveyor_belt": [0.0, 0.12, 1.03],
        "box": [0.0, 0.14, 1.035],
        "sphere": [0.0, 0.15, 1.02],
        "cylinder": [0.0, 0.15, 1.02],
        "capsule": [0.0, 0.15, 1.02],
    }
    OBJECT_ORIENTATIONS = {
        "insole": [0, 0, 0],
        "pillow_small": [0.5, 0, 0],
        "insole2": [0, 0, np.pi / 2],
        "pillow_small2": [0, 0, 0],
        "insole_on_conveyor_belt": [0, 0, np.pi / 2],
        "box": [0, 0, 0],
        "sphere": [0, 0, 0],
        "cylinder": [0, 0, 0],
        "capsule": [0, 0, 0],
    }

    def __init__(self, pb_client: bc.BulletClient):
        self.pb_client = pb_client

    def create(
        self,
        object_name: str,
        object_position: Union[npt.ArrayLike, None] = None,
        object_orientation: Union[npt.ArrayLike, None] = None,
        object2world: Union[npt.ArrayLike, None] = None,
        **additional_args,
    ) -> tuple[BulletObjectBase, np.ndarray, np.ndarray]:
        """Create object to grasp.

        :param object_name: Name of the object. Must be one of 'insole',
        'pillow_small', 'insole2', 'pillow_small2', 'box', 'sphere',
        'cylinder', 'capsule', or 'insole_on_conveyor_belt/<grasp_point_name>'.
        :param object_position: Position of the object.
        :param object_orientation: Orientation of the object given as extrinsic
        xyz Euler angles.
        :param object2world: Pose of the object (alternative to position and
        orientation.
        :param additional_args: Additional arguments passed to the constructor
        of the object. This might overwrite default values.
        :return: Object, position, and orientation (extrinsic xyz Euler
        angles).
        :raises:
            KeyError: If object is unknown.
        """
        object_position, object_orientation, object2world = self.get_pose(
            object_name, object_position, object_orientation, object2world
        )

        if object_name == "insole":
            args = dict(
                fixed=True, mass=0.1, E=200000.0, fixed_nodes=[0, 40, 45]
            )
            args.update(additional_args)
            object_to_grasp = SoftObject(
                INSOLE_PATH,
                self.pb_client,
                world_pos=object_position,
                world_orn=object_orientation,
                **args,
            )
        elif object_name == "pillow_small":
            args = dict(
                fixed=True, mass=0.5, nu=0.1, E=20000.0, fixed_nodes=[0, 40, 45]
            )
            args.update(additional_args)
            object_to_grasp = SoftObject(
                PILLOW_PATH,
                self.pb_client,
                world_pos=object_position,
                world_orn=object_orientation,
                **args,
            )
        elif object_name == "insole2":
            args = dict(scale=1.0, fixed=True)
            args.update(additional_args)
            object_to_grasp = Insole(object2world, self.pb_client, **args)
        elif object_name == "pillow_small2":
            args = dict(scale=1.0, fixed=True)
            args.update(additional_args)
            object_to_grasp = PillowSmall(object2world, self.pb_client, **args)
        elif object_name.startswith("insole_on_conveyor_belt"):
            # TODO hacked special case, refactor this
            args = dict(scale=1.0)
            args.update(additional_args)
            args["grasp_point_name"] = object_name.split("/")[-1]
            object_to_grasp = InsoleOnConveyorBelt(
                object2world, self.pb_client, **args
            )
        elif object_name == "box":
            args = dict(half_extents=(0.1, 0.02, 0.02), mass=0.1, fixed=True)
            args.update(additional_args)
            object_to_grasp = BoxObject(
                self.pb_client,
                world_pos=object_position,
                world_orn=object_orientation,
                **args,
            )
        elif object_name == "sphere":
            args = dict(radius=0.05, mass=0.1, fixed=True)
            args.update(additional_args)
            object_to_grasp = SphereObject(
                self.pb_client,
                world_pos=object_position,
                world_orn=object_orientation,
                **args,
            )
        elif object_name == "cylinder":
            args = dict(radius=0.025, height=0.05, mass=0.1, fixed=True)
            args.update(additional_args)
            object_to_grasp = CylinderObject(
                self.pb_client,
                world_pos=object_position,
                world_orn=object_orientation,
                **args,
            )
        elif object_name == "capsule":
            args = dict(radius=0.025, height=0.05, mass=0.1, fixed=True)
            args.update(additional_args)
            object_to_grasp = CapsuleObject(
                self.pb_client,
                world_pos=object_position,
                world_orn=object_orientation,
                **args,
            )
        else:
            raise KeyError(f"Object '{object_name}' not available.")

        return object_to_grasp, object_position, object_orientation

    def get_pose(
        self,
        object_name: str,
        object_position: Union[npt.ArrayLike, None] = None,
        object_orientation: Union[npt.ArrayLike, None] = None,
        object2world: Union[npt.ArrayLike, None] = None,
    ) -> tuple[np.ndarray, np.ndarray, np.ndarray]:
        """Translates between pose representations.

        :param object_name: Name of the object. Must be one of 'insole',
        'pillow_small', 'insole2', 'pillow_small2', 'box', 'sphere',
        'cylinder', 'capsule', or 'insole_on_conveyor_belt/<grasp_point_name>'.
        :param object_position: Position of the object.
        :param object_orientation: Orientation of the object given as extrinsic
        xyz Euler angles.
        :param object2world: Pose of the object (alternative to position and
        orientation.
        :return: Object, position, and orientation (extrinsic xyz Euler
        angles), and pose (transformation matrix).
        """
        if "/" in object_name:  # TODO hacked special case, refactor this
            object_name = object_name.split("/")[0]

        if object2world is None:
            if object_position is None:
                object_position = np.copy(self.OBJECT_POSITIONS[object_name])
            if object_orientation is None:
                object_orientation = np.copy(
                    self.OBJECT_ORIENTATIONS[object_name]
                )

            object2world = pt.transform_from(
                R=pr.active_matrix_from_extrinsic_euler_xyz(object_orientation),
                p=object_position,
            )
        else:
            object_position = object2world[:3, 3]
            object_orientation = pr.extrinsic_euler_xyz_from_active_matrix(
                object2world[:3, :3]
            )
        return object_position, object_orientation, object2world


class Pose:
    def __init__(
        self,
        position,
        orientation,
        pb_client: bc.BulletClient,
        scale=0.1,
        line_width=10,
    ):
        self.position = position
        self.orientation = orientation
        self.scale = scale
        self.line_width = line_width
        self.pb_client = pb_client
        self.ids = draw_pose(
            self.position,
            self.orientation,
            s=self.scale,
            lw=self.line_width,
            pb_client=self.pb_client,
        )

    def update(self, position, orientation):
        self.position = position
        self.orientation = orientation
        draw_pose(
            self.position,
            self.orientation,
            s=self.scale,
            lw=self.line_width,
            replace_item_unique_ids=self.ids,
            pb_client=self.pb_client,
        )

    def remove(self):
        for item in self.ids:
            self.pb_client.removeUserDebugItem(item)
        del self
