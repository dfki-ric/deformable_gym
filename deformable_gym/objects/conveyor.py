from pybullet_utils import bullet_client as bc

from ..objects.bullet_object import BoxObject, ObjectFactory


class Conveyor:
    def __init__(
        self,
        pb_client: bc.BulletClient,
        height_m=1,
        conveyor_pos=(0, 1.5, 0.5),
        conveyor_rot=(0, 0, 0),
        insole_pos=(-0.1, 0.9, 1.02),
        insole_rot=(0.0, 0.0, -1.57),
        E=100000.0,
    ):
        self.height_m = height_m
        self.width_m = 0.5
        self.length_m = 1.5
        self.pb_client = pb_client

        self.conveyor = BoxObject(
            self.pb_client,
            half_extents=[
                self.width_m / 2,
                self.length_m / 2,
                self.height_m / 2,
            ],
            world_pos=conveyor_pos,
            world_orn=conveyor_rot,
            fixed=True,
            lateralFriction=0.3,
            rollingFriction=0.3,
        )

        self.object_factory = ObjectFactory(self.pb_client)
        self.insole, self.insole_pos, self.insole_orn = (
            self.object_factory.create(
                "insole2",
                object_position=insole_pos,
                object_orientation=insole_rot,
                E=E,
            )
        )
        self.insole.remove_anchors()

    def reset_insole(self, new_pos=None, new_orn=None):
        if new_pos is not None:
            self.insole_pos = new_pos
        if new_orn is not None:
            self.insole_orn = new_orn
        object2world = self.object_factory.get_pose(
            "insole2", self.insole_pos, self.insole_orn
        )[2]
        self.insole.reset(object_markers2world=object2world)
        self.insole.remove_anchors()
