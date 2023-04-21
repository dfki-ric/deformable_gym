import pybullet as pb
import pybullet_data
from pathlib import Path
from time import sleep

dt = 1.0 / 125.0

# Setup PyBullet:
pcid = pb.connect(pb.GUI)
pb.configureDebugVisualizer(pb.COV_ENABLE_GUI, 0, physicsClientId=pcid)
pb.configureDebugVisualizer(pb.COV_ENABLE_SHADOWS, 0, physicsClientId=pcid)
pb.configureDebugVisualizer(pb.COV_ENABLE_MOUSE_PICKING, 0, physicsClientId=pcid)
pb.setAdditionalSearchPath(pybullet_data.getDataPath(), physicsClientId=pcid)
pb.resetSimulation(pb.RESET_USE_DEFORMABLE_WORLD, physicsClientId=pcid)
pb.setTimeStep(dt, physicsClientId=pcid)
pb.setGravity(0, 0, -9.81, physicsClientId=pcid)

# Create plane:
plane = pb.loadURDF("plane.urdf", useFixedBase=True)

# Create and connect to robot:
model = "shadow_hand_on_ur5"
URDF_path = (
    f"{Path(__file__).parent.absolute()}"
    f"/april_robot_description/urdf/{model}.urdf"
)
robot_id = pb.loadURDF(
    URDF_path, useFixedBase=1,
    flags=pb.URDF_USE_SELF_COLLISION | pb.URDF_USE_SELF_COLLISION_EXCLUDE_PARENT,
    physicsClientId=pcid
    )

while True:
    pb.stepSimulation()
    sleep(dt)
