import pybullet as pb
import pybullet_data
from pybullet_utils import bullet_client as bc

from ..helpers import pybullet_helper as pbh
from ..robots.bullet_robot import BulletRobot


class BulletSimulation:
    """Interface to PyBullet API.

    :param time_delta: Time between steps.
    :param mode: PyBullet connection mode.
    :param gravity: Gravitation along z-axis (positive upwards).
    :param soft: Deformable world.
    :param real_time: Real-time simulation.
    :param verbose_dt: Time interval after which debug information is printed.
    :param pybullet_options: Options that should be passed to PyBullet
    connection command.
    """

    def __init__(
        self,
        time_delta: float = 0.001,
        mode: int = pb.GUI,
        gravity: float = -9.81,
        soft: bool = False,
        real_time: bool = False,
        verbose_dt: float = 0.01,
        pybullet_options: str = "",
    ):

        self.time_delta = time_delta
        self.mode = mode
        self.gravity = gravity
        self.soft = soft
        self.real_time = real_time

        with pbh.stdout_redirected():
            self.pb_client = bc.BulletClient(
                connection_mode=self.mode, options=pybullet_options
            )
        self.pb_client.setAdditionalSearchPath(pybullet_data.getDataPath())

        self.timing = BulletTiming(
            pb_client=self.pb_client,
            dt=time_delta,
            verbose_dt=verbose_dt,
        )

        self.reset()

        self.camera = BulletCamera(self.pb_client)

    def reset(self):
        """Reset and initialize simulation."""

        with pbh.stdout_redirected():
            if self.soft:
                self.pb_client.resetSimulation(pb.RESET_USE_DEFORMABLE_WORLD)
            else:
                self.pb_client.resetSimulation()

        self.pb_client.setGravity(0, 0, self.gravity)
        self.pb_client.setRealTimeSimulation(self.real_time)
        self.pb_client.setTimeStep(self.time_delta)

        self.pb_client.configureDebugVisualizer(pb.COV_ENABLE_RENDERING, 1)
        self.pb_client.configureDebugVisualizer(pb.COV_ENABLE_GUI, 0)

    def add_robot(self, robot: BulletRobot):
        """Add robot to this simulation.

        If the same type of robot is already connected, all of its subsystems
        will be removed and overwritten by this new robot instance.

        :param robot: Robot that should be simulated.
        """
        for name, sys in robot.subsystems.items():
            if name in self.timing.subsystems:
                self.timing.remove_subsystem(name)
            self.timing.add_subsystem(name, sys[0], sys[1])

    def step_to_trigger(self, trigger_name: str):
        """Run simulation until an event is triggered.

        :param trigger_name: Name of the event trigger for which we wait.
        """
        self.timing.step()
        triggers = self.timing.get_triggered_subsystems()

        while trigger_name not in triggers:
            self.timing.step()
            triggers = self.timing.get_triggered_subsystems()

    def simulate_time(self, time):
        """Simulate for a given time without control input.

        :param time: Amount of time in seconds to simulate.
        """
        for _ in range(int(time / self.time_delta)):
            self.timing.step()

    def disconnect(self) -> None:
        """Shut down physics client instance."""
        self.pb_client.disconnect()


class BulletTiming:
    """This class handles all timing issues for a single BulletSimulation."""

    def __init__(
        self,
        pb_client: bc.BulletClient,
        dt: float = 0.001,
        verbose_dt: float = 0.01,
    ):
        """
        Create new BulletTiming instance.

        :param dt: The time delta used in the BulletSimulation.
        :param verbose_dt: Time after we print debug info.
        :param pb_client: PyBullet instance ID.
        """

        # initialise values
        self.dt = dt
        self.verbose_dt = verbose_dt
        self._pb_client = pb_client
        self.time_step = 0
        self.sim_time = 0.0

        self.subsystems = {}

    def add_subsystem(self, name, frequency, callback=None):
        """
        Adds a new (robot) subsystem to the timing module.

        .. warning::

            This function does not overwrite an already existing subsystem.

        :param name: The name of the subsystem to be added.
        :param frequency: The frequency of the subsystem to be added. (in Hertz)
        :param callback: The callback function to be called when the subsystem
        is triggered.
        """
        if name not in self.subsystems.keys():
            self.subsystems[name] = (
                max(1, round(1.0 / frequency / self.dt)),
                callback,
            )

    def remove_subsystem(self, name):
        """
        Removes a (robot) subsystem from the timing module.

        :param name: The name of the subsystem to be removed.
        """
        if name in self.subsystems.keys():
            del self.subsystems[name]

    def get_triggered_subsystems(self):
        triggered_systems = []
        for name, sys in self.subsystems.items():
            if self.time_step % sys[0] == 0:
                triggered_systems.append(name)

        return triggered_systems

    def _run_callbacks(self, systems):
        for name in systems:
            if self.subsystems[name][1] is not None:
                self.subsystems[name][1]()

    def step(self):
        """
        Performs a single time step in the simulation and triggers subsystems
        if necessary.
        """
        triggers = self.get_triggered_subsystems()
        self._run_callbacks(triggers)
        self._pb_client.stepSimulation()
        self.time_step += 1
        self.sim_time += self.dt

        if (self.sim_time % self.verbose_dt) < self.dt:
            print(
                f"Step: {self.time_step}, Time: {self.sim_time}, "
                f"Triggers: {triggers}"
            )

    def reset(self):
        self.time_step = 0
        self.sim_time = 0.0


class BulletCamera:
    """This class handles all camera operations for one BulletSimulation."""

    def __init__(
        self,
        pb_client: bc.BulletClient,
        position: tuple = (0, 0, 0),
        pitch: int = -52,
        yaw: int = 30,
        distance: int = 3,
    ):
        self.position = position
        self.pitch = pitch
        self.yaw = yaw
        self.distance = distance
        self.pb_client = pb_client

        self._active = False
        self._logging_id = None

        self.pb_client.resetDebugVisualizerCamera(
            distance, yaw, pitch, position
        )

    def start_recording(self, path):
        if not self._active:
            self._logging_id = self.pb_client.startStateLogging(
                pb.STATE_LOGGING_VIDEO_MP4, path
            )
            return self._logging_id
        else:
            return None

    def stop_recording(self):
        if self._active:
            self.pb_client.stopStateLogging(self._logging_id)

    def reset(self, position, pitch, yaw, distance):
        self.position = position
        self.pitch = pitch
        self.yaw = yaw
        self.distance = distance

        self.pb_client.resetDebugVisualizerCamera(
            distance, yaw, pitch, position
        )
