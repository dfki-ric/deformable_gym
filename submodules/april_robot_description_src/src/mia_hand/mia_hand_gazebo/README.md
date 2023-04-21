# mia_hand_gazebo
Package for simulating Mia Hand in Gazebo environment, interfacing the simulated hardware with ROS Control.
The cpp class Mia_hw_sim expand the gazebo class gazebo_ros_control/DefaultRobotHWSim and allow to simulate the non linear behaviour of the Mia Hand coupled fingers.
If the simulated Mia hand is attached to another robot the Mia_hw_sim class controls the joints of the additional robot as the gazebo_ros_control/DefaultRobotHWSim does.
This allow to use Mia_hw_sim with every robot.

To be use add the MiaHWSim plugin lib in the gazebo ros control plugin lines as shown in the following example:


    < gazebo >
        <plugin name="gazebo_ros_control" filename="libgazebo_ros_control.so">
            <robotNamespace> $(arg robotNamespace) </robotNamespace>
            <robotSimType> mia/MiaHWSim </robotSimType>
        </plugin>
    <gazebo>

In the folder config some simple controllers and trajectories controllers are provided.
Please note that these controllers can be used in the robotNamespace "/". If the exploited robotnamespace is different adjust the namespace by declaring the robotNamespace argumentes in the launch file listed below.
By default robotNamespace:=mia_hand_sim.

To launch the mia hand simulation you can use one of the following commands (please use robotNamespace arg to specify the namespace, by default robotNamespace:=mia_hand_sim):

  - to launch simulation with one controller (velocity controller as default) for each DoF of the Mia Hand:

        roslaunch mia_hand_bringup mia_hand_Sim.launch

  - to launch simulation with one controller for each DoF of the Mia Hand specifying its type (in the exaple Position controllers for the thumb, Effort controllers for the index and Velocity controller for the mrl fingers:

        roslaunch mia_hand_bringup mia_hand_Sim.launch hw_interface_thumb_fle:=PositionJointInterface controller_thumb_fle:=position hw_interface_index_fle:=EffortJointInterface controller_index_fle:=eff hw_interface_mrl_fle:=VelocityJointInterface controller_mrl_fle:=velocity

  - to launch simulation with trajectory controller (velocity controller as default):

         roslaunch mia_hand_bringup mia_hand_Sim_traj.launch

  - to launch simulation with trajectory controller specifying the type
   - (for example velocity):

         roslaunch mia_hand_bringup mia_hand_Sim_traj.launch tc_type:=vel eff_interface:=false

   - (for example position):

         roslaunch mia_hand_bringup mia_hand_Sim_traj.launch tc_type:=pos eff_interface:=false

   - Note that: Effort control is not implemented in the real Mia hand.

Please note that to these launch files load the mia joint transmission YAML
configuration file (stored in the mia_hand_description/calibration folder)
to the parameter server.
