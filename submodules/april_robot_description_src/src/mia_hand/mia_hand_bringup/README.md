mia_hand_bringup
====================

This package contains the launch files of the mia hand.

- to launch the node drive of the real Mia hand and send serial commands directly to the hand please use:

      roslaunch mia_hand_bringup mia_hand_driver_node.launch

  See readme file and documentation of the mia_hand_driver pkg for further details.

- to launch the control loop (frequency 20Hz), the hardware interface of the real Mia Hand  with ros controllers please use:

      roslaunch mia_hand_bringup mia_hand_hw.launch
      roslaunch mia_hand_bringup mia_hand_hw_traj.launch
      roslaunch mia_hand_bringup mia_hand_hw_moveit.launch

  See readme file and documentation of the mia_hand_ros_control pkg for further details.
  Please note that to these launch files load the mia joint transmission YAML
  configuration file (stored in the mia_hand_description/calibration folder)
  to the parameter server.

- to launch the control the simulation of the Mia hand within Gazebo with controllers please use:

        roslaunch mia_hand_bringup mia_hand_Sim.launch
        roslaunch mia_hand_bringup mia_hand_Sim_traj.launch

  See readme file and documentation of the mia_hand_gazebo pkg for further details.
  Please note that to these launch files load the mia joint transmission YAML
  configuration file (stored in the mia_hand_description/calibration folder)
  to the parameter server.

  Please, in order to properly control the mia hand hardware, set the serial low_latency flag:

        setserial /dev/<tty_name> low_latency
