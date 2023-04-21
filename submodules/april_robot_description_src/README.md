# Robot Description Repository

This repository contains the robot description models for the APRIL project in the form of `urdf` and `xacro` files.

This is a branch dedicated for the standalone generation of the URDF models for PyBullet. This had to be separated because the PyBullet and the Gazebo simulation have different requirements with regards to the URDF models in the specific case of virtual links.

Some of the models included such virtual links without information about the inertial values of the link (which is required by Gazebo). For such links, PyBullet sets a default value of 1.0, which alters the dynamics of the model in simulation. Therefore, the inertial values of virtual links have to be set to 0.0 explicitly for the PyBullet models.

In this branch, we use the [`universal_robot` version with inertial values for the virtual links](https://github.com/aprilprojecteu/universal_robot/tree/pybullet).

## Setup

To generate the URDF files for PyBullet, this repository is supposed to be setup in a ROS environment (i.e., catkin workspace). The following steps describe the minimal setup (assuming all dependencies on the ROS ecosystem are already met, for information on these dependencies checkout the [wiki repository](https://github.com/aprilprojecteu/wiki)).

1. Clone the code. This already creates the file structure for the catkin workspace including submodules in the `src` directory on which the models depend.
   ```shell
   git clone -b pybullet --recurse-submodules https://github.com/aprilprojecteu/april_robot_description.git
   ```
2. Build the project.
   ```shell
   source <path to your ROS setup.bash/setup.zsh/...>
   cd april_robot_description
   catkin init && catkin build
   ```

Now, you can alter the models (`xacro` files) and regenerate the URDF files for PyBullet.

> :grey_exclamation: For the models including the Shadow hand, this repository uses the packages provided by Shadow. This also include virtual links without any default interial values. If you regenerate these URDF files, make sure to add default values to these virtual links afterwards.

## Visualizing URDFs in PyBullet

For quickly visualizing the URDFs in PyBullet you can use [`test_urdf.py`](./test/test_urdf.py). Just set the URDF you want to visualize and run the script.

## Already Generated URDFs

If you are only after the already generated URDF files, you can get them [here](./src/april_robot_description/urdf/).
