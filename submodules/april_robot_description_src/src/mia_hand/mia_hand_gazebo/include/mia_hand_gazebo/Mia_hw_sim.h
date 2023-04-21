/*
* Copyright (C) 2021 Prensilia s.r.l.
*
* Redistribution and use in source and binary forms, with or without modification,
* are permitted provided that the following conditions are met:
*
* 1. Redistributions of source code must retain the above copyright notice, this
* list of conditions and the following disclaimer.
*
* 2. Redistributions in binary form must reproduce the above copyright notice,
* this list of conditions and the following disclaimer in the documentation and/or
* other materials provided with the distribution.
*
* 3. Neither the name of the copyright holder nor the names of its contributors
* may be used to endorse or promote products derived from this software without
* specific prior written permission.
*
* THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS "AS IS" AND
* ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED TO, THE IMPLIED
* WARRANTIES OF MERCHANTABILITY AND FITNESS FOR A PARTICULAR PURPOSE ARE
* DISCLAIMED. IN NO EVENT SHALL THE COPYRIGHT HOLDER OR CONTRIBUTORS BE LIABLE
* FOR ANY DIRECT, INDIRECT, INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL
* DAMAGES (INCLUDING, BUT NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR
* SERVICES; LOSS OF USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER
* CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT LIABILITY,
* OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN ANY WAY OUT OF THE USE
* OF THIS SOFTWARE, EVEN IF ADVISED OF THE POSSIBILITY OF SUCH DAMAGE.
*/

#ifndef MIA_HW_SIM_H
#define MIA_HW_SIM_H

// ros_control
#include <control_toolbox/pid.h>
#include <hardware_interface/joint_command_interface.h>
#include <hardware_interface/robot_hw.h>
#include <joint_limits_interface/joint_limits.h>
#include <joint_limits_interface/joint_limits_interface.h>
#include <joint_limits_interface/joint_limits_rosparam.h>
#include <joint_limits_interface/joint_limits_urdf.h>

// Gazebo
#include <gazebo/common/common.hh>
#include <gazebo/physics/physics.hh>
#include <gazebo/gazebo.hh>

// ROS
#include <ros/ros.h>
#include <angles/angles.h>
#include <pluginlib/class_list_macros.h>

// gazebo_ros_control
#include <gazebo_ros_control/robot_hw_sim.h> // Inheriting the base class

// URDF
#include <urdf/model.h>

// PassiveJoint Thumb_opp + Index_fle
#include "mia_hand_description/mia_thumb_opp_passivejoints.h"


namespace mia
{

    /**
     * Struct to store the indexes of each joints in the MiaHWSim Class
     * Index of each joint depends on the order of trasmission defined in the urdf file
     */
    struct Joint_index_mapping
    {
    	int j_thumb_opp;  /**< Index number of the thumb_opp joint. */
    	int j_thumb_flex; /**< Index number of the thumb_fle joint. */
    	int j_index_flex; /**< Index number of the index_fle joint. */
    	int j_mrl_flex;   /**< Index number of the mrl_fle (medium) joint. */
    	int j_mrl_2;      /**< Index number of the mrl_2 (ring) joint. */
    	int j_mrl_3;      /**< Index number of the mrl_3 (little) joint. */
    };


    /**
     * Class hardware sim interface of the simulated Mia hand.
     * This class inherits from public gazebo_ros_control::RobotHWSim and it can
     * be used also when the Mia hand is mounted on another robots that work with
     * the gazebo_ros_control::DefaultRobotHWSim class. In this case this class
     * is able to control both the simulated mia hand and the other robot components.
     */
    class MiaHWSim : public gazebo_ros_control::RobotHWSim
    {

     public:

      /**
        * Initialize the simulated robot hardware.
        * Initialize the simulated robot hardware by register the correct ros control
        * interfaces and the joint limits.
        * @param robot_namespace Robot namespace.
        * @param model_nh Model node handle.
        * @param parent_model Parent model.
        * @param urdf_model URDF model.
        * @param transmissions Transmissions.
        * @return True if the simulated robot hardware is initialized successfully, False if not.
        */
        virtual bool initSim( const std::string& robot_namespace,
                              ros::NodeHandle model_nh,
                              gazebo::physics::ModelPtr parent_model,
                              const urdf::Model *const urdf_model,
                              std::vector<transmission_interface::TransmissionInfo> transmissions);

      /**
        * Read state data from the simulated robot hardware.
        * Read joint positions velocities and efforts from the simulated robot hardware.
        * @param time Simulation time.
        * @param period Time since the last simulation step.
        */
        virtual void readSim(ros::Time time, ros::Duration period);

      /**
        * Send commands to the simulated robot hardware.
        * Send command as joint positions or velocities or efforts to the simulated robot hardware.
        * @param time Simulation time.
        * @param period Time since the last simulation step.
        */
        virtual void writeSim(ros::Time time, ros::Duration period);

        virtual void eStopActive(const bool active); //!< Unused.

     protected:

        /**
        * Joints control methods.
        * All possible joints controlled methods allowed by ROS control
        */
        enum ControlMethod {EFFORT, POSITION, POSITION_PID, VELOCITY, VELOCITY_PID};

      /**
        * Register the joint limits of the simulated hardware.
        * Register the limits of the joint specified by joint_name and joint_handle. The limits are
        * retrieved from joint_limit_nh. If urdf_model is not NULL, limits are retrieved from it also.
        * @param joint_name Name of the joint whose limits has to be registered.
        * @param joint_handle Handler of the joint.
        * @param ctrl_method Array of enum describe the control methods available for that joint.
        * @param joint_limit_nh Joint node handle.
        * @param urdf_model URDF model.
        * @param joint_type Joint type (e.g. urdf::Joint::REVOLUTE).
        * @param lower_limit Lower position limit of the joint returned by this method.
        * @param upper_limit Upper position limit of the joint returned by this method.
        * @param effort_limit Effortlimit of the joint returned by this method.
        */
        void registerJointLimits(const std::string& joint_name,
                                 const std::vector<hardware_interface::JointHandle>& joint_handle,
                                 const std::vector<ControlMethod> ctrl_method,
                                 const ros::NodeHandle& joint_limit_nh,
                                 const urdf::Model *const urdf_model,
                                 int *const joint_type, double *const lower_limit,
                                 double *const upper_limit, double *const effort_limit);

      /**
        * Evaluate the position of the Mia thumb opposition joint.
        * Since in the actual Mia hand the opposition of the thumb (passive joint)
        * is coupled to the movement of the index flexion this function uses
        * the sate of the index flexion joint of the simulated mia hand
        * to evaluate the position that the simulated thumb opposition joint has to
        * reach at every simulation step.
        * @param period Time since the last simulation step.
        * @return The target position of the Mia simulated thumb opposition joint.
        */
        double GetThumbOppPosition(ros::Duration period);


      /**
        * Evaluate if any of the Mia simulated middle ring or little flexion joint is stuck.
        * Since in the actual Mia hand the middle, little and ring flexion JointState
        * are coupled, while in the Mia simulated model are not (since they have
        * different rotation axis) this fucntion has the aim to avoid that the difference
        * among the position of these joints exceeds a certain threshold, simulating
        * the behaviour of coupled joints.
        * @param joint_position_m Position of the Mia simulated middle flexion joint.
        * @param joint_position_r Position of the Mia simulated ring flexion joint.
        * @param joint_position_l Position of the Mia simulated little flexion joint.
        * @param flex_direction direction of the ongoing flexion movement.
        * @return True if the difference among the position of these joints exceeds a certain threshold (e.g when any joint is stuck).
        */
        bool GetMiddlePosition(double joint_position_m, double joint_position_r, double joint_position_l, double flex_direction);

      /**
        * Select the control method to move the simulated hardware.
        * Since velocity and position interfaces are both loaded for each joint,
        * this method recognize wich type of control the received new command requires.
        * @param Default_joint_control_methods_ Default control method for the joint.
        * @param last_joint_control_methods_ Last type of control method used for the joint.
        * @param last_joint_velocity_command_ Last velocity command received for the joint.
        * @param joint_velocity_command_ Actual velocity command received for the joint.
        * @param last_joint_position_command_ Last position command received for the joint.
        * @param joint_position_command_ Actual position command received for the joint.
        * @return the control method to use in the current simulation step to move the joint.
        */
        enum ControlMethod SelectCtrMethod(	enum MiaHWSim::ControlMethod Default_joint_control_methods_,
                                              enum ControlMethod last_joint_control_methods_,
         					                            const double last_joint_velocity_command_,
		    				                              const double joint_velocity_command_,
		    				                              const double last_joint_position_command_,
		    		                                  const double joint_position_command_);


      /**
        * Joint_index_mapping struct.
        * Struct that stores the index number of the simulated mia hand joint as contained
        * in the interfaces and arrays of this class
        * @see Joint_index_mapping()
        */
 	      Joint_index_mapping joints_ii;

        unsigned int n_dof_; //!< Number of degree of freedom of the simulated hardware.
        unsigned int write_counter;  //!< Counter of the write method.


        hardware_interface::JointStateInterface    js_interface_; //!< Interface for simulated joint state.
        hardware_interface::EffortJointInterface   ej_interface_; //!< Interface for simulated joint effort commands.
        hardware_interface::PositionJointInterface pj_interface_; //!< Interface for simulated joint position commands.
        hardware_interface::VelocityJointInterface vj_interface_; //!< Interface for simulated joint velocity commands.

        joint_limits_interface::EffortJointSaturationInterface   ej_sat_interface_;    //!< Interface for simulated joint effort saturation limit.
        joint_limits_interface::EffortJointSoftLimitsInterface   ej_limits_interface_; //!< Interface for simulated joint effort soft limit.
        joint_limits_interface::PositionJointSaturationInterface pj_sat_interface_;    //!< Interface for simulated joint position saturation limit.
        joint_limits_interface::PositionJointSoftLimitsInterface pj_limits_interface_; //!< Interface for simulated joint position sodt limit.
        joint_limits_interface::VelocityJointSaturationInterface vj_sat_interface_;   //!< Interface for simulated joint velocity saturation limit.
        joint_limits_interface::VelocityJointSoftLimitsInterface vj_limits_interface_;//!< Interface for simulated joint velocity soft limit.


        std::vector<std::string> joint_names_;    //!< Name of the joints of the simulated hardware.
        std::vector<int> joint_types_;            //!< Type of the joints of the simulated hardware.
        std::vector<double> joint_lower_limits_;  //!< Lower position limits of the joints of the simulated hardware.
        std::vector<double> joint_upper_limits_;  //!< Upper position limits of the joints of the simulated hardware.
        std::vector<double> joint_effort_limits_; //!< Effort upper limits of the joints of the simulated hardware.

        std::vector<double> joint_position_;      //!< Actual position state of the joints of the simulated hardware.
        std::vector<double> joint_velocity_;      //!< Actual velocity state of the joints of the simulated hardware.
        std::vector<double> joint_effort_;        //!< Actual effort state of the joints of the simulated hardware.
        std::vector<double> joint_effort_command_;//!< Actual effort command of the joints of the simulated hardware.
        std::vector<double> last_joint_effort_command_; //!< Last effort command of the joints of the simulated hardware.
        std::vector<double> joint_position_command_; //!< Actual position command of the joints of the simulated hardware.
        std::vector<double> last_joint_position_command_; //!< Last position command of the joints of the simulated hardware.
        std::vector<double> joint_velocity_command_; //!< Actual velocity command of the joints of the simulated hardware.
        std::vector<double> last_joint_velocity_command_;//!< Last velocity command of the joints of the simulated hardware.
        std::vector<ControlMethod> last_joint_control_methods_; //!< Last control method  used for the joints of the simulated hardware.

        std::vector<ControlMethod> joint_control_methods_; //!< Actual control method  for the joints of the simulated hardware.
        std::vector<std::vector<ControlMethod>> List_joint_control_methods_; //!< List of control method available for the joints of the simulated hardware.

        std::vector<control_toolbox::Pid> pid_controllers_;
        //

        double* j_index_flex_pos;    //!< Pointer to the position status of Mia simulated index flexion joint.
        double  j_index_flex_pos_Th; //!< Threshold used for the simulated index flexion position control to switch its sign.
        int 	  j_index_flex_sign;   //!< Current sign of the Mia simulated index flexion joint.
        double jThOpp_Target_position; //!< Target position of the Mia simulated thumb opposition joint evaluated by this class.
        double jMiddle_StackTarget_position; //!< Target position of the Mia simulated middle flexion joint when ring and/or little get stuck.
        bool   mrl_united; //!< If True then middle ring little fingers are physical connected in the URDF, False if they ate independent joints.

      /**
        * Class used to evaluate the target position of the Mia simulated thumb opposition joint.
        * Class describing the relation between the position  of the Mia index flexion
        * joint and the dependent one of the thumb opposition. This class is implemented in the
        * mia_hand_description pkg.
        * @see GetThumbOppPosition()
        */
        mia_hand::thumb_opp_passive_joint MyTh_opp_passiveJoint;


        std::vector<gazebo::physics::JointPtr> sim_joints_; //!< Gazebo joint pointer.


        bool e_stop_active_, last_e_stop_active_;//!< Unused.

    };

typedef boost::shared_ptr<MiaHWSim> MiaHWSimPtr;

}


#endif
